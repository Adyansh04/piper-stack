"""Renders a task's assets in Isaac Sim, headless, to PNG files, as the simulator has them.

Builds the simulator's world from sim.yaml on its engine (PHYSICS_ENGINE overrides it): the table,
the light and the arm in the task's preview pose. Places the task's parts at a state's poses and
lets them settle (dynamic parts stay where they are put in the goal state), then renders the task's
preview views and the overhead camera's view. Run inside the sim image:

    /isaac-sim/python.sh /piper-stack/sim/preview.py --task /piper-stack/tasks/assembly \
        --state goal --out /piper-stack/data/preview/assembly

Python because Isaac Sim's scripting API is Python.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import numpy as np
import task as task_spec

import sim

# The views' cameras: a 50 degree horizontal field of view.
VIEW_CAMERA = {'width_px': 1280, 'height_px': 720, 'fx_px': 1372.0, 'near_m': 0.01, 'far_m': 10.0}
# Path-traced frames accumulated per image (each of SAMPLES_PER_FRAME samples a pixel), denoised.
RENDER_SUBFRAMES = 8
SAMPLES_PER_FRAME = 64

log = logging.getLogger('sim.preview')


def look_at(eye: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Return the optical frame (z forward, x right, y down) of a camera at eye facing target.

    Raises:
        ValueError: If the camera would look straight up or down (no horizon to keep level).
    """
    forward = (target - eye) / np.linalg.norm(target - eye)
    right = np.cross(forward, [0.0, 0.0, 1.0])
    if np.linalg.norm(right) < 1e-6:
        raise ValueError('A preview view looks straight up or down; tilt it a little')
    right /= np.linalg.norm(right)
    T = np.eye(4)
    T[:3, :3] = np.column_stack((right, np.cross(forward, right), forward))
    T[:3, 3] = eye
    return T


def pose_arm(stage, root: str, joints: dict[str, float]) -> None:
    """Start the arm's joints at positions and hold them there with their drives.

    Args:
        stage: The stage.
        root: The robot's prim.
        joints: Joint positions by name, radians (metres for a prismatic joint).

    Raises:
        ValueError: If a joint is not in the robot.
    """
    from pxr import PhysxSchema, Usd, UsdPhysics

    found = {
        p.GetName(): p for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.IsA(UsdPhysics.Joint)
    }
    for name, position in joints.items():
        if name not in found:
            raise ValueError(f'The robot has no joint {name!r}')
        prim = found[name]
        angular = prim.IsA(UsdPhysics.RevoluteJoint)
        kind, value = ('angular', float(np.degrees(position))) if angular else ('linear', position)
        UsdPhysics.DriveAPI.Apply(prim, kind).CreateTargetPositionAttr(value)
        PhysxSchema.JointStateAPI.Apply(prim, kind).CreatePositionAttr(value)


def report_settling(spec: task_spec.Task, prims: dict[str, str], state: str, engine: str) -> None:
    """Log how far each dynamic part (and the cable's plug and middle) moved while settling.

    A part that moved far fell or was thrown out of an overlap: its pose or collision is wrong.
    """
    import omni.physics.tensors as tensors
    import omni.usd
    import task_usd

    starts = {
        name: (path, spec.states[state][name][:3, 3])
        for name, path in task_usd.body_paths(spec, prims).items()
    }
    cable = spec.cable
    if cable is not None:
        middle = len(cable.lengths_m) // 2
        segment = f'{prims["cable"]}/{task_usd.segment_name(middle)}'
        starts['cable middle'] = (segment, cable.centres[middle])
    view = tensors.create_simulation_view(
        'torch', stage_id=omni.usd.get_context().get_stage_id(), backend=engine
    )
    view.set_subspace_roots('/')
    bodies = view.create_rigid_body_view([path for path, _ in starts.values()])
    positions = bodies.get_transforms()[:, :3].cpu().numpy()
    for (name, (_, start)), position in zip(starts.items(), positions, strict=True):
        log.info('%s moved %.1f mm while settling', name, 1e3 * np.linalg.norm(position - start))


def render(app, cfg: dict, args: argparse.Namespace) -> None:
    """Build the scene, let it settle and write the images."""
    import isaacsim.core.experimental.utils.app as app_utils
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    import robot
    import sensors
    import simulator
    import task_usd
    import world
    from isaacsim.core.simulation_manager import SimulationManager
    from omni.kit.loop import _loop
    from PIL import Image
    from pxr import Gf, Sdf, UsdLux, UsdPhysics

    spec = task_spec.load(args.task)
    stage = omni.usd.get_context().get_stage()
    world.build(stage, {**cfg['world'], 'cubes': []})
    # A key light for shape and shadow; the simulator's world has the dome light only.
    sun = UsdLux.DistantLight.Define(stage, Sdf.Path('/World/KeyLight'))
    sun.CreateIntensityAttr(1500.0)
    sun.CreateAngleAttr(1.0)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(35.0, 0.0, 30.0))
    r = cfg['robot']
    usd, urdf = robot.import_robot(
        Path(r['urdf']), Path(r['usd_cache']), r['packages'], r['drives']
    )
    arm = stage.DefinePrim(Sdf.Path('/World/piper'), 'Xform')
    arm.GetReferences().AddReference(usd)
    engine = cfg['app']['engine']
    arm.GetVariantSet('Physics').SetVariantSelection(simulator.PHYSICS_VARIANTS[engine])
    if engine == 'newton':
        robot.set_mujoco_gains(stage, '/World/piper', r['drives'])
    app.update()
    pose_arm(stage, '/World/piper', spec.preview.get('arm', {}))
    files = task_usd.build(spec, Path(r['usd_cache']).parent / 'tasks')
    prims = task_usd.add(stage, spec, files, state=args.state)
    if args.state != 'start':
        # Placed parts may interlock (a thread's phase is the physics' to find): hold them still.
        for name in spec.states[args.state]:
            body = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(prims[name]))
            body.CreateKinematicEnabledAttr(True)
    cameras = {}
    for name, view in spec.preview.get('views', {}).items():
        path = f'/World/Preview/{name}'
        T = look_at(np.asarray(view['eye'], dtype=float), np.asarray(view['target'], dtype=float))
        sensors.define_camera(stage, path, T, VIEW_CAMERA)
        cameras[name] = (path, VIEW_CAMERA)
    overhead = cfg['overhead_camera']
    # The robot's base, and so its world frame, is the stage's origin.
    _, T = robot.frame_in_body(urdf, overhead['frame'])
    sensors.define_camera(stage, '/World/Preview/overhead', T, overhead)
    cameras['overhead'] = ('/World/Preview/overhead', overhead)

    physics_hz, render_hz = cfg['app']['physics_hz'], cfg['app']['render_hz']
    if engine == 'newton':
        simulator.configure_newton(cfg['newton'], physics_hz)
        SimulationManager.switch_physics_engine('newton')
    SimulationManager.setup_simulation(dt=1.0 / physics_hz, device=args.device)
    loop_runner = _loop.acquire_loop_interface()
    loop_runner.set_manual_step_size(1.0 / render_hz)
    loop_runner.set_manual_mode(True)
    app_utils.play()
    for _ in range(round(args.settle_s * render_hz)):
        app.update()
    omni.timeline.get_timeline_interface().pause()
    log.info('Settled for %.1f s of simulation time', args.settle_s)
    report_settling(spec, prims, args.state, engine)

    annotators = {}
    for name, (path, camera) in cameras.items():
        product = rep.create.render_product(
            path, (int(camera['width_px']), int(camera['height_px']))
        )
        annotators[name] = rep.AnnotatorRegistry.get_annotator('rgb')
        annotators[name].attach([product])
    rep.orchestrator.step(rt_subframes=RENDER_SUBFRAMES)
    args.out.mkdir(parents=True, exist_ok=True)
    for name, annotator in annotators.items():
        image = np.asarray(annotator.get_data())[..., :3]
        out = args.out / f'{spec.name}_{args.state}_{name}.png'
        Image.fromarray(image).save(out)
        log.info('Wrote %s', out)


def main() -> None:
    """Start Isaac Sim headless, render a task's preview and exit with 0 on success."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--task', type=Path, required=True, help="The task's directory")
    parser.add_argument('--state', choices=('start', 'goal'), default='start')
    parser.add_argument('--out', type=Path, required=True, help='Directory for the PNG files')
    parser.add_argument('--config', default='/piper-stack/sim/config/sim.yaml')
    parser.add_argument('--settle-s', type=float, default=2.0, help='Simulation time to settle')
    # SDF collision of the parts runs on the GPU pipeline.
    parser.add_argument('--device', default='cuda')
    args = parser.parse_args()
    cfg = sim.load_config(args.config)
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter('[%(name)s] %(levelname)s: %(message)s'))
    root = logging.getLogger('sim')
    root.addHandler(handler)
    root.setLevel(logging.INFO)
    root.propagate = False

    from isaacsim import SimulationApp

    engine = cfg['app']['engine']
    app = SimulationApp(
        {
            'headless': True,
            'renderer': 'PathTracing',
            'samples_per_pixel_per_frame': SAMPLES_PER_FRAME,
            'denoiser': True,
            # As in sim.py: the engine is chosen at start, Newton must not switch to itself.
            'extra_args': [
                f'--/exts/isaacsim.core.simulation_manager/default_engine={engine}',
                '--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
            ],
        }
    )
    exit_code = 0
    try:
        import isaacsim.core.experimental.utils.app as app_utils

        # world.py brings rclpy, which comes with the ROS 2 bridge.
        for ext in sim.EXTENSIONS + (sim.NEWTON_EXTENSIONS if engine == 'newton' else ()):
            app_utils.enable_extension(ext)
        app.update()
        render(app, cfg, args)
    except Exception:
        # Kit's fast shutdown ends the process in close(): the traceback must be logged first.
        log.exception('The preview failed')
        exit_code = 1
    finally:
        app.close(exit_code=exit_code)


if __name__ == '__main__':
    main()
