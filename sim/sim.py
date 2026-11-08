"""Simulates the Piper in Isaac Sim: the arm and its gripper, driven by the stack over ROS 2.

Run inside the sim image (``scripts/run/sim.sh`` does):

    /isaac-sim/python.sh /piper-stack/sim/sim.py --config /piper-stack/sim/config/sim.yaml

HEADLESS and LIVESTREAM (true or false), PHYSICS_ENGINE (physx or newton), SIM_MODE (control or
shadow), TASK (a task under tasks/) and TASK_SEED override app.headless, app.livestream.enabled,
app.engine, app.mode, world.task and world.task_seed. Python because Isaac Sim's scripting API is
Python.
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import threading

import yaml

# Enabled before the simulator is imported: rclpy comes with the ROS 2 bridge.
EXTENSIONS = ('isaacsim.ros2.core', 'isaacsim.ros2.bridge')
# Newton and its backend for the physics tensors the joint loop reads the arm through.
NEWTON_EXTENSIONS = ('isaacsim.physics.newton', 'isaacsim.physics.newton.tensors')
ENGINES = ('physx', 'newton')
MODES = ('control', 'shadow')


def load_config(path: str) -> dict:
    """Return sim.yaml with the HEADLESS, LIVESTREAM, PHYSICS_ENGINE, SIM_MODE and TASK overrides.

    Raises:
        ValueError: If the engine is neither physx nor newton, the mode neither control nor shadow,
            or the livestream is on without headless (the stream replaces the window).
    """
    with open(path) as f:
        cfg = yaml.safe_load(f)
    if os.getenv('HEADLESS'):
        cfg['app']['headless'] = os.getenv('HEADLESS') == 'true'
    if os.getenv('PHYSICS_ENGINE'):
        cfg['app']['engine'] = os.getenv('PHYSICS_ENGINE')
    if os.getenv('LIVESTREAM'):
        cfg['app']['livestream']['enabled'] = os.getenv('LIVESTREAM') == 'true'
    if os.getenv('SIM_MODE'):
        cfg['app']['mode'] = os.getenv('SIM_MODE')
    if os.getenv('TASK'):
        cfg['world']['task'] = os.getenv('TASK')
    if os.getenv('TASK_SEED'):
        cfg['world']['task_seed'] = int(os.getenv('TASK_SEED'))
    if cfg['app']['engine'] not in ENGINES:
        raise ValueError(f'app.engine {cfg["app"]["engine"]!r} is not one of {ENGINES}')
    if cfg['app']['mode'] not in MODES:
        raise ValueError(f'app.mode {cfg["app"]["mode"]!r} is not one of {MODES}')
    if cfg['app']['livestream']['enabled'] and not cfg['app']['headless']:
        raise ValueError('The livestream replaces the local window: it needs HEADLESS=true')
    return cfg


def start_livestream(app, stream: dict) -> None:
    """Stream the app's UI over WebRTC on the ports of sim.yaml's app.livestream."""
    import isaacsim.core.experimental.utils.app as app_utils

    prefix = '/exts/omni.kit.livestream.app/primaryStream'
    app.set_setting(f'{prefix}/signalPort', int(stream['signal_port']))
    app.set_setting(f'{prefix}/streamPort', int(stream['stream_port']))
    if stream['public_ip']:
        app.set_setting(f'{prefix}/publicIp', str(stream['public_ip']))
    # The remote user's pointer is drawn into the stream.
    app.set_setting('/app/window/drawMouse', True)
    app_utils.enable_extension('omni.kit.livestream.app')


def main() -> None:
    """Start Isaac Sim and run the simulator until its window closes or the process is stopped."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default='/piper-stack/sim/config/sim.yaml')
    args, _ = parser.parse_known_args()
    cfg = load_config(args.config)
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter('[%(name)s] %(levelname)s: %(message)s'))
    logger = logging.getLogger('sim')
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    logger.propagate = False
    # A container stop's SIGTERM and Ctrl-C end the run loop, so the simulator shuts down in order.
    stop = threading.Event()
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: stop.set())

    from isaacsim import SimulationApp

    engine = cfg['app']['engine']
    stream = cfg['app']['livestream']
    app = SimulationApp(
        {
            'headless': cfg['app']['headless'],
            # The stream shows the UI, which a headless app otherwise does not build.
            'hide_ui': not stream['enabled'],
            'renderer': 'RayTracedLighting',
            # The engine is chosen at start, and Newton's extension must not switch to itself.
            'extra_args': [
                f'--/exts/isaacsim.core.simulation_manager/default_engine={engine}',
                '--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
            ],
        }
    )
    exit_code = 0
    try:
        import isaacsim.core.experimental.utils.app as app_utils

        if stream['enabled']:
            start_livestream(app, stream)
            logger.info(
                f'Livestream on: WebRTC signalling {stream["signal_port"]}/tcp, media '
                f'{stream["stream_port"]}/udp'
            )
        for ext in EXTENSIONS:
            app_utils.enable_extension(ext)
        if engine == 'newton':
            for ext in NEWTON_EXTENSIONS:
                app_utils.enable_extension(ext)
        app.update()
        import simulator

        simulator.run(app, cfg, stop)
    except Exception:
        # Kit's fast shutdown ends the process in close(): the traceback must be logged first.
        logger.exception('The simulator failed')
        exit_code = 1
    finally:
        app.close(exit_code=exit_code)


if __name__ == '__main__':
    main()
