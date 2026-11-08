"""The simulated arm's URDF: cleaning it for the importer, reading frames from it, its USD import.

The URDF helpers need only numpy and the standard library, so the tests import them without Isaac
Sim; :func:`import_robot` runs inside Isaac Sim. Lengths in metres, angles in radians.
"""

from __future__ import annotations

import hashlib
import logging
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

log = logging.getLogger('sim.robot')

#: How the URDF is imported: part of the cache key, so a change re-imports the robot.
IMPORT_SETTINGS = {'merge_fixed_joints': True, 'fix_base': True, 'joint_drive_type': 'force'}
#: Finished conversions kept in the cache.
KEEP_CONVERSIONS = 3
ISAAC_VERSION_FILE = Path('/isaac-sim/VERSION')


def clean_urdf(xml: str) -> str:
    """Return the URDF without what the importer must not see.

    ros2_control, gazebo and transmission elements go (the stack's controllers drive the joints
    over topics), and so do mimic tags (read them first with :func:`mimic_of`): the simulator
    drives a mimicking joint itself.
    """
    root = ET.fromstring(xml)
    for element in list(root):
        if element.tag in ('ros2_control', 'gazebo', 'transmission'):
            root.remove(element)
    for joint in root.findall('joint'):
        for mimic in joint.findall('mimic'):
            joint.remove(mimic)
    return ET.tostring(root, encoding='unicode')


def mimic_of(xml: str, joint_name: str) -> tuple[str, float, float] | None:
    """Return the joint a joint mimics, with the multiplier and offset, or None.

    Its position is multiplier x the mimicked joint's position + offset.
    """
    for joint in ET.fromstring(xml).findall('joint'):
        mimic = joint.find('mimic')
        if joint.get('name') == joint_name and mimic is not None:
            return (
                mimic.get('joint'),
                float(mimic.get('multiplier', '1')),
                float(mimic.get('offset', '0')),
            )
    return None


def origin_matrix(element: ET.Element) -> np.ndarray:
    """Return the 4 x 4 transform of an element's ``origin`` (identity without one)."""
    origin = element.find('origin')
    attrs = origin.attrib if origin is not None else {}
    xyz = [float(v) for v in attrs.get('xyz', '0 0 0').split()]
    roll, pitch, yaw = (float(v) for v in attrs.get('rpy', '0 0 0').split())
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    T = np.eye(4)
    # URDF rpy: fixed-axis roll about x, then pitch about y, then yaw about z.
    T[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    T[:3, 3] = xyz
    return T


def frame_in_body(xml: str, frame: str) -> tuple[str, np.ndarray]:
    """Return the link a frame is merged into by the import, and the frame's transform in it.

    The importer merges links joined by fixed joints, so a sensor frame ends up inside the first
    link above it that a moving joint carries.
    """
    joints = {j.find('child').get('link'): j for j in ET.fromstring(xml).findall('joint')}
    T = np.eye(4)
    link = frame
    while link in joints and joints[link].get('type') == 'fixed':
        T = origin_matrix(joints[link]) @ T
        link = joints[link].find('parent').get('link')
    return link, T


def _mesh_digest(xml: str, packages: dict[str, str]) -> str:
    """Return a digest of the mesh files a URDF references, by path and content."""
    digest = hashlib.sha1()
    for mesh in sorted({m.get('filename') for m in ET.fromstring(xml).iter('mesh')}):
        path = mesh
        for name, root in packages.items():
            path = path.replace(f'package://{name}', root)
        digest.update(mesh.encode())
        if Path(path).is_file():
            digest.update(hashlib.sha1(Path(path).read_bytes()).digest())
    return digest.hexdigest()


def prune_cache(cache: Path, keep: int) -> None:
    """Delete all but the newest ``keep`` finished conversions in the cache."""
    finished = sorted(
        (d for d in cache.iterdir() if (d / 'done').is_file()),
        key=lambda d: (d / 'done').stat().st_mtime,
        reverse=True,
    )
    for old in finished[keep:]:
        log.info('Removing the old conversion %s', old)
        shutil.rmtree(old, ignore_errors=True)


def drive_gain(values: float | dict[str, float], joint: str) -> float:
    """Return a drive gain for a joint: one number, or the value of the first regex it matches.

    Raises:
        KeyError: If no regex matches the joint.
    """
    if not isinstance(values, dict):
        return float(values)
    for pattern, value in values.items():
        if re.fullmatch(pattern, joint):
            return float(value)
    raise KeyError(f'No drive gain matches joint {joint!r}')


def set_mujoco_gains(stage, root: str, drives: dict) -> int:
    """Give the robot's MuJoCo actuators sim.yaml's drive gains, per radian and per metre.

    Isaac Sim 6.2's URDF importer copies the PhysX drives' gains, which are per degree on revolute
    joints, into the MuJoCo variant's position actuators as they are: 57 times too soft under
    Newton. A MuJoCo position actuator's force is gain * target - gain * q - damping * qdot.

    Args:
        stage: The stage, the robot's ``mujoco`` physics variant selected.
        root: The robot's prim.
        drives: sim.yaml's robot drives (stiffness and damping per joint regex).

    Returns:
        The number of actuators set.
    """
    from pxr import Usd, Vt

    count = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.GetTypeName() != 'MjcActuator':
            continue
        joint = prim.GetRelationship('mjc:target').GetTargets()[0].name
        stiffness = drive_gain(drives['stiffness'], joint)
        damping = drive_gain(drives['damping'], joint)
        gain = list(prim.GetAttribute('mjc:gainPrm').Get())
        bias = list(prim.GetAttribute('mjc:biasPrm').Get())
        gain[0], bias[1], bias[2] = stiffness, -stiffness, -damping
        prim.GetAttribute('mjc:gainPrm').Set(Vt.DoubleArray(gain))
        prim.GetAttribute('mjc:biasPrm').Set(Vt.DoubleArray(bias))
        count += 1
    return count


def set_sdf_collision(stage, root: str, newton: dict) -> int:
    """Turn some links' collision meshes into signed distance fields (SDF) for Newton.

    The URDF importer gives every link a convex hull, which under Newton collides with no SDF
    mesh: fingers so made pass through a task's parts. The importer instances each link's collision
    mesh, so each instance is made editable before its mesh is changed.

    Args:
        stage: The stage, the robot loaded.
        root: The robot's prim.
        newton: sim.yaml's newton section: ``sdf_links``, ``sdf_resolution`` (cells along a mesh's
            longest side), ``sdf_band_m`` and ``contact_gap_m`` (metres).

    Returns:
        The number of meshes changed.
    """
    from pxr import PhysxSchema, Sdf, Usd, UsdPhysics

    links = set(newton['sdf_links'])
    # A link's collision instance shares the link's name (its look is <link>_gray).
    instances = [
        prim
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root))
        if prim.IsInstance()
        and prim.GetName() in links
        and prim.GetParent().GetName() == prim.GetName()
    ]
    real = Sdf.ValueTypeNames.Float
    count = 0
    for instance in instances:
        instance.SetInstanceable(False)
        for prim in Usd.PrimRange(instance):
            if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                continue
            UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Set('sdf')
            PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim).CreateSdfResolutionAttr(
                int(newton['sdf_resolution'])
            )
            prim.AddAppliedSchema('NewtonSDFCollisionAPI')
            for name, type_name, value in (
                ('sdfMaxResolution', Sdf.ValueTypeNames.Int, int(newton['sdf_resolution'])),
                ('sdfNarrowBandInner', real, -float(newton['sdf_band_m'])),
                ('sdfNarrowBandOuter', real, float(newton['sdf_band_m'])),
                ('contactGap', real, float(newton['contact_gap_m'])),
            ):
                prim.CreateAttribute(f'newton:{name}', type_name, custom=False).Set(value)
            count += 1
    return count


def import_robot(
    urdf_path: Path, cache: Path, packages: dict[str, str], drives: dict
) -> tuple[str, str]:
    """Return the robot's USD and its URDF text, converting the URDF when needed.

    A conversion is reused while the URDF, the mesh files, the drives, the import settings, the
    package paths and the Isaac Sim version stay the same. One cut off before it finished (no
    ``done`` marker) is redone. The newest :data:`KEEP_CONVERSIONS` conversions stay.

    Args:
        urdf_path: The expanded URDF (xacro output).
        cache: Directory of the conversions.
        packages: ROS package names and the directories their ``package://`` URLs resolve to.
        drives: ``target`` (position or velocity), ``stiffness`` and ``damping`` (a number or a
            ``{joint regex: value}`` mapping), as the importer takes them.

    Returns:
        The path of the USD to reference, and the URDF text (with its mimic tags).
    """
    xml = urdf_path.read_text()
    clean = clean_urdf(xml)
    name = ET.fromstring(xml).get('name')
    version = ISAAC_VERSION_FILE.read_text().strip() if ISAAC_VERSION_FILE.is_file() else ''
    key = hashlib.sha1(
        (
            clean
            + repr(drives)
            + repr(IMPORT_SETTINGS)
            + repr(packages)
            + version
            + _mesh_digest(clean, packages)
        ).encode()
    ).hexdigest()[:12]
    usd, done = cache / key / name / f'{name}.usda', cache / key / 'done'
    if usd.exists() and done.exists():
        return str(usd), xml
    (cache / key).mkdir(parents=True, exist_ok=True)
    clean_path = cache / key / f'{name}.urdf'
    clean_path.write_text(clean)

    # The importer is an Isaac Sim extension, there only once the app runs.
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    for ext in (
        'omni.scene.optimizer.core',
        'isaacsim.robot.schema',
        'isaacsim.asset.importer.urdf',
    ):
        manager.set_extension_enabled_immediate(ext, True)
    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig

    log.info('Converting the robot URDF to USD in %s', cache / key)
    URDFImporter(
        URDFImporterConfig(
            urdf_path=str(clean_path),
            usd_path=str(cache / key),
            ros_package_paths=[{'name': n, 'path': p} for n, p in packages.items()],
            joint_target_type=drives['target'],
            override_joint_stiffness=drives['stiffness'],
            override_joint_damping=drives['damping'],
            **IMPORT_SETTINGS,
        )
    ).import_urdf()
    # Written last: a start cut off mid-import redoes it.
    done.write_text(key)
    prune_cache(cache, KEEP_CONVERSIONS)
    return str(usd), xml
