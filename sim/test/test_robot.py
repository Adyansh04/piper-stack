"""The simulator's URDF helpers on the Piper's real description (no Isaac Sim needed)."""

from __future__ import annotations

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import robot
import yaml

XACRO = Path(__file__).resolve().parents[2] / 'ros_ws/src/piper_description/urdf/piper.urdf.xacro'
CONFIG = Path(__file__).resolve().parents[1] / 'config/sim.yaml'


@pytest.fixture(scope='module')
def urdf() -> str:
    return subprocess.run(['xacro', str(XACRO)], check=True, capture_output=True, text=True).stdout


def test_the_importer_sees_no_ros2_control_and_no_mimic(urdf: str) -> None:
    clean = ET.fromstring(robot.clean_urdf(urdf))
    assert clean.find('ros2_control') is None
    assert not clean.findall('joint/mimic')
    assert len(clean.findall('joint')) == len(ET.fromstring(urdf).findall('joint'))


def test_the_right_finger_mirrors_the_left(urdf: str) -> None:
    assert robot.mimic_of(urdf, 'joint8') == ('joint7', -1.0, 0.0)
    assert robot.mimic_of(urdf, 'joint7') is None


def test_the_wrist_camera_rides_on_link6_where_the_urdf_puts_it(urdf: str) -> None:
    cfg = yaml.safe_load(CONFIG.read_text())
    body, T = robot.frame_in_body(urdf, cfg['camera']['frame'])
    assert body == 'link6'
    np.testing.assert_allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-9)
    # The colour lens sits on the mount, ahead of the flange and off the gripper's axis.
    assert 0.0 < T[2, 3] < 0.2 and np.hypot(T[0, 3], T[1, 3]) > 0.03


def test_the_physics_rate_is_a_multiple_of_the_render_rate() -> None:
    app = yaml.safe_load(CONFIG.read_text())['app']
    assert app['physics_hz'] % app['render_hz'] == 0
    assert set(app['device']) == {'physx', 'newton'}


def test_a_drive_gain_is_the_first_regex_that_matches_the_joint() -> None:
    gains = {'joint[1-6]': 4000.0, 'joint[78]': 2000.0}
    assert robot.drive_gain(gains, 'joint2') == 4000.0
    assert robot.drive_gain(gains, 'joint8') == 2000.0
    assert robot.drive_gain(50.0, 'joint8') == 50.0
    with pytest.raises(KeyError):
        robot.drive_gain(gains, 'joint9')
