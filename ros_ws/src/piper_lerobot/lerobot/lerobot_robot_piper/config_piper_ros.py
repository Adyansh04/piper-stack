"""The configuration of LeRobot's ``piper_ros`` robot."""

from __future__ import annotations

from dataclasses import dataclass

from lerobot.robots import RobotConfig


@RobotConfig.register_subclass('piper_ros')
@dataclass
class PiperRosConfig(RobotConfig):
    """The Piper through the stack: piper_lerobot's bridge node serves it over ZMQ."""

    #: The bridge's endpoint.
    endpoint: str = 'tcp://127.0.0.1:5590'
    #: True: send no actions, while the stack's own teleoperation moves the arm (recording).
    passive: bool = False
    #: How long a reply of the bridge may take, in s.
    timeout_s: float = 1.0
    #: Rows of the wrist camera's colour image (the camera driver's 640 x 480 mode).
    image_height: int = 480
    #: Columns of the wrist camera's colour image.
    image_width: int = 640
