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
    #: The cameras observed and the size each image is given at, ``name:HEIGHTxWIDTH`` separated
    #: by commas (LeRobot's own ``cameras`` field is for its camera drivers): the wrist camera in
    #: the driver's 640 x 480 mode by default. An image the bridge serves at another size of the
    #: same aspect is scaled down to it (the overhead camera's 1280 x 720 to ``overhead:360x640``).
    images: str = 'wrist:480x640'
    #: True: each joint's effort is observed too (``joint1.effort`` ..., N m; the gripper's in N).
    effort: bool = False

    def camera_sizes(self) -> dict[str, tuple[int, int]]:
        """Return each camera's image size, (height, width) by name.

        Raises:
            ValueError: If ``images`` is not ``name:HEIGHTxWIDTH`` separated by commas.
        """
        sizes = {}
        for entry in self.images.split(','):
            name, _, size = entry.strip().partition(':')
            height, _, width = size.partition('x')
            if not name or not height.isdigit() or not width.isdigit():
                raise ValueError(f'images: {entry!r} is not name:HEIGHTxWIDTH')
            sizes[name] = (int(height), int(width))
        return sizes
