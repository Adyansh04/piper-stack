"""LeRobot's ``piper_ros`` robot: the Piper through the stack's bridge node."""

from __future__ import annotations

from typing import Any

from lerobot.robots import Robot

from .bridge import BridgeClient
from .config_piper_ros import PiperRosConfig

#: The arm's joints and the gripper, in the bridge's order; positions in rad, the gripper's in m.
JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper')
#: The wrist camera's key: ``observation.images.wrist`` in a dataset.
CAMERA = 'wrist'


class PiperRos(Robot):
    """The Piper through the stack: joint positions and the wrist image in, joint targets out.

    Actions go to the bridge, which turns them into MoveIt Servo joint jogs (collision and limit
    checked) and gripper goals; a passive robot sends none.

    Args:
        config: The robot's configuration.
    """

    config_class = PiperRosConfig
    name = 'piper_ros'

    def __init__(self, config: PiperRosConfig) -> None:
        super().__init__(config)
        self.config = config
        self._bridge: BridgeClient | None = None

    @property
    def observation_features(self) -> dict:
        """Each joint's position (float) and the wrist image (height, width, 3)."""
        image = (self.config.image_height, self.config.image_width, 3)
        return {**{f'{joint}.pos': float for joint in JOINTS}, CAMERA: image}

    @property
    def cameras(self) -> dict[str, str]:
        """The cameras by name, with their dataset keys (LeRobot's scripts count them)."""
        return {CAMERA: f'observation.images.{CAMERA}'}

    @property
    def action_features(self) -> dict:
        """Each joint's target position (float)."""
        return {f'{joint}.pos': float for joint in JOINTS}

    @property
    def is_connected(self) -> bool:
        """Whether the bridge answered on connect."""
        return self._bridge is not None

    def connect(self, calibrate: bool = True) -> None:
        """Reach the bridge and check that its image has the configured size.

        Raises:
            TimeoutError: If the bridge does not answer.
            RuntimeError: If the bridge has no joint state or image yet.
            ValueError: If the camera's image has another size than configured.
        """
        bridge = BridgeClient(self.config.endpoint, self.config.timeout_s)
        try:
            shape = bridge.observe().image.shape[:2]
        except Exception:
            bridge.close()
            raise
        if shape != (self.config.image_height, self.config.image_width):
            bridge.close()
            raise ValueError(
                f'The wrist image is {shape[0]} x {shape[1]}, the robot is configured for '
                f'{self.config.image_height} x {self.config.image_width}'
            )
        self._bridge = bridge

    @property
    def is_calibrated(self) -> bool:
        """Always: the stack's controllers own the arm's calibration."""
        return True

    def calibrate(self) -> None:
        """Nothing to do: the stack's controllers own the arm's calibration."""

    def configure(self) -> None:
        """Nothing to do: the stack configures the arm."""

    def get_observation(self) -> dict[str, Any]:
        """Return each joint's measured position and the wrist image."""
        observation = self._bridge.observe()
        positions = dict(
            zip((f'{joint}.pos' for joint in JOINTS), map(float, observation.measured), strict=True)
        )
        return {**positions, CAMERA: observation.image}

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Send the joint targets to the bridge, unless passive; return the action as given."""
        if not self.config.passive:
            self._bridge.act([float(action[f'{joint}.pos']) for joint in JOINTS])
        return action

    def disconnect(self) -> None:
        """Stop the arm, unless passive, and close the connection."""
        if self._bridge is None:
            return
        try:
            if not self.config.passive:
                self._bridge.release()
        finally:
            self._bridge.close()
            self._bridge = None
