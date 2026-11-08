"""LeRobot's ``piper_stack`` teleoperator: the arm's commanded joint positions, as actions.

The operator drives the arm with the stack's own teleoperation (the keyboard jog through MoveIt
Servo, the gripper keys); this teleoperator reads what that commands, through piper_lerobot's
bridge node, so that ``lerobot-record`` with a passive ``piper_ros`` robot records it as the action.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot_robot_piper.bridge import BridgeClient
from lerobot_robot_piper.piper_ros import JOINTS


@TeleoperatorConfig.register_subclass('piper_stack')
@dataclass
class PiperStackConfig(TeleoperatorConfig):
    """The stack's own teleoperation, read through the bridge node."""

    #: The bridge's endpoint.
    endpoint: str = 'tcp://127.0.0.1:5590'
    #: How long a reply of the bridge may take, in s.
    timeout_s: float = 1.0


class PiperStack(Teleoperator):
    """The arm's commanded joint positions (the trajectory controller's reference) as actions.

    The gripper's action is its measured opening: its controller publishes no command.

    Args:
        config: The teleoperator's configuration.
    """

    config_class = PiperStackConfig
    name = 'piper_stack'

    def __init__(self, config: PiperStackConfig) -> None:
        super().__init__(config)
        self.config = config
        self._bridge: BridgeClient | None = None

    @property
    def action_features(self) -> dict:
        """Each joint's target position (float), as ``piper_ros`` takes them."""
        return {f'{joint}.pos': float for joint in JOINTS}

    @property
    def feedback_features(self) -> dict:
        """None: the stack's teleoperation takes no feedback from LeRobot."""
        return {}

    @property
    def is_connected(self) -> bool:
        """Whether the bridge answered on connect."""
        return self._bridge is not None

    def connect(self, calibrate: bool = True) -> None:
        """Reach the bridge.

        Raises:
            TimeoutError: If the bridge does not answer.
            RuntimeError: If the bridge has no joint state or image yet.
        """
        bridge = BridgeClient(self.config.endpoint, self.config.timeout_s)
        try:
            bridge.observe()
        except Exception:
            bridge.close()
            raise
        self._bridge = bridge

    @property
    def is_calibrated(self) -> bool:
        """Always: nothing to calibrate."""
        return True

    def calibrate(self) -> None:
        """Nothing to calibrate."""

    def configure(self) -> None:
        """Nothing to configure."""

    def get_action(self) -> dict[str, Any]:
        """Return each joint's commanded position."""
        commanded = self._bridge.observe().commanded
        return dict(zip((f'{joint}.pos' for joint in JOINTS), map(float, commanded), strict=True))

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """Ignore feedback: the stack's teleoperation takes none."""

    def disconnect(self) -> None:
        """Close the connection."""
        if self._bridge is not None:
            self._bridge.close()
            self._bridge = None
