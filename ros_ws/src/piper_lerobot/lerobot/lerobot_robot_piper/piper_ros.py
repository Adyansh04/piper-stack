"""LeRobot's ``piper_ros`` robot: the Piper through the stack's bridge node."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np
from lerobot.robots import Robot

from .bridge import BridgeClient, Observation
from .config_piper_ros import PiperRosConfig

#: The arm's joints and the gripper, in the bridge's order; positions in rad, the gripper's in m.
JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper')


class PiperRos(Robot):
    """The Piper through the stack: joint positions, efforts and camera images in, targets out.

    Actions go to the bridge, which turns them into MoveIt Servo joint jogs (collision and limit
    checked) and gripper goals; a passive robot sends none. A camera's image is
    ``observation.images.<name>`` in a dataset, at its configured size.

    Args:
        config: The robot's configuration.
    """

    config_class = PiperRosConfig
    name = 'piper_ros'

    def __init__(self, config: PiperRosConfig) -> None:
        super().__init__(config)
        self.config = config
        self._sizes = config.camera_sizes()
        self._bridge: BridgeClient | None = None
        #: The bridge's last answer to an observation (its episode marker for a recorder).
        self.last_observation: Observation | None = None

    @property
    def observation_features(self) -> dict:
        """Each joint's position (and effort, if observed) as floats, each camera (h, w, 3)."""
        features = {f'{joint}.pos': float for joint in JOINTS}
        if self.config.effort:
            features |= {f'{joint}.effort': float for joint in JOINTS}
        return features | {name: (*size, 3) for name, size in self._sizes.items()}

    @property
    def cameras(self) -> dict[str, str]:
        """The cameras by name, with their dataset keys (LeRobot's scripts count them)."""
        return {name: f'observation.images.{name}' for name in self._sizes}

    @property
    def action_features(self) -> dict:
        """Each joint's target position (float)."""
        return {f'{joint}.pos': float for joint in JOINTS}

    @property
    def is_connected(self) -> bool:
        """Whether the bridge answered on connect."""
        return self._bridge is not None

    def connect(self, calibrate: bool = True) -> None:
        """Reach the bridge and check that it serves every configured camera at a fitting size.

        Raises:
            TimeoutError: If the bridge does not answer.
            RuntimeError: If the bridge has no joint state or image yet.
            ValueError: If a camera is missing, or its image is smaller than configured or of
                another aspect.
        """
        bridge = BridgeClient(self.config.endpoint, self.config.timeout_s)
        try:
            images = bridge.observe().images
            for name, (height, width) in self._sizes.items():
                if name not in images:
                    raise ValueError(f'The bridge serves no {name} camera (it has {list(images)})')
                h, w = images[name].shape[:2]
                if h < height or w < width or h * width != w * height:
                    raise ValueError(
                        f'The {name} image is {h} x {w}, which does not scale down to the '
                        f'configured {height} x {width}'
                    )
        except Exception:
            bridge.close()
            raise
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
        """Return each joint's measured position (and effort) and every camera's image."""
        observation = self._bridge.observe()
        self.last_observation = observation
        out: dict[str, Any] = dict(
            zip((f'{joint}.pos' for joint in JOINTS), map(float, observation.measured), strict=True)
        )
        if self.config.effort:
            # A joint that reports no effort reads 0, so a dataset never holds NaN.
            efforts = np.nan_to_num(observation.effort, nan=0.0)
            out |= dict(
                zip((f'{joint}.effort' for joint in JOINTS), map(float, efforts), strict=True)
            )
        for name, (height, width) in self._sizes.items():
            image = observation.images[name]
            if image.shape[:2] != (height, width):
                image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
            out[name] = image
        return out

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
