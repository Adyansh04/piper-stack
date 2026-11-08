"""The reach policy's action: joint position targets, approached under the arm's caps.

The law of the stack's policy controller (``piper_controllers``' JointRamp), so a policy meets the
same motion in training and on the arm: setpoints move gradually from the measured state.
"""

from __future__ import annotations

from dataclasses import MISSING

import torch
from isaaclab.envs.mdp.actions.actions_cfg import JointActionCfg
from isaaclab.envs.mdp.actions.joint_actions import JointAction
from isaaclab.utils import configclass


class RampedJointPositionAction(JointAction):
    """Wanted joint positions, approached under velocity and acceleration caps each physics step.

    The action, times ``scale`` plus each joint's default position and inside the soft joint
    limits, is the joint's wanted position in rad. Every physics step the wanted velocity is the
    fastest from which the joint still stops at that position (at most ``max_velocity``), the
    commanded velocity moves towards it by at most ``max_acceleration`` times the step, and the
    position target moves by the commanded velocity. A reset holds the joints where they are.

    Args:
        cfg: The term's configuration.
        env: The environment the term acts in.
    """

    cfg: RampedJointPositionActionCfg

    def __init__(self, cfg: RampedJointPositionActionCfg, env) -> None:
        super().__init__(cfg, env)
        self._offset = self._asset.data.default_joint_pos.torch[:, self._joint_ids].clone()
        self._dt = env.physics_dt
        self._velocity = torch.zeros_like(self._raw_actions)
        self._target = torch.zeros_like(self._raw_actions)
        limits = self._asset.data.soft_joint_pos_limits.torch[:, self._joint_ids]
        self._lower = limits[..., 0].clone()
        self._upper = limits[..., 1].clone()

    def process_actions(self, actions: torch.Tensor) -> None:
        """Turn the policy's actions into wanted positions inside the joint limits."""
        super().process_actions(actions)
        torch.clamp(self._processed_actions, self._lower, self._upper, out=self._processed_actions)

    def apply_actions(self) -> None:
        """Move the position targets one physics step towards the wanted positions."""
        cfg = self.cfg
        error = self.processed_actions - self._target
        distance = error.abs()
        speed = torch.minimum(
            torch.sqrt(2.0 * cfg.max_acceleration * distance), distance / self._dt
        ).clamp(max=cfg.max_velocity)
        change = cfg.max_acceleration * self._dt
        self._velocity += (torch.copysign(speed, error) - self._velocity).clamp(-change, change)
        self._target += self._velocity * self._dt
        torch.clamp(self._target, self._lower, self._upper, out=self._target)
        self._asset.set_joint_position_target_index(target=self._target, joint_ids=self._joint_ids)

    def reset(self, env_ids=None) -> None:
        """Hold the reset environments' joints at their measured positions."""
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        self._velocity[ids] = 0.0
        self._target[ids] = self._asset.data.joint_pos.torch[ids][:, self._joint_ids]
        self._processed_actions[ids] = self._target[ids]


@configclass
class RampedJointPositionActionCfg(JointActionCfg):
    """Configuration of :class:`RampedJointPositionAction`; ``scale`` is in rad."""

    class_type: type = RampedJointPositionAction

    max_velocity: float = MISSING
    """Fastest commanded joint velocity, in rad/s."""

    max_acceleration: float = MISSING
    """Largest change of a joint's commanded velocity, in rad/s^2."""
