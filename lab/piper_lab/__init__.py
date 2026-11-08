"""Isaac Lab tasks for the Piper, on the simulator's own robot USD and joint drives.

Isaac Lab's training and play scripts register them through
``--external_callback piper_lab.register`` (with ``/piper-stack/lab`` on ``PYTHONPATH``). Python
because Isaac Lab is Python.
"""

from __future__ import annotations

import gymnasium as gym

#: The reach task: the tool to a pose over the table, as the stack's controllers move the arm.
REACH_TASK = 'Piper-Reach'


def register() -> None:
    """Register the Piper's tasks with gymnasium; a second call does nothing."""
    if REACH_TASK in gym.registry:
        return
    gym.register(
        id=REACH_TASK,
        entry_point='isaaclab.envs:ManagerBasedRLEnv',
        disable_env_checker=True,
        kwargs={
            'env_cfg_entry_point': 'piper_lab.reach_env_cfg:PiperReachEnvCfg',
            'rsl_rl_cfg_entry_point': 'piper_lab.agents:PiperReachPPORunnerCfg',
        },
    )
