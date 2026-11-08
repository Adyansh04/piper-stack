"""RSL-RL PPO settings of the Piper's reach task, after Isaac Lab's deployed UR10e reach task."""

from __future__ import annotations

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


@configclass
class PiperReachPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """PPO for the reach policy: a 256-128-64 MLP with normalised observations."""

    num_steps_per_env = 64
    max_iterations = 1000
    save_interval = 50
    experiment_name = 'piper_reach'
    obs_groups = {'actor': ['policy'], 'critic': ['policy']}
    actor = RslRlMLPModelCfg(
        hidden_dims=[256, 128, 64],
        activation='elu',
        obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[256, 128, 64], activation='elu', obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.0,
        num_learning_epochs=8,
        num_mini_batches=8,
        learning_rate=5.0e-4,
        schedule='adaptive',
        gamma=0.99,
        lam=0.95,
        desired_kl=0.008,
        max_grad_norm=1.0,
    )
