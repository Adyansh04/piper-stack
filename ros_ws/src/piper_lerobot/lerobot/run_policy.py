"""Run a trained LeRobot policy on the Piper through piper_lerobot's bridge, for a set time.

lerobot-rollout keeps only a real robot's ``.pos`` and ``.vel`` state, so a policy trained on the
joints' efforts too gets a shorter state than it was trained on; this runner gives the policy the
robot's whole observation, as the recorder stored it. Runs in LeRobot's environment, from here:

    HF_HUB_OFFLINE=1 PYTHONPATH=$PWD /opt/lerobot-venv/bin/python run_policy.py \\
        --policy <checkpoint>/pretrained_model --images wrist:480x640,overhead:360x640 --effort \\
        --task "Put the ring on the post" --duration-s 60

Python because LeRobot is Python.
"""

from __future__ import annotations

import argparse
import logging
import time

import numpy as np
import torch
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies import prepare_observation_for_inference
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.processor import policy_output_to_transition, transition_to_policy_action
from lerobot.utils.constants import OBS_STATE, OBS_STR
from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features
from lerobot_robot_piper import PiperRos, PiperRosConfig
from lerobot_robot_piper.piper_ros import JOINTS

log = logging.getLogger('run_policy')


def main() -> None:
    """Load the policy, connect the robot and run the policy at the rate for the duration."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--policy', required=True, help="The checkpoint's pretrained_model")
    parser.add_argument('--task', default='', help="The episodes' task text")
    parser.add_argument('--duration-s', type=float, default=60.0, help='How long it runs, s')
    parser.add_argument('--fps', type=int, default=15, help='Actions a second (the data rate)')
    parser.add_argument('--endpoint', default='tcp://127.0.0.1:5590', help="The bridge's endpoint")
    parser.add_argument('--images', default='wrist:480x640', help='name:HEIGHTxWIDTH,...')
    parser.add_argument('--effort', action='store_true', help='The policy observes efforts')
    args = parser.parse_args()
    # LeRobot sets up the root logger on import.
    logging.basicConfig(level=logging.INFO, format='[run_policy] %(message)s', force=True)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    config = PreTrainedConfig.from_pretrained(args.policy)
    config.device = device
    policy = get_policy_class(config.type).from_pretrained(args.policy, config=config)
    policy.to(device).eval()
    preprocessor, postprocessor = make_pre_post_processors(config, pretrained_path=args.policy)
    robot = PiperRos(PiperRosConfig(endpoint=args.endpoint, images=args.images, effort=args.effort))
    robot.connect()
    features = hw_to_dataset_features(robot.observation_features, OBS_STR)
    policy.reset()
    period_s = 1.0 / args.fps
    started = time.perf_counter()
    next_s = started
    steps = 0
    try:
        while time.perf_counter() - started < args.duration_s:
            frame = build_dataset_frame(features, robot.get_observation(), OBS_STR)
            # Read before inference, which turns the frame's arrays into tensors in place.
            state = np.array(frame[OBS_STATE][: len(JOINTS)])
            with torch.inference_mode():
                observation = prepare_observation_for_inference(
                    frame, torch.device(device), args.task, robot.name
                )
                transition = policy_output_to_transition(
                    policy.select_action(preprocessor(observation))
                )
                action = postprocessor(transition_to_policy_action(transition))
            values = action.squeeze(0).cpu().tolist()
            if steps % args.fps == 0:
                lead = np.asarray(values) - state
                log.info('%3d s: action - state %s', steps // args.fps, np.round(lead, 3).tolist())
            robot.send_action({f'{joint}.pos': v for joint, v in zip(JOINTS, values, strict=True)})
            steps += 1
            next_s += period_s
            delay_s = next_s - time.perf_counter()
            if delay_s > 0:
                time.sleep(delay_s)
            else:
                next_s = time.perf_counter()
    finally:
        robot.disconnect()
    log.info('Ran %d steps in %.0f s', steps, time.perf_counter() - started)


if __name__ == '__main__':
    main()
