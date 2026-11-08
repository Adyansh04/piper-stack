"""Record LeRobot episodes of what the stack does, cut by the stack's episode marker.

Runs in LeRobot's environment beside piper_lerobot's bridge, from this directory:

    HF_HUB_OFFLINE=1 PYTHONPATH=$PWD /opt/lerobot-venv/bin/python record_episodes.py \\
        --root /piper-stack/data/lerobot/assembly_ring --repo-id local/assembly_ring \\
        --images wrist:480x640,overhead:360x640 --effort --episodes 50

The marker (the bridge's ``episode_topic``, std_msgs/String) is ``record:<n>:<task>`` while
episode n runs, then ``keep:<n>`` or ``discard:<n>``; ``end`` stops the recorder. A frame is the
``piper_ros`` robot's observation and the arm's commanded positions as the action, as
lerobot-record takes them with a passive robot and the ``piper_stack`` teleoperator.

Python because LeRobot is Python.
"""

from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features
from lerobot_robot_piper import PiperRos, PiperRosConfig
from lerobot_robot_piper.piper_ros import JOINTS

log = logging.getLogger('record_episodes')


def record(args: argparse.Namespace) -> int:
    """Record until args.episodes are kept or the marker says end; return the episodes kept."""
    robot = PiperRos(
        PiperRosConfig(endpoint=args.endpoint, passive=True, images=args.images, effort=args.effort)
    )
    robot.connect()
    features = {
        **hw_to_dataset_features(robot.action_features, ACTION),
        **hw_to_dataset_features(robot.observation_features, OBS_STR),
    }
    dataset = LeRobotDataset.create(
        args.repo_id,
        args.fps,
        features,
        root=args.root,
        robot_type=robot.name,
        use_videos=True,
        image_writer_threads=4,
    )
    kept = 0
    episode: tuple[str, str] | None = None
    period_s = 1.0 / args.fps
    next_s = time.perf_counter()
    try:
        while kept < args.episodes:
            observation = robot.get_observation()
            state = robot.last_observation
            kind, _, rest = state.episode.partition(':')
            if kind == 'record':
                number, _, task = rest.partition(':')
                if episode is not None and episode[0] != number:
                    # The last episode's verdict never came: what it recorded is dropped.
                    log.warning('Episode %s ended without keep or discard: dropped', episode[0])
                    dataset.clear_episode_buffer()
                episode = (number, task)
                action = dict(
                    zip(
                        (f'{joint}.pos' for joint in JOINTS),
                        map(float, state.commanded),
                        strict=True,
                    )
                )
                frame = {
                    **build_dataset_frame(dataset.features, observation, OBS_STR),
                    **build_dataset_frame(dataset.features, action, ACTION),
                    'task': task,
                }
                dataset.add_frame(frame)
            elif kind in ('keep', 'discard') and episode is not None and rest == episode[0]:
                if kind == 'keep':
                    dataset.save_episode()
                    kept += 1
                    log.info('Kept episode %s (%d of %d)', episode[0], kept, args.episodes)
                else:
                    dataset.clear_episode_buffer()
                    log.info('Discarded episode %s', episode[0])
                episode = None
            elif kind == 'end':
                break
            next_s += period_s
            delay_s = next_s - time.perf_counter()
            if delay_s > 0:
                time.sleep(delay_s)
            else:
                # Behind (a slow reply): pace from now rather than rush to catch up.
                next_s = time.perf_counter()
    finally:
        if episode is not None:
            dataset.clear_episode_buffer()
        dataset.finalize()
        robot.disconnect()
    return kept


def main() -> None:
    """Parse the arguments and record."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--root', type=Path, required=True, help="The dataset's directory (new)")
    parser.add_argument('--repo-id', default='local/episodes', help="The dataset's name")
    parser.add_argument('--fps', type=int, default=15, help='Frames recorded a second')
    parser.add_argument('--episodes', type=int, default=50, help='Episodes to keep')
    parser.add_argument('--endpoint', default='tcp://127.0.0.1:5590', help="The bridge's endpoint")
    parser.add_argument(
        '--images', default='wrist:480x640', help='Cameras and their sizes, name:HEIGHTxWIDTH,...'
    )
    parser.add_argument('--effort', action='store_true', help="Record the joints' efforts too")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='[record_episodes] %(message)s')
    kept = record(args)
    log.info('Recorded %d episodes to %s', kept, args.root)


if __name__ == '__main__':
    main()
