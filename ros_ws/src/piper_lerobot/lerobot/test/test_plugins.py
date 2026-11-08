"""The plugins against a stub of the bridge node that speaks its protocol."""

from __future__ import annotations

import threading

import numpy as np
import pytest
import zmq
from lerobot_robot_piper import PiperRos, PiperRosConfig
from lerobot_teleoperator_piper import PiperStack, PiperStackConfig

MEASURED = np.arange(7, dtype='<f8') / 10
COMMANDED = MEASURED + 0.5
EFFORT = np.array([1.5, -2.0, 3.0, 0.0, 0.25, -0.5, np.nan], dtype='<f8')
HEIGHT, WIDTH = 2, 3
IMAGE = np.arange(HEIGHT * WIDTH * 3, dtype=np.uint8)
# The overhead camera at twice the wrist's size, one colour per 2 x 2 block.
OVERHEAD = np.repeat(np.repeat(IMAGE.reshape(HEIGHT, WIDTH, 3), 2, axis=0), 2, axis=1)


class StubBridge:
    """A reply socket on a free loopback port that answers as the bridge node does."""

    def __init__(self, refuse_observe: bool = False, episode: bytes = b'') -> None:
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        port = self.socket.bind_to_random_port('tcp://127.0.0.1')
        self.endpoint = f'tcp://127.0.0.1:{port}'
        self.refuse_observe = refuse_observe
        self.episode = episode
        self.requests: list[list[bytes]] = []
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.serve)
        self.thread.start()

    def serve(self) -> None:
        while not self.stop.is_set():
            if not self.socket.poll(50):
                continue
            request = self.socket.recv_multipart()
            self.requests.append(request)
            if request[0] == b'observe' and self.refuse_observe:
                self.socket.send_multipart([b'error', b'no camera image yet'])
            elif request[0] == b'observe':
                state = np.concatenate(([12.5], MEASURED, COMMANDED, EFFORT)).astype('<f8')
                frames = [b'ok', state.tobytes(), self.episode]
                for name, image in (
                    (b'wrist', IMAGE.reshape(HEIGHT, WIDTH, 3)),
                    (b'overhead', OVERHEAD),
                ):
                    size = np.array(image.shape[:2], dtype='<u4')
                    frames += [name, size.tobytes(), image.tobytes()]
                self.socket.send_multipart(frames)
            else:
                self.socket.send_multipart([b'ok'])

    def close(self) -> None:
        self.stop.set()
        self.thread.join()
        self.socket.close(linger=0)
        self.context.term()


@pytest.fixture
def bridge():
    stub = StubBridge()
    yield stub
    stub.close()


def robot_for(bridge: StubBridge, passive: bool = False, **config) -> PiperRos:
    config.setdefault('images', f'wrist:{HEIGHT}x{WIDTH}')
    robot = PiperRos(PiperRosConfig(endpoint=bridge.endpoint, passive=passive, **config))
    robot.connect()
    return robot


def test_robot_observes_joints_and_wrist_image(bridge):
    robot = robot_for(bridge)
    observation = robot.get_observation()
    assert set(observation) == set(robot.observation_features)
    assert observation['joint1.pos'] == pytest.approx(0.0)
    assert observation['gripper.pos'] == pytest.approx(0.6)
    assert observation['wrist'].shape == (HEIGHT, WIDTH, 3)
    np.testing.assert_array_equal(observation['wrist'].ravel(), IMAGE)
    robot.disconnect()


def test_robot_sends_actions_in_joint_order_and_releases_on_disconnect(bridge):
    robot = robot_for(bridge)
    keys = list(robot.action_features)
    # Built in reverse: the order on the wire comes from the joints, not the dict.
    action = {key: float(i) for i, key in reversed(list(enumerate(keys)))}
    assert robot.send_action(action) == action
    robot.disconnect()
    acts = [r for r in bridge.requests if r[0] == b'act']
    assert len(acts) == 1
    sent = np.frombuffer(acts[0][1], dtype='<f8')
    np.testing.assert_allclose(sent, np.arange(len(keys)))
    assert bridge.requests[-1] == [b'release']


def test_passive_robot_sends_nothing(bridge):
    robot = robot_for(bridge, passive=True)
    action = dict.fromkeys(robot.action_features, 0.1)
    assert robot.send_action(action) == action
    robot.disconnect()
    assert {r[0] for r in bridge.requests} == {b'observe'}


def test_teleoperator_reads_the_commanded_positions(bridge):
    teleop = PiperStack(PiperStackConfig(endpoint=bridge.endpoint))
    teleop.connect()
    action = teleop.get_action()
    assert list(action) == list(teleop.action_features)
    np.testing.assert_allclose(list(action.values()), COMMANDED)
    teleop.disconnect()


def test_robot_scales_a_camera_down_to_its_configured_size(bridge):
    robot = robot_for(bridge, images=f'wrist:{HEIGHT}x{WIDTH},overhead:{HEIGHT}x{WIDTH}')
    assert robot.cameras == {
        'wrist': 'observation.images.wrist',
        'overhead': 'observation.images.overhead',
    }
    observation = robot.get_observation()
    assert set(observation) == set(robot.observation_features)
    # Each 2 x 2 block of one colour becomes that colour.
    np.testing.assert_array_equal(observation['overhead'], IMAGE.reshape(HEIGHT, WIDTH, 3))
    robot.disconnect()


def test_robot_observes_efforts_a_missing_one_as_zero(bridge):
    robot = robot_for(bridge, effort=True)
    observation = robot.get_observation()
    assert set(observation) == set(robot.observation_features)
    assert observation['joint2.effort'] == pytest.approx(-2.0)
    assert observation['gripper.effort'] == 0.0
    robot.disconnect()


def test_robot_keeps_the_episode_marker():
    stub = StubBridge(episode=b'record:Put the ring on the post')
    try:
        robot = robot_for(stub)
        robot.get_observation()
        assert robot.last_observation.episode == 'record:Put the ring on the post'
        robot.disconnect()
    finally:
        stub.close()


def test_robot_refuses_a_size_the_image_does_not_scale_to(bridge):
    robot = PiperRos(PiperRosConfig(endpoint=bridge.endpoint, images='wrist:480x640'))
    with pytest.raises(ValueError, match='2 x 3'):
        robot.connect()
    assert not robot.is_connected


def test_robot_refuses_a_camera_the_bridge_lacks(bridge):
    robot = PiperRos(PiperRosConfig(endpoint=bridge.endpoint, images='side:2x3'))
    with pytest.raises(ValueError, match='no side camera'):
        robot.connect()


def test_config_refuses_a_malformed_camera_list():
    with pytest.raises(ValueError, match='name:HEIGHTxWIDTH'):
        PiperRos(PiperRosConfig(images='wrist:480'))


def test_robot_reports_the_bridge_refusal():
    stub = StubBridge(refuse_observe=True)
    try:
        robot = PiperRos(PiperRosConfig(endpoint=stub.endpoint))
        with pytest.raises(RuntimeError, match='no camera image yet'):
            robot.connect()
    finally:
        stub.close()


def test_robot_times_out_without_a_bridge():
    robot = PiperRos(PiperRosConfig(endpoint='tcp://127.0.0.1:1', timeout_s=0.2))
    with pytest.raises(TimeoutError):
        robot.connect()
