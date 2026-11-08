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
HEIGHT, WIDTH = 2, 3
IMAGE = np.arange(HEIGHT * WIDTH * 3, dtype=np.uint8)


class StubBridge:
    """A reply socket on a free loopback port that answers as the bridge node does."""

    def __init__(self, refuse_observe: bool = False) -> None:
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        port = self.socket.bind_to_random_port('tcp://127.0.0.1')
        self.endpoint = f'tcp://127.0.0.1:{port}'
        self.refuse_observe = refuse_observe
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
                state = np.concatenate(([12.5], MEASURED, COMMANDED)).astype('<f8')
                size = np.array([HEIGHT, WIDTH], dtype='<u4')
                self.socket.send_multipart(
                    [b'ok', state.tobytes(), size.tobytes(), IMAGE.tobytes()]
                )
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


def robot_for(bridge: StubBridge, passive: bool = False) -> PiperRos:
    robot = PiperRos(
        PiperRosConfig(
            endpoint=bridge.endpoint, passive=passive, image_height=HEIGHT, image_width=WIDTH
        )
    )
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


def test_robot_refuses_another_image_size(bridge):
    robot = PiperRos(PiperRosConfig(endpoint=bridge.endpoint, image_height=480, image_width=640))
    with pytest.raises(ValueError, match='2 x 3'):
        robot.connect()
    assert not robot.is_connected


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
