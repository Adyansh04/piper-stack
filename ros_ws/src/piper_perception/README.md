# piper_perception

Finds objects by name in the wrist camera's image and proposes grasps for them. Two services:

- `segment_objects` (`piper_msgs/srv/SegmentObjects`): [SAM 3.1](https://github.com/facebookresearch/sam3)
  instance masks for text prompts such as "red cube", run as TensorRT engines from C++.
- `compute_grasps` (`piper_msgs/srv/ComputeGrasps`): parallel-gripper grasps for one object's
  points from [GraspGenX](https://github.com/NVlabs/GraspGenX) (NVIDIA, Apache 2.0), best first.

## Run

In the dev container, with the models under `/models` (`PIPER_MODELS` in `.env`, default
`~/models`):

```bash
ros2 launch piper_perception sam3_segmentation.launch.py
ros2 launch piper_perception grasp_generation.launch.py
```

The first start of the segmentation node builds the TensorRT engines from the ONNX files in
`/models/sam3` (minutes, once per GPU). The grasp server loads GraspGenX's checkpoints from
`/models/graspgenx/{gen,dis}` and runs the model once before it answers (about 20 s).

## Nodes and processes

| Name | What it does | Parameters |
|---|---|---|
| `sam3_segmentation` | Serves `segment_objects`: segments the request's image, or the latest image on `image_topic` when the request has none | `config/sam3_segmentation.yaml` |
| `grasp_generation` | Serves `compute_grasps` by forwarding each request to the GraspGenX server over ZMQ | `config/grasp_generation.yaml` |
| `graspgenx_server.py` | The GraspGenX model behind a ZMQ REP socket, in its own Python 3.12 venv (`/opt/graspgenx-venv`); not a ROS node | `config/graspgenx_server.yaml` |

Worth knowing:

- `min_score` (segmentation): masks scored below it are dropped.
- `endpoint` (server) and `server_endpoint` (node) must be equal; a test checks it. The server has
  no authentication: keep it on localhost or a trusted network.
- The gripper is described to GraspGenX by its finger sweep volume (`extents_*_m`, `offset_*_m`),
  taken from GraspGenX's own `piper_hand` description. Grasps come back as poses of the `tcp`
  link, so `tcp_depth_m` must equal `piper_description`'s `tcp_joint`; a test checks that too.
- The gripper is symmetric: a grasp turned half a turn about its approach axis is the same grasp.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `image_topic` (empty: none) | `sensor_msgs/msg/Image` | in, `segment_objects` |

## Why Python for the grasp server

GraspGenX's point-cloud backbones have no TensorRT or TorchScript export, so the model runs in
PyTorch. It needs Python 3.12 (its torch has no wheels for Lyrical's 3.14), so it runs in its own
venv behind ZMQ, and the C++ `grasp_generation` node puts it on the ROS graph. The wire protocol
(`graspgenx/graspgenx_server.py`) is raw little-endian arrays with a binary header; nothing is
unpickled.

## Tests

```bash
./scripts/manage.sh test
```

`test_sam3_cpu` (tokenizer, tensor types, request checks), `test_graspgenx_client` (the client's
protocol, timeouts and recovery against a fake server) and `test_graspgenx_server` (the server's
request handling with the model stubbed out, and the parameter files' agreement), all without a
GPU. With a GPU and the engines, `check_sam3_engine` runs SAM 3.1 on a test image.

SAM 3.1's model files are under the SAM License (`resources/SAM_LICENSE`); the tokenizer's
vocabulary is OpenAI CLIP's (MIT, `resources/CLIP_LICENSE`).
