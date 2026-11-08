"""The wrist camera: an RTX camera on the mount that publishes as Percipio's driver does.

Colour, depth aligned to colour and their camera info go out under the driver's names
(``<namespace>/color/image_raw``, ``color/camera_info``, ``depth/image_raw``, ``depth/camera_info``)
from Isaac Sim's OmniGraph camera helpers, which publish from C++.
"""

from __future__ import annotations

import json

import numpy as np
import omni.graph.core as og
import usdrt.Sdf
from pxr import Gf, Sdf, Usd, UsdGeom

# Every camera publisher's QoS: reliable serves the stack's best-effort sensor subscribers and
# RViz's reliable default alike; only the latest frames matter.
SENSOR_QOS = json.dumps(
    {
        'history': 'keepLast',
        'depth': 10,
        'reliability': 'reliable',
        'durability': 'volatile',
        'deadline': 0.0,
        'lifespan': 0.0,
        'liveliness': 'systemDefault',
        'leaseDuration': 0.0,
    }
)
HELPER_TYPES = {
    'Rgb': 'isaacsim.ros2.bridge.ROS2CameraHelper',
    'RgbInfo': 'isaacsim.ros2.bridge.ROS2CameraInfoHelper',
    'Depth': 'isaacsim.ros2.bridge.ROS2CameraHelper',
    'DepthInfo': 'isaacsim.ros2.bridge.ROS2CameraInfoHelper',
}


def frame_skip(render_hz: float, rate_hz: float) -> int:
    """Return the rendered frames to skip between published ones for the rate nearest rate_hz."""
    return min(range(int(render_hz)), key=lambda k: abs(render_hz / (k + 1) - rate_hz))


def place(stage: Usd.Stage, path: str, T: np.ndarray, type_name: str = 'Xform') -> Usd.Prim:
    """Define a prim at a path with a 4 x 4 transform in its parent."""
    prim = stage.DefinePrim(Sdf.Path(path), type_name)
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    # USD's matrices act on row vectors.
    xformable.AddTransformOp().Set(Gf.Matrix4d(*np.asarray(T, dtype=float).T.flatten()))
    return prim


def add_camera(
    stage: Usd.Stage, path: str, T_optical: np.ndarray, cfg: dict, render_hz: float
) -> None:
    """Add the wrist camera and its publishers.

    The image has the real colour camera's focal length with the principal point at its centre
    and no distortion; the camera info describes exactly that image.

    Args:
        stage: The stage.
        path: The camera's prim path, under the body that carries it.
        T_optical: The colour optical frame in that body (z forward, x right, y down).
        cfg: sim.yaml's ``camera`` section.
        render_hz: Frames rendered a second.
    """
    width_px, height_px = int(cfg['width_px']), int(cfg['height_px'])
    # USD cameras look along -z with y up: half a turn about x from the optical frame.
    flip = np.diag([1.0, -1.0, -1.0, 1.0])
    camera = UsdGeom.Camera(place(stage, path, T_optical @ flip, 'Camera'))
    aperture_mm = 20.955
    camera.CreateHorizontalApertureAttr(aperture_mm)
    camera.CreateVerticalApertureAttr(aperture_mm * height_px / width_px)
    camera.CreateFocalLengthAttr(aperture_mm * float(cfg['fx_px']) / width_px)
    camera.CreateClippingRangeAttr(Gf.Vec2f(float(cfg['near_m']), float(cfg['far_m'])))
    camera.CreateProjectionAttr('perspective')
    skip = frame_skip(render_hz, float(cfg['rate_hz']))
    helpers = {
        'Rgb': ('color/image_raw', 'rgb', cfg['color_frame_id']),
        'RgbInfo': ('color/camera_info', None, cfg['color_frame_id']),
        'Depth': ('depth/image_raw', 'depth', cfg['depth_frame_id']),
        'DepthInfo': ('depth/camera_info', None, cfg['depth_frame_id']),
    }
    values = [
        ('RenderProduct.inputs:cameraPrim', [usdrt.Sdf.Path(path)]),
        ('RenderProduct.inputs:width', width_px),
        ('RenderProduct.inputs:height', height_px),
    ]
    for node, (topic, kind, frame_id) in helpers.items():
        values += [
            (f'{node}.inputs:nodeNamespace', cfg['namespace']),
            (f'{node}.inputs:frameId', frame_id),
            (f'{node}.inputs:topicName', topic),
            (f'{node}.inputs:frameSkipCount', skip),
            (f'{node}.inputs:qosProfile', SENSOR_QOS),
        ]
        if kind is not None:
            values.append((f'{node}.inputs:type', kind))
    graph = f'/Graphs/{Sdf.Path(path).name}'
    keys = og.Controller.Keys
    og.Controller.edit(
        {
            'graph_path': graph,
            'evaluator_name': 'push',
            'pipeline_stage': og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ('OnTick', 'omni.graph.action.OnTick'),
                ('RenderProduct', 'isaacsim.core.nodes.IsaacCreateRenderProduct'),
            ]
            + [(node, HELPER_TYPES[node]) for node in helpers],
            keys.CONNECT: [('OnTick.outputs:tick', 'RenderProduct.inputs:execIn')]
            + [
                (f'RenderProduct.outputs:{out}', f'{node}.inputs:{inp}')
                for node in helpers
                for out, inp in (('execOut', 'execIn'), ('renderProductPath', 'renderProductPath'))
            ],
            keys.SET_VALUES: values,
        },
    )
    og.Controller.evaluate_sync(graph)
