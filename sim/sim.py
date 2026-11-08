"""Simulates the Piper in Isaac Sim: the arm and its gripper, driven by the stack over ROS 2.

Run inside the sim image (``scripts/run/sim.sh`` does):

    /isaac-sim/python.sh /piper-stack/sim/sim.py --config /piper-stack/sim/config/sim.yaml

HEADLESS (true or false), PHYSICS_ENGINE (physx or newton) and SIM_MODE (control or shadow)
override app.headless, app.engine and app.mode. Python because Isaac Sim's scripting API is Python.
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import threading

import yaml

# Enabled before the simulator is imported: rclpy comes with the ROS 2 bridge.
EXTENSIONS = ('isaacsim.ros2.core', 'isaacsim.ros2.bridge')
ENGINES = ('physx', 'newton')
MODES = ('control', 'shadow')


def load_config(path: str) -> dict:
    """Return sim.yaml with the HEADLESS, PHYSICS_ENGINE and SIM_MODE overrides applied.

    Raises:
        ValueError: If the engine is neither physx nor newton, or the mode neither control nor
            shadow.
    """
    with open(path) as f:
        cfg = yaml.safe_load(f)
    if os.getenv('HEADLESS'):
        cfg['app']['headless'] = os.getenv('HEADLESS') == 'true'
    if os.getenv('PHYSICS_ENGINE'):
        cfg['app']['engine'] = os.getenv('PHYSICS_ENGINE')
    if os.getenv('SIM_MODE'):
        cfg['app']['mode'] = os.getenv('SIM_MODE')
    if cfg['app']['engine'] not in ENGINES:
        raise ValueError(f'app.engine {cfg["app"]["engine"]!r} is not one of {ENGINES}')
    if cfg['app']['mode'] not in MODES:
        raise ValueError(f'app.mode {cfg["app"]["mode"]!r} is not one of {MODES}')
    return cfg


def main() -> None:
    """Start Isaac Sim and run the simulator until its window closes or the process is stopped."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default='/piper-stack/sim/config/sim.yaml')
    args, _ = parser.parse_known_args()
    cfg = load_config(args.config)
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter('[%(name)s] %(levelname)s: %(message)s'))
    logger = logging.getLogger('sim')
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    logger.propagate = False
    # A container stop's SIGTERM and Ctrl-C end the run loop, so the simulator shuts down in order.
    stop = threading.Event()
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: stop.set())

    from isaacsim import SimulationApp

    engine = cfg['app']['engine']
    app = SimulationApp(
        {
            'headless': cfg['app']['headless'],
            'renderer': 'RayTracedLighting',
            # The engine is chosen at start, and Newton's extension must not switch to itself.
            'extra_args': [
                f'--/exts/isaacsim.core.simulation_manager/default_engine={engine}',
                '--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
            ],
        }
    )
    exit_code = 0
    try:
        import isaacsim.core.experimental.utils.app as app_utils

        for ext in EXTENSIONS:
            app_utils.enable_extension(ext)
        if engine == 'newton':
            app_utils.enable_extension('isaacsim.physics.newton')
        app.update()
        import simulator

        simulator.run(app, cfg, stop)
    except Exception:
        # Kit's fast shutdown ends the process in close(): the traceback must be logged first.
        logger.exception('The simulator failed')
        exit_code = 1
    finally:
        app.close(exit_code=exit_code)


if __name__ == '__main__':
    main()
