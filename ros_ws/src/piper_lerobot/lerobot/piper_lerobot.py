"""LeRobot's command line with the Piper's plugins (``piper_ros``, ``piper_stack``) registered.

LeRobot finds plugins only as installed packages; these run from the source instead, so this
imports them before handing over to LeRobot's own script. Run with LeRobot's environment and this
directory on the path (``PYTHONPATH=<this directory>``):

  /opt/lerobot-venv/bin/python piper_lerobot.py record --robot.type=piper_ros ...

The first argument picks the script (record, rollout, teleoperate, replay, train); the others go
to it as they are.
"""

from __future__ import annotations

import importlib
import sys

import lerobot_robot_piper  # noqa: F401  (registers piper_ros)
import lerobot_teleoperator_piper  # noqa: F401  (registers piper_stack)

SCRIPTS = {
    name: f'lerobot.scripts.lerobot_{name}'
    for name in ('record', 'rollout', 'teleoperate', 'replay', 'train')
}


def main() -> int:
    """Run the LeRobot script the first argument names; return the exit code."""
    if len(sys.argv) < 2 or sys.argv[1] not in SCRIPTS:
        print(f'usage: piper_lerobot.py {{{",".join(SCRIPTS)}}} [lerobot arguments...]')
        return 2
    script = importlib.import_module(SCRIPTS[sys.argv[1]])
    sys.argv = [f'lerobot-{sys.argv[1]}', *sys.argv[2:]]
    script.main()
    return 0


if __name__ == '__main__':
    sys.exit(main())
