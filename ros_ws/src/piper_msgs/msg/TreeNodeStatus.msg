# One node of the running behaviour tree and its status.

uint8 IDLE = 0
uint8 RUNNING = 1
uint8 SUCCESS = 2
uint8 FAILURE = 3
uint8 SKIPPED = 4

# The node's UID in TreeStatus.xml (its _uid attribute).
uint16 uid
uint8 status
# Its last status other than IDLE in this run: how it ended, after a finished parent reset it to
# IDLE (IDLE: it has not run yet).
uint8 last
