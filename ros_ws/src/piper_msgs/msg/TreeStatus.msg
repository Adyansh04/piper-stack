# The behaviour tree executor's state: the running tree, its nodes' statuses and its blackboard.
# Published on a change (at most every status_period_s of the executor) and once a second.

std_msgs/Header header

uint8 IDLE = 0
# Ticking.
uint8 RUNNING = 1
# Pause: no tick; the skills already running go on to their end, nothing new starts.
uint8 PAUSED = 2
# Hold: no tick and the running skills halted (the arm stops); Resume starts them again.
uint8 HELD = 3
uint8 state

# The tree's ID (the last one while idle), and its instance as XML with each node's _uid and
# _fullpath: what `nodes` refers to. The XML is sent when a run starts and then once in 10 messages.
string tree
string xml

# Every node of the tree; while idle, the last run's as it ended (empty before the first run).
piper_msgs/TreeNodeStatus[] nodes

# The running tree's blackboard; while idle, the last run's.
piper_msgs/BlackboardEntry[] blackboard

# The last run's result: "SUCCESS", "FAILURE", "HALTED" or "" before the first, and its message.
string result
string message
