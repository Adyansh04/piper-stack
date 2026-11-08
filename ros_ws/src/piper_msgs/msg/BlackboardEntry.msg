# One entry of a behaviour tree's blackboard.

# The key, prefixed by the subtree it belongs to ("pick_and_place/pick:prompt"; the root's
# entries have no prefix).
string key

# The C++ type as BehaviorTree.CPP names it, e.g. "std::string", "double".
string type

# The value as text; "" when it has none yet, the type in angle brackets when it has no text form.
string value
