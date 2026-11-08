# An object found on the table by an open-vocabulary prompt.

# The prompt's words joined by underscores and a counter within one segmentation: "red_cube_0".
string id

# The prompt that found it, e.g. "red cube".
string label

# Segmentation score, 0 to 1.
float32 score

# Centroid of the object's own points, axes along their principal directions on the floor (z up), in
# the robot's base frame; the stamp is the image's.
geometry_msgs/PoseStamped pose

# Extent of the points along the pose axes, in metres.
geometry_msgs/Vector3 size

# Number of the object's own points (those of its mask, without the table or a neighbour).
uint32 point_count
