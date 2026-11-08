# A parallel-gripper grasp.

# Tool centre point pose at the grasp: z along the approach direction, y between the fingers.
geometry_msgs/PoseStamped pose

# Finger opening the grasp needs, in metres: twice the distance from the fingers' centre plane to
# the object point farthest from it.
float32 width_m

# Width of the object between the fingers, where they stop when closing, in metres.
float32 object_width_m

# Grasp quality from the grasp generator's discriminator, 0 to 1.
float32 score
