# "Target center depth info" published by the camera detector node
# Constraint: center_depth / position_* must come from the centroid of the
# target segmentation mask, NOT the bounding box center

std_msgs/Header header

# ---- Detection result ----
string  class_name        # target class name (YOLOv8 class)
int32   class_id          # target class id
float32 confidence        # confidence [0,1]

# ---- Target center ----
uint16  center_u          # center pixel column coordinate u
uint16  center_v          # center pixel row coordinate v

# ---- Target center depth ----
float32 center_depth      # target center depth (unit: m); must be the target center, not box average

# ---- 3D coordinates ----
geometry_msgs/Point position_camera    # center point in camera frame (m)
geometry_msgs/Point position_gripper   # center point transformed to gripper frame via hand-eye calibration (m)

# ---- Status flags ----
bool    valid             # whether depth is valid for this frame (depth valid & within range)
bool    in_range          # whether target is within acceptable grasping range
