# PersonDetection.msg
# Information about a detected person

std_msgs/Header header

# Detection info
bool detected
float32 confidence

# Bounding box in image coordinates (pixels)
int32 bbox_x      # Top-left x
int32 bbox_y      # Top-left y
int32 bbox_width
int32 bbox_height

# Center position relative to image center (normalized -1 to 1)
float32 center_x  # Positive = right
float32 center_y  # Positive = down

# Estimated position in camera frame (meters)
geometry_msgs/Point position

# Tracking ID (for multi-person tracking)
int32 tracking_id
