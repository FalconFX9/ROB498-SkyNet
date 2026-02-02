# TrackingStatus.msg
# Current person tracking status

std_msgs/Header header

# Tracking state
uint8 STATE_LOST = 0
uint8 STATE_ACQUIRING = 1
uint8 STATE_TRACKING = 2
uint8 STATE_SWITCHING = 3

uint8 state

# Current target
int32 target_id
float32 track_confidence
float32 time_since_detection  # seconds

# Target position and velocity
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
