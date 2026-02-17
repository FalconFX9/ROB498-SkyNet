# MissionStatus.msg
# Overall mission status

std_msgs/Header header

# Mission state
uint8 STATE_IDLE = 0
uint8 STATE_PREFLIGHT = 1
uint8 STATE_ARMING = 2
uint8 STATE_TAKEOFF = 3
uint8 STATE_ACQUIRING = 4
uint8 STATE_FOLLOWING = 5
uint8 STATE_SWITCHING = 6
uint8 STATE_LANDING = 7
uint8 STATE_EMERGENCY = 8

uint8 state
string state_name

# System status
bool armed
bool offboard_enabled
bool person_detected
bool localization_valid

# Sensor health
bool t265_ok
bool camera_ok

# Current values
float32 altitude
float32 battery_voltage
float32 battery_percent

# Human-readable status
string status_message
