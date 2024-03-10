# timestamp
builtin_interfaces/Time timestamp

# sensor info
string frame_id
string sensor_topic

# child sensor info for extrinsic calibration
string child_frame_id
string child_sensor_topic

# basic info
# calibration type: camera_intrinsic_calibration, etc.
string calibration_type
string command_topic

# state type
uint8 UNKNOWN = 0
uint8 READY = 1
uint8 COLLECTING = 2
uint8 OPTIMIZING = 3
uint8 DONE = 4

# calibration state
uint8 state 0

# calibration result
bool success false

# calibration info
string info
