# timestamp
builtin_interfaces/Time timestamp

# basic sensor info
string frame_id
string sensor_topic

# child_sensor for extrinsic calibration
string child_frame_id
string child_sensor_topic

# basic calibration info
# calibration type: camera_intrinsic, lidar_lidar_extrinsic, etc.
string calibration_type
string command_topic

# state type
uint8 UNKNOWN = 0
uint8 READY = 1
uint8 COLLECTING = 2
uint8 OPTIMIZING = 3
uint8 SUCCESSED = 4
uint8 FAILED = 5

# state
uint8 state 0

# status info
string info
