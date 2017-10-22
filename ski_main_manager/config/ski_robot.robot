[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE
/dev/ttyUSB0| 1000000  | head



[ device info ]
## TYPE    | PORT NAME   | ID   | MODEL         | PROTOCOL | DEV NAME      | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 |  1   | MX-106-2-gear | 2.0      | head          | present_position
dynamixel | /dev/ttyUSB0 | 10   | MX-106-2-gear | 2.0      | waist_roll    | present_position
dynamixel | /dev/ttyUSB0 | 11   | MX-106-2-gear | 2.0      | l_hip_pitch   | present_position
dynamixel | /dev/ttyUSB0 | 12   | MX-106-2-gear | 2.0      | r_hip_pitch   | present_position
dynamixel | /dev/ttyUSB0 | 13   | MX-106-2-gear | 2.0      | l_hip_roll    | present_position
dynamixel | /dev/ttyUSB0 | 14   | MX-106-2-gear | 2.0      | r_hip_roll    | present_position
dynamixel | /dev/ttyUSB0 | 15   | MX-106-2-gear | 2.0      | l_hip_yaw     | present_position
dynamixel | /dev/ttyUSB0 | 16   | MX-106-2-gear | 2.0      | r_hip_yaw     | present_position

dynamixel | /dev/ttyUSB0 | 17   | MX-106-2-gear | 2.0      | l_knee_pitch  | present_position
dynamixel | /dev/ttyUSB0 | 18   | MX-106-2-gear | 2.0      | r_knee_pitch  | present_position
dynamixel | /dev/ttyUSB0 | 19   | MX-106-2-gear | 2.0      | l_ankle_pitch | present_position
dynamixel | /dev/ttyUSB0 | 20   | MX-106-2-gear | 2.0      | r_ankle_pitch | present_position
dynamixel | /dev/ttyUSB0 | 21   | MX-106-2-gear | 2.0      | l_ankle_roll  | present_position
dynamixel | /dev/ttyUSB0 | 22   | MX-106-2-gear | 2.0      | r_ankle_roll  | present_position
