# Message for defining the state of the car
Header header           # header for when the message is published
bool estop_on           # whether the estop is turned on. When the estop is on (true) the robot is disabled
bool mux_automatic      # when mux_automatic is set to true, the robot is being controlled autonomously
float32 speed_mps       # speed of the car in m/s

#axes rotation in degree
float32 axes_roll		
float32 axes_pitch
float32 axes_yaw		