# Message for defining the state of the car
std_msgs/Header header           # header for when the message is published
bool estop_on           # whether the estop is turned on. When the estop is on (true) the robot is disabled
bool mux_autonomous     # when mux_automous is set to true, the robot is being controlled autonomously
float32 speed_mps       # speed of the car in m/s
float32 steer_rad       # steering angle of the car in radians
string state            # FSM state from the Arduino
bool record_bag		    # reading from remote control. When true, data is recorded to bag file.
