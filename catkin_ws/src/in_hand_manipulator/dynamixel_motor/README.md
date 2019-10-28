terminal 1. Connecting to Dynamixel bus

	laptop $ roslaunch dynamixel_tutorials controller_manager.launch
-----------------------------------------------------------------------------------------------
terminal 2. Start the controller

	laptop $ roslaunch dynamixel_tutorials start_dual_motor_controller.launch
-----------------------------------------------------------------------------------------------
terminal 2. Control the gripper

	(open)
	laptop $ rostopic pub -1 /dual_motor_controller/command std_msgs/Float64 -- 0.5
	(close)
	laptop $ rostopic pub -1 /dual_motor_controller/command std_msgs/Float64 -- -0.55
