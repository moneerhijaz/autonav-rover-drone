Important:

How to start up:

ros2 run micro_ros_agent micro_rog_agent serial --dev /dev/ttyACM0 -v6

How to control via keyboard

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_val=/cmd_vel -p speed:=0.25 -p turn:=.50

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6  ros2 run teleop_twist_keyboard teleop_twist_keyboard   --ros-args   -r cmd_vel:=/cmd_vel   -p speed:=0.25   -p turn:=0.50
