# ur_control
control UR robots program

## Get Start
### Use sim robot
ros2 run ur_client_library start_ursim.sh -m ur3

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.56.101 launch_rviz:=true

cd to my project's workspace
source it

ros2 run ur_control ur_control ${path to your trajectory file}
