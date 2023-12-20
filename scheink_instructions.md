cd ~/Documents/repos/vrx

colcon build --merge-install

. install/setup.bash

ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

gz topic -t "/X3/gazebo/command/twist" -m gz.msgs.Twist -p "linear: {x:0 y: 5 z: 1} angular {z: 0}"
