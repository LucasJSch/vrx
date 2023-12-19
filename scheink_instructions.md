cd ~/Documents/repos/vrx

colcon build --merge-install

. install/setup.bash

ros2 launch vrx_gz competition.launch.py world:=sydney_regatta