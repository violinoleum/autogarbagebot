Source ROS2:
source /opt/ros/jazzy/setup.bash

New terminal:
Start gazebo to ros2 bridge:
ros2 launch ros_gz_bridge ros_gz_bridge.launch.py bridge_name:=ros_gz_bridge config_file:=bridge.yaml

New terminal:
Launch gazebo world from ros2:
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=gazebo_world.sdf

New terminal
sudo apt install python3.12-venv
python3 -m venv ros2_env
Launch control.py: (Need to enter ros2_env environment first)
source ros2_env/bin/activate
python3 controls.py

Any other installations done:

In python env:
pip install pyproj
pip install simple-pid
pip install pyyaml
pip install numpy
pip install opencv-python
pip install ultralytics
pip install numpy==1.26.4 // cv_bridge only works with numpy 1.x