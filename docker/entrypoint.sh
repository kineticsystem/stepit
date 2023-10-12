#! /bin/bash -e

# Start a new bash session that sources the ROS setup script on start
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

/bin/bash
