#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/RVLuser/ferit_ur5_ws/devel/setup.bash

# explicitly set PYTHONPATH for your pybind11 modules
export PYTHONPATH=$PYTHONPATH:/home/RVLuser/rvl-linux/python/build/lib

# Execute the command passed as arguments
exec "$@"