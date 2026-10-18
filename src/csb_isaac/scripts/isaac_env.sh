# Isaac Sim 6.1 environment (roadmap 036): the pip install's venv and Isaac's bundled ROS 2
# Humble libraries (built for its Python 3.12; do not source the system Humble into it).
# RMW_IMPLEMENTATION and CYCLONEDDS_URI pass through, so the stack's DDS settings apply.
I=${ISAAC_ROOT:-/mnt/disk1/isaac-sim}
source $I/venv/bin/activate
export OMNI_KIT_ACCEPT_EULA=YES
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export LD_LIBRARY_PATH=$I/venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
# Isaac's rclpy, and Autoware's messages rebuilt for it (`build_msgs.sh`, roadmap 038)
H=$I/venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/humble
export PYTHONPATH=$I/msgs:$H/rclpy${PYTHONPATH:+:$PYTHONPATH}
export LD_LIBRARY_PATH=$I/msgs/lib:$LD_LIBRARY_PATH
