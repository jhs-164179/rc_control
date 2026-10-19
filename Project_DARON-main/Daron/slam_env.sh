source ~/venv/cuvslam/bin/activate
source /opt/ros/humble/setup.bash
source ~/Documents/cuVSLAM/build/cuvslam_vars.sh
export PYTHONPATH=$PYTHONPATH:$HOME/Documents/cuVSLAM/python/build/cp310-cp310-linux_x86_64
source ~/cuvslam_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "[SLAM 환경] python3=$(which python3) / DOMAIN=$ROS_DOMAIN_ID"
