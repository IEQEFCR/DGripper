CURRENT_DIR=$(cd $(dirname $0); pwd)
source $CURRENT_DIR/devel/setup.bash
rosrun moveit_arm teleop.py
