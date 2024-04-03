CURRENT_DIR=$(cd $(dirname $0); pwd)
source $CURRENT_DIR/DGripper_ws/devel/setup.bash
roslaunch moveit_arm demo.launch