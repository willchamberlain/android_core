
export ROS_CONFIG_USING_VOS_AA1="true"

if [[ "$ROS_CONFIG_USING_VOS_AA1" = "true" || -e ~/Desktop/ROS_CONFIG_USING_VOS_AA1 ]];
then
  echo "----------------------------------------------------------------"
  export ROS_CONFIG_VOS_AA1_catkin_ws_dir=/mnt/nixbig/build_workspaces/aa1_vos_android_catkin_ws
  export ROS_CONFIG_VOS_AA1_setup_bash=${ROS_CONFIG_VOS_AA1_catkin_ws_dir}/devel/setup.bash
  source ${ROS_CONFIG_VOS_AA1_setup_bash}
  echo "ROS_CONFIG_USING_VOS_AA1 = true OR file ~/Desktop/ROS_CONFIG_USING_VOS_AA1 exists: "
  echo " --> exporting and sourcing ROS_CONFIG_VOS_AA1_setup_bash as ${ROS_CONFIG_VOS_AA1_setup_bash}"
  echo "----------------------------------------------------------------"
else
  echo "----------------------------------------------------------------"
  echo "ROS_CONFIG_USING_VOS_AA1 != true AND Desktop/ROS_CONFIG_USING_VOS_AA1 does not exist: not exporting and sourcing ROS_CONFIG_VOS_AA1_setup_bash"
  echo "----------------------------------------------------------------"
fi