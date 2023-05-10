# ros2-rmdx8
for developing the ROS2 graph for the RMD x8 in our lab

Currently ran by (from ~/ros2_ws)

  - ros2 launch rmd_test experiment.launch.py
  
which will get the nodes all set up. Then, to start the desired trajectory 9and hence the experiment)

  - ros2 topic pub --once /experiment/go std_msgs/msg/Bool "{data:1}"
  
this is due to the current architecture being the time generator always being zero until the "go" command changes the flag
