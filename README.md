# ros2-rmdx8
## For developing the ROS2 graph for the RMD x8 in our lab

Only tested on Linux

### setting up the servo

Only run if computer has been powered off

    slcand -c -o -s8 -t hw -S 3000000 /dev/ttyUSB0
    ip link set up slcan0
    
should be able to see slcan0 if you run ifconfig.

Currently ran by (from ~/ros2_ws)

    source install/setup.bash

    ros2 launch rmd_test experiment.launch.py
  
which will get the nodes all set up. Then, to start the desired trajectory 9and hence the experiment)

    ros2 topic pub --once /experiment/go std_msgs/msg/Bool "{data:1}"
  
this is due to the current architecture being the time generator always being zero until the "go" command changes the flag
