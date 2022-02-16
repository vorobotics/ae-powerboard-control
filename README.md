# AE-POWERBOARD-CONTROL PACKAGE
This repository contains ROS package needed for control DroneCore.Power board.


### Cloning and using this repository
There are some extra steps needed when cloning repository with submodules. 
Clone, like you would do it with any other repository: 

*Tip: clones this in you catkin_ws/src directly, so you can build it right away* 

    git clone https://github.com/airvolute/ae-powerboard-control.git
Then you need to init submodules

    git submodule init
Finally, update submodules (clone actual submodules):

    git submodule update




## Building the code on DroneCore.Suite
To build the code execute: 

    source catkin_ws/devel/setup.bash

    catkin clean ae_powerboard_control

    catkin build ae_powerboard_control


## Running the code on DroneCore.Suite

Run roscore in separate terminal window:

    roscore

Launch control.launch in separate terminal window:

    roslaunch ae_powerboard_control control.launch

Run one of the following examples:

    rosrun ae_powerboard_control example_led_custom_color
    rosrun ae_powerboard_control example_led_one_color
    rosrun ae_powerboard_control example_set_predefined_effect
    rosrun ae_powerboard_control example_set_custom_effect	


