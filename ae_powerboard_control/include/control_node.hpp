#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include "ros/ros.h"

#include "i2c_driver.h" 
#include "pb6s40a_control.h"

class Control
{
    private:
        //  ******* properties ********
        // ros node
        ros::NodeHandle nh_;
        //i2c
        

        //  ******* methods *******
        void Init();
    
    public:
        // constructor
        Control(const ros::NodeHandle &nh);
};

#endif //CONTROL_NODE_HPP