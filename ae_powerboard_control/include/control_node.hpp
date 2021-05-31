#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include "ros/ros.h"

#include "i2c_driver.h" 
#include "pb6s40a_control.h"

#define DEVICE_I2C "/dev/i2c-1"
class Control
{
    private:
        //  ******* properties ********
        // ros node
        ros::NodeHandle nh_;
        //i2c
        I2CDriver i2c1_driver_;
        Pb6s40aDroneControl *drone_control_;
        Pb6s40aDroneControl *led_control_;

        //  ******* methods *******
        // init
        void Init();
        void DefaultValues();
        // i2c
        void OpenI2C();
        void CloseI2C();
    
    public:
        // constructor
        Control(const ros::NodeHandle &nh);
};

#endif //CONTROL_NODE_HPP