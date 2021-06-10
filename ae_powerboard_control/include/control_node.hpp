#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include "ros/ros.h"

#include "utils.hpp"
#include "i2c_driver.h" 
#include "pb6s40a_control.h"

#include "ae_powerboard_control/GetDeviceInfo.h"

#define DEVICE_I2C_NANO "/dev/i2c-1"
#define DEVICE_I2C_NX "/dev/i2c-8"
class Control
{
    private:
        //  ******* properties ********
        // ros node
        ros::NodeHandle nh_;
        // ros servers
        ros::ServiceServer dev_info_srv_;
        //i2c
        I2CDriver i2c_driver_;
        bool i2c_error_;
        Pb6s40aDroneControl *drone_control_;
        Pb6s40aDroneControl *led_control_;
        //
        ERROR_WARN_LOG esc_error_logs_[4];
        ADB_DEVICE_INFO esc_device_infos_[4];

        //  ******* methods *******
        // init
        void Init();
        void DefaultValues();
        void SetupServices();
        // i2c
        void OpenI2C();
        void CloseI2C();
        //All
        void GetAll();
        //Esc
        void GetEscErrorLog();
        void GetEscDataLog();
        void GetEscDeviceInfo();
        //Board
        void GetBoardDeviceInfo();
        //Callback for service
        bool CallbackDeviceInfo(ae_powerboard_control::GetDeviceInfo::Request &req, ae_powerboard_control::GetDeviceInfo::Response &res);
    
    public:
        // constructor
        Control(const ros::NodeHandle &nh);
        ~Control();
};

#endif //CONTROL_NODE_HPP