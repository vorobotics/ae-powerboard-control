#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include "ros/ros.h"

#include "utils.hpp"
#include "i2c_driver.h" 
#include "pb6s40a_control.h"

#include "ae_powerboard_control/GetEscDeviceInfo.h"
#include "ae_powerboard_control/GetBoardDeviceInfo.h"
#include "ae_powerboard_control/GetEscErrorLog.h"
#include "ae_powerboard_control/GetEscDataLog.h"

#define DEVICE_I2C_NANO "/dev/i2c-1"
#define DEVICE_I2C_NX "/dev/i2c-8"
class Control
{
    private:
        //  ******* properties ********
        // ros node
        ros::NodeHandle nh_;
        // ros servers
        ros::ServiceServer esc_dev_info_srv_;
        ros::ServiceServer esc_error_log_srv_;
        ros::ServiceServer esc_data_log_srv_;
        ros::ServiceServer board_dev_info_srv_;
        //i2c
        I2CDriver i2c_driver_;
        bool i2c_error_;
        Pb6s40aDroneControl *drone_control_;
        Pb6s40aDroneControl *led_control_;
        //esc error log
        ERROR_WARN_LOG esc_error_log_[4];
        uint8_t esc_error_log_status_;
        //esc data log
        RUN_DATA_Struct esc_data_log_[4];
        uint8_t esc_data_log_status_;
        //esc device info
        ADB_DEVICE_INFO esc_device_info_[4];
        uint8_t esc_device_info_status_;
        //board device info
        POWER_BOARD_INFO board_device_info_;
        bool board_device_info_status_;

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
        bool CallbackEscDeviceInfo(ae_powerboard_control::GetEscDeviceInfo::Request &req, ae_powerboard_control::GetEscDeviceInfo::Response &res);
        bool CallbackEscErrorLog(ae_powerboard_control::GetEscErrorLog::Request &req, ae_powerboard_control::GetEscErrorLog::Response &res);
        bool CallbackEscDataLog(ae_powerboard_control::GetEscDataLog::Request &req, ae_powerboard_control::GetEscDataLog::Response &res);
        bool CallbackBoardDeviceInfo(ae_powerboard_control::GetBoardDeviceInfo::Request &req, ae_powerboard_control::GetBoardDeviceInfo::Response &res);
    
    public:
        // constructor
        Control(const ros::NodeHandle &nh);
        ~Control();
};

#endif //CONTROL_NODE_HPP