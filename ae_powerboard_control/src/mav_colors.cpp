#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GPSRAW.h>
#include "ae_powerboard_control/SetLedCustomColor.h"

#define LED_COUNT 3  
#define LED_COUNT_ADD 0

ros::ServiceClient ledSetCustomColorSrv;

bool isArmed = false;
bool gpsOk = false;

enum LedState {
    NONE,
    WARN,
    FLYING,
    READY
};
LedState currentLedState = LedState::NONE;

bool setLedWarn();
bool setLedFlying();
bool setLedReady();
bool setLedCustomColor(ae_powerboard_control::SetLedCustomColor &customColor); 
void stateCallback(const mavros_msgs::State::ConstPtr& msg);
void gpsStatusCallback(const mavros_msgs::GPSRAW::ConstPtr& msg);

bool setLedCustomColor(ae_powerboard_control::SetLedCustomColor &customColorCmd) 
{
    if (!ledSetCustomColorSrv.call(customColorCmd))
    {
        ROS_ERROR("Service \"/ae_powerboard_control/led/set_custom_color\" call was not successful!");
        return false;
    }
    
    if (!customColorCmd.response.success)
    {
        ROS_ERROR("Service \"/ae_powerboard_control/led/set_custom_color\" was not successful!");
        return false;
    }
    return true;
}

bool setLedWarn()
{
    ae_powerboard_control::SetLedCustomColor setLedWarnCmd;

    setLedWarnCmd.request.enable_add = false;

    ae_powerboard_control::Color yellow;
    yellow.r = 255;
    yellow.g = 255;
    yellow.b = 0;

    setLedWarnCmd.request.front_left.color.resize(LED_COUNT);
    std::fill(setLedWarnCmd.request.front_left.color.begin(), 
              setLedWarnCmd.request.front_left.color.end(), 
              yellow);
    
    setLedWarnCmd.request.front_right.color.resize(LED_COUNT);
    std::fill(setLedWarnCmd.request.front_right.color.begin(), 
              setLedWarnCmd.request.front_right.color.end(), 
              yellow);
    
    setLedWarnCmd.request.rear_left.color.resize(LED_COUNT);
    std::fill(setLedWarnCmd.request.rear_left.color.begin(), 
              setLedWarnCmd.request.rear_left.color.end(), 
              yellow);

    setLedWarnCmd.request.rear_right.color.resize(LED_COUNT);
    std::fill(setLedWarnCmd.request.rear_right.color.begin(), 
              setLedWarnCmd.request.rear_right.color.end(), 
              yellow);

    /*setLedWarnCmd.request.add.color.resize(LED_COUNT_ADD);
    std::fill(setLedWarnCmd.request.rear_right.color.begin(), 
              setLedWarnCmd.request.rear_right.color.end(), 
              color);*/

    return setLedCustomColor(setLedWarnCmd);
}

bool setLedFlying()
{
    ae_powerboard_control::SetLedCustomColor setLedFlyingCmd;

    setLedFlyingCmd.request.enable_add = false;

    ae_powerboard_control::Color white;
    white.r = 255;
    white.g = 255;
    white.b = 255;

    ae_powerboard_control::Color red;
    red.r = 255;
    red.g = 0;
    red.b = 0;

    ae_powerboard_control::Color green;
    green.r = 0;
    green.g = 255;
    green.b = 0;

    setLedFlyingCmd.request.front_left.color.resize(LED_COUNT);
    std::fill(setLedFlyingCmd.request.front_left.color.begin(), 
              setLedFlyingCmd.request.front_left.color.end(), 
              white);
    
    setLedFlyingCmd.request.front_right.color.resize(LED_COUNT);
    std::fill(setLedFlyingCmd.request.front_right.color.begin(), 
              setLedFlyingCmd.request.front_right.color.end(), 
              white);
    
    setLedFlyingCmd.request.rear_left.color.resize(LED_COUNT);
    std::fill(setLedFlyingCmd.request.rear_left.color.begin(), 
              setLedFlyingCmd.request.rear_left.color.end(), 
              red);

    setLedFlyingCmd.request.rear_right.color.resize(LED_COUNT);
    std::fill(setLedFlyingCmd.request.rear_right.color.begin(), 
              setLedFlyingCmd.request.rear_right.color.end(), 
              green);

    /*setLedFlyingCmd.request.add.color.resize(LED_COUNT_ADD);
    std::fill(setLedFlyingCmd.request.rear_right.color.begin(), 
              setLedFlyingCmd.request.rear_right.color.end(), 
              color);*/

    return setLedCustomColor(setLedFlyingCmd);
}

bool setLedReady()
{
    ae_powerboard_control::SetLedCustomColor setLedReadyCmd;

    setLedReadyCmd.request.enable_add = false;

    ae_powerboard_control::Color white;
    white.r = 255;
    white.g = 255;
    white.b = 255;

    ae_powerboard_control::Color blue;
    blue.r = 0;
    blue.g = 0;
    blue.b = 255;

    setLedReadyCmd.request.front_left.color.resize(LED_COUNT);
    std::fill(setLedReadyCmd.request.front_left.color.begin(), 
              setLedReadyCmd.request.front_left.color.end(), 
              white);
    
    setLedReadyCmd.request.front_right.color.resize(LED_COUNT);
    std::fill(setLedReadyCmd.request.front_right.color.begin(), 
              setLedReadyCmd.request.front_right.color.end(), 
              white);
    
    setLedReadyCmd.request.rear_left.color.resize(LED_COUNT);
    std::fill(setLedReadyCmd.request.rear_left.color.begin(), 
              setLedReadyCmd.request.rear_left.color.end(), 
              blue);

    setLedReadyCmd.request.rear_right.color.resize(LED_COUNT);
    std::fill(setLedReadyCmd.request.rear_right.color.begin(), 
              setLedReadyCmd.request.rear_right.color.end(), 
              blue);

    /*setLedReadyCmd.request.add.color.resize(LED_COUNT_ADD);
    std::fill(setLedReadyCmd.request.rear_right.color.begin(), 
              setLedReadyCmd.request.rear_right.color.end(), 
              color);*/

    return setLedCustomColor(setLedReadyCmd);
}

// Callback function for vehicle state
void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    isArmed = msg->armed;
    if (isArmed) {
        if(currentLedState != LedState::FLYING) {
            ROS_INFO("Vehicle is flying");
            setLedFlying();
            currentLedState = LedState::FLYING;
        }
    } else if (!isArmed && gpsOk) {
        if(currentLedState != LedState::READY) {
            ROS_INFO("Vehicle is ready");
            setLedReady();
            currentLedState = LedState::READY;
        }
    } else {
        if(currentLedState != LedState::WARN) {
            ROS_INFO("Vehicle is not ready");
            setLedWarn();
            currentLedState = LedState::WARN;
        }
    }
}

void gpsStatusCallback(const mavros_msgs::GPSRAW::ConstPtr& msg) {
    gpsOk = msg->fix_type >= 3;
    if (!isArmed && gpsOk) {
        if(currentLedState != LedState::READY) {
            ROS_INFO("Vehicle is ready");
            setLedReady();
            currentLedState = LedState::READY;
        }
    } else if (!isArmed && !gpsOk) {
        if(currentLedState != LedState::WARN) {
            ROS_INFO("Vehicle is not ready");
            setLedWarn();
            currentLedState = LedState::WARN;
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mav_colors");
    ros::NodeHandle nh;

    ledSetCustomColorSrv = nh.serviceClient<ae_powerboard_control::SetLedCustomColor>("/ae_powerboard_control/led/set_custom_color");
    if (!ledSetCustomColorSrv.exists())
    {
        ROS_FATAL("Service \"/ae_powerboard_control/led/set_custom_color\" does not exist!");
        return EXIT_FAILURE;
    }

    ROS_INFO("Starting MAV_COLORS");
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
    ros::Subscriber gps_sub = nh.subscribe<mavros_msgs::GPSRAW>("mavros/gpsstatus/gps1/raw", 10, gpsStatusCallback);
    ros::spin();
    return EXIT_SUCCESS;
}