#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/EstimatorStatus.h>
#include "ae_powerboard_control/SetLedCustomColor.h"

#define LED_COUNT 3  
#define LED_COUNT_ADD 0

ros::ServiceClient ledSetCustomColorSrv;
ros::NodeHandle nh;
bool isArmed = false;
bool ekfOk = false;


enum LedState {
    NONE,
    WARN,
    FLYING,
    READY
}
LedState currentLedState = LedState::NONE;

bool setLedWarn();
bool setLedFlying();
bool setLedReady();
bool setLedCustomColor(ae_powerboard_control::SetLedCustomColor &customColor); 
void stateCallback(const mavros_msgs/State::ConstPtr& msg);
void estimatorStatusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& msg);

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
    color.r = 255;
    color.g = 255;
    color.b = 0;

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
    color.r = 255;
    color.g = 255;
    color.b = 255;

    ae_powerboard_control::Color red;
    color.r = 255;
    color.g = 0;
    color.b = 0;

    ae_powerboard_control::Color green;
    color.r = 0;
    color.g = 255;
    color.b = 0;

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
    color.r = 255;
    color.g = 255;
    color.b = 255;

    ae_powerboard_control::Color blue;
    color.r = 0;
    color.g = 0;
    color.b = 255;

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
            setLedFlying();
            currentLedState = LedState::FLYING;
        }
    } else if (!isArmed && ekfOk) {
        if(currentLedState != LedState::READY) {
            setLedReady();
            currentLedState = LedState::READY;
        }
    } else {
        if(currentLedState != LedState::WARN) {
            setLedWarn();
            currentLedState = LedState::WARN;
        }
    }
}

// Callback function for EKF status
void estimatorStatusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& msg) {
    ekfOk = (msg->flags & mavros_msgs::EstimatorStatus::ESTIMATOR_PRED_POS_HORIZ_REL) != 0;
    if (!isArmed && ekfOk) {
        if(currentLedState != LedState::READY) {
            setLedReady();
            currentLedState = LedState::READY;
        }
    } else if (!isArmed && !ekfOk) {
        if(currentLedState != LedState::WARN) {
            setLedWarn();
            currentLedState = LedState::WARN;
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mav_colors");

    ledSetCustomColorSrv = nh.serviceClient<ae_powerboard_control::SetLedCustomColor>("/ae_powerboard_control/led/set_custom_color");
    if (!ledSetCustomColorSrv.exists())
    {
        ROS_FATAL("Service \"/ae_powerboard_control/led/set_custom_color\" does not exist!");
        return EXIT_FAILURE;
    }

    ROS_INFO("Starting MAV_COLORS");
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs/State>("mavros/state", 10, stateCallback);
    ros::Subscriber estimator_status_sub = nh.subscribe<mavros_msgs/EstimatorStatus>("mavros/estimator_status", 10, estimatorStatusCallback);
    ros::spin();
    return EXIT_SUCCESS;
}