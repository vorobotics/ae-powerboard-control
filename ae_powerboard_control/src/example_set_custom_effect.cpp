#include "ros/ros.h"
#include "ae_powerboard_control/SetLedCustomEffect.h"



/*
*  This example enables selected custom effect and starts handling of user-custom effects in main timer. User can write own effects
*  and add handler to CallbackMainTimer. This effects are completelly handled from ROS. As example FLIGHT_MODE effect is writted in control_node.cpp
*
*  NOTE:    predefined effect is automatically disabled if custom effect is started. But there is possibility to control only additional LEDs channel 
            while predefined effect on front and rear channels is still running.
*/
bool SetCustomEffect(ros::NodeHandle nh)
{
    ros::ServiceClient led_set_custom_effect_cl = nh.serviceClient<ae_powerboard_control::SetLedCustomEffect>("/ae_powerboard_control/led/set_custom_effect");

    ae_powerboard_control::SetLedCustomEffect led_set_custom_effect;
    

    /*Choose desired effect*/
    led_set_custom_effect.request.effect_type = 1;

    /*If your custom effect drives only additional LEDs channel and you want the predefined effect to continue execution, set kill_predefined_effect as false */
    led_set_custom_effect.request.kill_predefined_effect = true;
    

    if (!led_set_custom_effect_cl.exists())
    {
        ROS_ERROR("Service \"/ae_powerboard_control/led/set_custom_effect\" does not exist!");
        return false;
    }
    
    if (!led_set_custom_effect_cl.call(led_set_custom_effect))
    {
        ROS_ERROR("Service \"/ae_powerboard_control/led/set_custom_effect\" call was not successful!");
        return false;
    }
    
    if (!led_set_custom_effect.response.success)
    {
        ROS_ERROR("Service \"/ae_powerboard_control/led/set_custom_effect\" was not successful!");
        return false;
    }

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_set_custom_effect");
    ros::NodeHandle nh;
     
    if (SetCustomEffect(nh))
        ROS_INFO("Custom effect execution was started successfully.");

    
    return EXIT_SUCCESS;
}