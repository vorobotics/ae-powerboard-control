#include "ros/ros.h"
#include "ae_powerboard_control/SetLedPredefinedEffect.h"

#define LED_COUNT 8  // Define number of mounted LEDs

/*
*  In this example predefined effect handled by DroneCore.Power is set. You just need to set up parameters and
*  .Power board will handle it without any other command needed. There are two predefined effects in the 
*  present: 1=Toggling(turn on/turn off), 2=Circle(this effect is intended for use with Airvolute drone_arm_led_ring).
*/
bool SetPredefinedEffect(ros::NodeHandle nh)
{
    ros::ServiceClient led_set_predefined_effect_cl = nh.serviceClient<ae_powerboard_control::SetLedPredefinedEffect>("/ae_powerboard_control/led/set_predefined_effect");

    ae_powerboard_control::SetLedPredefinedEffect led_set_predefined_effect;
    
    ae_powerboard_control::Color color_a;
    color_a.r = 0;
    color_a.g = 255;
    color_a.b = 0;

    ae_powerboard_control::Color color_b;
    color_b.r = 255;
    color_b.g = 0;
    color_b.b = 0;

    led_set_predefined_effect.request.leds_count = LED_COUNT;
    led_set_predefined_effect.request.on_led_cycles=20;
    led_set_predefined_effect.request.off_led_cycles=20;
    led_set_predefined_effect.request.effect_type=1;
    led_set_predefined_effect.request.front_left = color_a;
    led_set_predefined_effect.request.front_right = color_a;
    led_set_predefined_effect.request.rear_left = color_b;
    led_set_predefined_effect.request.rear_right = color_b;

    /*If set_default=true this effect will start immediatelly after drone restart and will owerwrite existing effect (toggling red on all channels).*/
    led_set_predefined_effect.request.set_default=false;



    if (!led_set_predefined_effect_cl.exists())
    {
        ROS_ERROR("Service \"/ae_powerboard_control/led/set_predefined_effect\" does not exist!");
        return false;
    }
    
    if (!led_set_predefined_effect_cl.call(led_set_predefined_effect))
    {
        ROS_ERROR("Service \"/ae_powerboard_control/led/set_predefined_effect\" call was not successful!");
        return false;
    }
    
    if (!led_set_predefined_effect.response.success)
    {
        ROS_ERROR("Service \"/ae_powerboard_control/led/set_predefined_effect\" was not successful!");
        return false;
    }

    return true;

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_set_predefined_effect");
    ros::NodeHandle nh;
     
    if (SetPredefinedEffect(nh))
        ROS_INFO("Predefined effect successfuly set.");

    
    return EXIT_SUCCESS;
}