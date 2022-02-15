#include "ros/ros.h"
#include "ae_powerboard_control/SetLedColor.h"


#define LED_COUNT 8  // Define number of mounted LEDs
#define LED_COUNT_ADD 10 //Define number of mounted leds on additional channel

/*
*  set_color service callback fills all 4 buffers with same color according to service request
*  additional LEDs channel is filled separatelly  
*/
bool SetOneColor(ros::NodeHandle nh)
{
    ros::ServiceClient led_set_one_color_cl = nh.serviceClient<ae_powerboard_control::SetLedColor>("/ae_powerboard_control/led/set_color");

    ae_powerboard_control::SetLedColor led_set_one_color;
    
    ae_powerboard_control::Color color;
    color.r = 255;
    color.g = 0;
    color.b = 0;

    led_set_one_color.request.leds_count = LED_COUNT;
    led_set_one_color.request.leds_color = color;

    /*Disable additional LED channel*/
    led_set_one_color.request.enable_add = false;
    
    led_set_one_color.request.leds_add_count = LED_COUNT_ADD;
    led_set_one_color.request.add_color = color;

    if (!led_set_one_color_cl.exists())
    {
        ROS_ERROR("Service \"/ae_powerboard_control/led/set_color\" does not exist!");
        return false;
    }
    
    if (!led_set_one_color_cl.call(led_set_one_color))
    {
        ROS_ERROR("Service \"/ae_powerboard_control/led/set_color\" call was not successful!");
        return false;
    }
    
    if (!led_set_one_color.response.success)
    {
        ROS_ERROR("Service \"/ae_powerboard_control/led/set_color\" was not successful!");
        return false;
    }

    return true;

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_led_one_color");
    ros::NodeHandle nh;
     
    if (SetOneColor(nh))
        ROS_INFO("All channels successfully set with selected color.");

    
    return EXIT_SUCCESS;
}