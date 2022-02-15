#include "ros/ros.h"
#include "ae_powerboard_control/SetLedCustomColor.h"

/*
*Define number of mounted LEDs
*/

#define LED_COUNT 8  
#define LED_COUNT_ADD 10 

/*
*  Function to fill and call service.
*  set_custom_color service callback fills and sends LED channel buffers according to service request 
*/
bool SetCustomColor(ros::NodeHandle nh)
{
    ros::ServiceClient led_set_custom_color_cl = nh.serviceClient<ae_powerboard_control::SetLedCustomColor>("/ae_powerboard_control/led/set_custom_color");

    ae_powerboard_control::SetLedCustomColor led_custom_color;

    /*Disable additional LED channel*/
    led_custom_color.request.enable_add = false;

    ae_powerboard_control::Color color;
    color.r = 255;
    color.g = 0;
    color.b = 255;

    /*
    * Fill buffers with desired colors
    * you can fill array manually with random colors
    */
    led_custom_color.request.front_left.color.resize(LED_COUNT);
    std::fill(led_custom_color.request.front_left.color.begin(), 
              led_custom_color.request.front_left.color.end(), 
              color);
    
    led_custom_color.request.front_right.color.resize(LED_COUNT);
    std::fill(led_custom_color.request.front_right.color.begin(), 
              led_custom_color.request.front_right.color.end(), 
              color);
    
    led_custom_color.request.rear_left.color.resize(LED_COUNT);
    std::fill(led_custom_color.request.rear_left.color.begin(), 
              led_custom_color.request.rear_left.color.end(), 
              color);

    led_custom_color.request.rear_right.color.resize(LED_COUNT);
    std::fill(led_custom_color.request.rear_right.color.begin(), 
              led_custom_color.request.rear_right.color.end(), 
              color);

    /*Fill buffer for additional LED channel*/
    /*led_custom_color.request.add.color.resize(LED_COUNT_ADD);
    std::fill(led_custom_color.request.rear_right.color.begin(), 
              led_custom_color.request.rear_right.color.end(), 
              color);*/

    if (!led_set_custom_color_cl.exists())
    {
        ROS_ERROR("Service \"/ae_powerboard_control/led/set_custom_color\" does not exist!");
        return false;
    }
    
    if (!led_set_custom_color_cl.call(led_custom_color))
    {
        ROS_ERROR("Service \"/ae_powerboard_control/led/set_custom_color\" call was not successful!");
        return false;
    }
    
    if (!led_custom_color.response.success)
    {
        ROS_ERROR("Service \"/ae_powerboard_control/led/set_custom_color\" was not successful!");
        return false;
    }

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_led_custom_color");
    ros::NodeHandle nh;


    if (SetCustomColor(nh))
        ROS_INFO("Custom led colors set successfully");
        
    return EXIT_SUCCESS;
}