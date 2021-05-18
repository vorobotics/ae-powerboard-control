#include "control_node.hpp"

Control::Control(const ros::NodeHandle &nh)
    :nh_(nh)
{
    // Init();
}

void Control::Init()
{
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pb_control_node");
    ros::NodeHandle n;
       
    Control control(n);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::waitForShutdown();

    spinner.stop();

    return 0;
}