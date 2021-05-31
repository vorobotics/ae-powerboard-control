#include "control_node.hpp"

Control::Control(const ros::NodeHandle &nh)
    :nh_(nh)
{
    this->Init();
}

void Control::Init()
{
    this->DefaultValues();
    this->OpenI2C();
    this->CloseI2C();
}

void Control::DefaultValues()
{
    drone_control_ = new Pb6s40aDroneControl(i2c1_driver_);
    
}

void Control::OpenI2C()
{
    std::string device(DEVICE_I2C);
    if(i2c1_driver_.I2cOpen(device.c_str()) != 0)
    {
        throw (std::string("I2C error happens when opening port: ") + device.c_str());
    }
    RUN_DATA_Struct esc_data_logs[4];
    uint8_t status=0;
    status= drone_control_->EscGetDataLogs(&esc_data_logs[0],esc1);      
    status= drone_control_->EscGetDataLogs(&esc_data_logs[1],esc2);      
    status= drone_control_->EscGetDataLogs(&esc_data_logs[2],esc3);      
    status= drone_control_->EscGetDataLogs(&esc_data_logs[3],esc4);
    ROS_ERROR("Result: %d, %d", esc_data_logs[0], esc_data_logs[1]);
}

void Control::CloseI2C()
{
    i2c1_driver_.I2cClose();
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