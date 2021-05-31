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
    this->GetEscDataLog();
    this->GetDeviceInfo();
    this->CloseI2C();
}

void Control::DefaultValues()
{
    drone_control_ = new Pb6s40aDroneControl(i2c_driver_);
    
}

void Control::OpenI2C()
{
    std::string device(DEVICE_I2C_NANO);

    i2c_error_ = i2c_driver_.I2cOpen(device.c_str());

    if(i2c_error_)
    {
        throw (std::string("I2C error happens when opening port: ") + device.c_str());
    }
}

void Control::GetEscDataLog()
{
    if(i2c_error_)
    {
        return;
    }
    RUN_DATA_Struct esc_data_logs[4];
    uint8_t status = 0;
    status= drone_control_->EscGetDataLogs(&esc_data_logs[0],esc1);      
    status= drone_control_->EscGetDataLogs(&esc_data_logs[1],esc2);      
    status= drone_control_->EscGetDataLogs(&esc_data_logs[2],esc3);      
    status= drone_control_->EscGetDataLogs(&esc_data_logs[3],esc4);
    for (int i = 0; i < 4; i++)
    {
        ROS_INFO("Status: %d, Is_max: %d, Is_avg: %d, Esc_temp_max: %d, Motor_temp_max: %d", esc_data_logs[i].Diagnostic_status,
        esc_data_logs[i].Is_Motor_Max, esc_data_logs[i].Is_Motor_Avg, esc_data_logs[i].Temp_ESC_Max,
        esc_data_logs[i].Temp_Motor_Max);
    }
}

void Control::GetDeviceInfo()
{
    if(i2c_error_)
    {
        return;
    }
    ADB_DEVICE_INFO esc_device_infos[4];
    uint8_t status = 0;
    status= drone_control_->EscGetDeviceInfo(&esc_device_infos[0],esc1); 
    status= drone_control_->EscGetDeviceInfo(&esc_device_infos[1],esc2);
    status= drone_control_->EscGetDeviceInfo(&esc_device_infos[2],esc3);
    status= drone_control_->EscGetDeviceInfo(&esc_device_infos[3],esc4);
    for (int i = 0; i < 4; i++)
    {
        ROS_INFO("Status: %d, Fw: %d.%d.%d, Address: %d, Hw build: %d, Sn: %d", esc_device_infos[i].Diagnostic_status,
        esc_device_infos[i].fw_number.major, esc_device_infos[i].fw_number.mid, esc_device_infos[i].fw_number.minor,
        esc_device_infos[i].device_address, esc_device_infos[i].hw_build, esc_device_infos[i].serial_number);
    }
}

void Control::CloseI2C()
{
    i2c_driver_.I2cClose();
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