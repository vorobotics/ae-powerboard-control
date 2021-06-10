#include "control_node.hpp"

Control::Control(const ros::NodeHandle &nh)
    :nh_(nh)
{
    this->Init();
    this->SetupServices();
    this->GetAll();
}

Control::~Control()
{
    this->CloseI2C();
    drone_control_ = NULL;
    delete drone_control_;
}

void Control::Init()
{
    this->DefaultValues();
    this->OpenI2C();
}

void Control::DefaultValues()
{
    drone_control_ = new Pb6s40aDroneControl(i2c_driver_);
    esc_device_info_status_ = 0x00;
}

void Control::SetupServices()
{
    // servers
    esc_dev_info_srv_ = nh_.advertiseService("/ae_powerboard_control/esc/get_dev_info", &Control::CallbackEscDeviceInfo, this);
    esc_error_log_srv_ = nh_.advertiseService("/ae_powerboard_control/esc/get_error_log", &Control::CallbackEscErrorLog, this);
}

bool Control::CallbackEscDeviceInfo(ae_powerboard_control::GetEscDeviceInfo::Request &req, ae_powerboard_control::GetEscDeviceInfo::Response &res)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        ae_powerboard_control::EscDeviceInfo dev_info;
        dev_info.esc_number = esc1 + i;
        dev_info.hw_build = esc_device_infos_[i].hw_build;
        dev_info.serial_number = esc_device_infos_[i].serial_number;
        dev_info.diagnostic_status = esc_device_infos_[i].Diagnostic_status;
        dev_info.address = esc_device_infos_[i].device_address;
        dev_info.test = esc_device_infos_[i].hw_build & 0x01;
        dev_info.fw_version.high = esc_device_infos_[i].fw_number.major;
        dev_info.fw_version.mid = esc_device_infos_[i].fw_number.mid;
        dev_info.fw_version.low = esc_device_infos_[i].fw_number.minor;
        dev_info.valid = esc_device_info_status_ & (1 << i);
        res.devices_info.push_back(dev_info);
    }
    return true;
}

bool Control::CallbackEscErrorLog(ae_powerboard_control::GetEscErrorLog::Request &req, ae_powerboard_control::GetEscErrorLog::Response &res)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        ae_powerboard_control::EscErrorLog error_log;
        error_log.esc_number = esc1 + i;
        error_log.diagnostic_status = esc_error_logs_[i].Diagnostic_status;
        error_log.valid = esc_error_log_status_ & (1 << i);
        error_log.last.error = esc_error_logs_[i].Last.Error;
        error_log.last.warning = esc_error_logs_[i].Last.Warn;
        error_log.previous.error = esc_error_logs_[i].Prev.Error;
        error_log.previous.warning = esc_error_logs_[i].Prev.Warn;
        error_log.all.error = esc_error_logs_[i].All.Error;
        error_log.all.warning = esc_error_logs_[i].All.Warn;
        res.error_log.push_back(error_log);
    }
    return true;
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

void Control::GetAll()
{
    this->GetEscErrorLog();
    this->GetEscDataLog();
    this->GetEscDeviceInfo();
    this->GetBoardDeviceInfo();
}

void Control::GetEscErrorLog()
{
    esc_error_log_status_ = 0x00;
    if(i2c_error_)
    {
        return;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        ERROR_WARN_LOG er_log = ERROR_WARN_LOG_INIT;
        uint8_t status = drone_control_->EscGetErrorLogs(&er_log, (esc1 + i));
        if(status)
        {
            ROS_ERROR("ESC%d ERROR LOG - problem reading data", i);
        }
        else
        {
            ROS_INFO("ESC%d ERROR LOG - Status: %u, Last E: 0x%x W: 0x%x, Prev E: 0x%x W: 0x%x, All E: 0x%x W: 0x%x", i,  
                er_log.Diagnostic_status, er_log.Last.Error, er_log.Last.Warn, er_log.Prev.Error, er_log.Prev.Warn,
                er_log.All.Error, er_log.All.Warn);
            esc_error_logs_[i] = er_log;
            esc_error_log_status_ |= (1 << i);
        }
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
    status = drone_control_->EscGetDataLogs(&esc_data_logs[0],esc1);      
    status = drone_control_->EscGetDataLogs(&esc_data_logs[1],esc2);      
    status = drone_control_->EscGetDataLogs(&esc_data_logs[2],esc3);      
    status = drone_control_->EscGetDataLogs(&esc_data_logs[3],esc4);
    for (int i = 0; i < 4; i++)
    {
        ROS_INFO("ESC DATA - Status: %d, Is_max: %f, Is_avg: %f, Esc_temp_max: %d, Motor_temp_max: %d", esc_data_logs[i].Diagnostic_status,
        Utils::ConvertFixedToFloat(esc_data_logs[i].Is_Motor_Max, Utils::I4Q8, 0), esc_data_logs[i].Is_Motor_Avg * 0.1f, 
        esc_data_logs[i].Temp_ESC_Max - 50, esc_data_logs[i].Temp_Motor_Max - 50);
    }
}

void Control::GetEscDeviceInfo()
{
    esc_device_info_status_ = 0x00;
    if(i2c_error_)
    {
        return;
    }

    for(uint8_t i = 0; i< 4; i++)
    {
        ADB_DEVICE_INFO dev_info;
        if(drone_control_->EscGetDeviceInfo(&dev_info, esc1 + i))
        {
            ROS_INFO("ESC%d INFO - - problem reading data", i);
        }
        else
        {
            ROS_INFO("ESC%d INFO - Status: %u, Fw: %u.%u.%u, Address: %u, Hw build: %u, Sn: %u", i, dev_info.Diagnostic_status,
                dev_info.fw_number.major, dev_info.fw_number.mid, dev_info.fw_number.minor, dev_info.device_address,
                dev_info.hw_build, dev_info.serial_number);
            esc_device_infos_[i] = dev_info;
            esc_device_info_status_ |= (1 << i);
        }
    }
}

void Control::GetBoardDeviceInfo()
{
    if(i2c_error_)
    {
        return;
    }
    POWER_BOARD_INFO power_board_info;
    uint8_t status = 0;
    status = drone_control_->PowerBoardInfoGet(&power_board_info);

    ROS_INFO("BOARD INFO - Status: %u, Fw: %u.%u.%u, Hw build: %u, Sn: %u", status, power_board_info.fw_number.major,
    power_board_info.fw_number.mid, power_board_info.fw_number.minor, power_board_info.hw_build, power_board_info.serial_number);
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