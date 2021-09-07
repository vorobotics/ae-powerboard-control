#include "control_node.hpp"

Control::Control(const ros::NodeHandle &nh)
    :nh_(nh)
{
    this->Init();
    this->SetupServices();
    this->SetupTimers();
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
    led_control_ = new Pb6s40aLedsControl(i2c_driver_);
    esc_device_info_status_ = 0x00;
    led_effect_run_ = false;
}

void Control::SetupServices()
{
    // servers
    esc_dev_info_srv_ = nh_.advertiseService("/ae_powerboard_control/esc/get_dev_info", &Control::CallbackEscDeviceInfo, this);
    esc_error_log_srv_ = nh_.advertiseService("/ae_powerboard_control/esc/get_error_log", &Control::CallbackEscErrorLog, this);
    esc_data_log_srv_ = nh_.advertiseService("/ae_powerboard_control/esc/get_data_log", &Control::CallbackEscDataLog, this);
    board_dev_info_srv_ = nh_.advertiseService("/ae_powerboard_control/board/get_dev_info", &Control::CallbackBoardDeviceInfo, this);
    led_set_custom_color_srv_ = nh_.advertiseService("/ae_powerboard_control/led/set_custom_color", &Control::CallbackLedCustomColor, this);
    led_set_color_srv_ = nh_.advertiseService("/ae_powerboard_control/led/set_color", &Control::CallbackLedColor, this);
    led_set_custom_effect_srv_ = nh_.advertiseService("/ae_powerboard_control/led/set_custom_effect", &Control::CallbackLedCustomEffect, this);
    led_set_effect_srv_ = nh_.advertiseService("/ae_powerboard_control/led/set_effect", &Control::CallbackLedEffect, this);
}

void Control::SetupTimers()
{
    main_tim_ = nh_.createTimer(ros::Duration(MAIN_TIME_PERIOD_S), &Control::CallbackMainTimer, this);
}

void Control::CallbackMainTimer(const ros::TimerEvent &event)
{
    static uint64_t ticks = 0;

    ticks++;
    
    if(!led_effect_run_)
    {
        return;
    }

    switch(led_effect_type_)
    {
        case NO_EFFECT:
            this->HandleNoEffect(ticks);
            break;
        case FLIGHT_MODE:
            this->HandleFlightModeEffect(ticks);
            break;
    }
    
}

void Control::HandleFlightModeEffect(uint64_t ticks)
{
    static bool front_switcher = false;
    static bool rear_switcher = false;
    static uint64_t tick_offset = 0;
    static COLOR color_buffer_front_d[LED_COUNT_EFFECT] = {WHITE, WHITE, WHITE, WHITE, OFFCOLOR, OFFCOLOR, OFFCOLOR, OFFCOLOR};
    static COLOR color_buffer_front_r[LED_COUNT_EFFECT] = {OFFCOLOR, OFFCOLOR, OFFCOLOR, OFFCOLOR, WHITE, WHITE, WHITE, WHITE};
    static COLOR color_buffer_rear_d[LED_COUNT_EFFECT] = {RED, RED, RED, RED, OFFCOLOR, OFFCOLOR, OFFCOLOR, OFFCOLOR};
    static COLOR color_buffer_rear_r[LED_COUNT_EFFECT] = {OFFCOLOR, OFFCOLOR, OFFCOLOR, OFFCOLOR, RED, RED, RED, RED};

    bool update_color = false;

    if(led_effect_update_)
    {
        tick_offset = ticks;
        front_switcher = false;
        rear_switcher = false;

        led_effect_update_ = false;
    }

    if((ticks - tick_offset) % 4 == 0)
    {
        front_switcher = !front_switcher;
        update_color = true;
    }

    if((ticks - tick_offset) % 8 == 0)
    {
        rear_switcher = !rear_switcher;
        update_color = true;
    }

    if(update_color)
    {
        led_control_->LedsSendColorBuffer(fl_buffer, (front_switcher ? color_buffer_front_d : color_buffer_front_r), LED_COUNT_EFFECT);
        led_control_->LedsSendColorBuffer(fr_buffer, (front_switcher ? color_buffer_front_d : color_buffer_front_r), LED_COUNT_EFFECT);
        led_control_->LedsSendColorBuffer(rl_buffer, (rear_switcher ? color_buffer_rear_d : color_buffer_rear_r), LED_COUNT_EFFECT);
        led_control_->LedsSendColorBuffer(rr_buffer, (rear_switcher ? color_buffer_rear_d : color_buffer_rear_r), LED_COUNT_EFFECT);

        led_control_->LedsUpdate();
    }
    

}

void Control::HandleNoEffect(uint64_t ticks)
{
    if(led_effect_update_)
    {
        //prepartion of color
        COLOR color_buffer[LED_COUNT_EFFECT];
        led_control_->LedsSetBufferWithOneColor(color_buffer, OFFCOLOR, LED_COUNT_EFFECT);
        led_control_->LedsSendColorBuffer(fl_buffer, color_buffer, LED_COUNT_EFFECT);
        led_control_->LedsSendColorBuffer(fr_buffer, color_buffer, LED_COUNT_EFFECT);
        led_control_->LedsSendColorBuffer(rl_buffer, color_buffer, LED_COUNT_EFFECT);
        led_control_->LedsSendColorBuffer(rr_buffer, color_buffer, LED_COUNT_EFFECT);
        
        //update led buffer
        led_control_->LedsUpdate();

        led_effect_update_ = false;
    }
}

bool Control::CallbackLedColor(ae_powerboard_control::SetLedColor::Request &req, ae_powerboard_control::SetLedColor::Response &res)
{
    if(i2c_error_)
    {
        res.success = false;
        return true;
    }

    //turn off predefinned effect
    led_effect_run_ = false;
    led_control_->LedsSwitchPredefinedEffect(false);
    
    //update led count
    LEDS_COUNT leds_count;
    led_control_->LedsGetLedsCount(leds_count);
    leds_count.fl_leds_count = req.leds_count;
    leds_count.fr_leds_count = req.leds_count;
    leds_count.rl_leds_count = req.leds_count;
    leds_count.rr_leds_count = req.leds_count;
    if(req.enable_add)
    {
        leds_count.ad_leds_count = req.leds_add_count;
    }
    led_control_->LedsSetLedsCount(leds_count);
    
    //front_left
    COLOR color_buffer_fl[req.leds_count];
    led_control_->LedsSetBufferWithOneColor(color_buffer_fl, *((COLOR*)&req.leds_color), req.leds_count);
    led_control_->LedsSendColorBuffer(fl_buffer, color_buffer_fl, req.leds_count);

    //front_right
    COLOR color_buffer_fr[req.leds_count];
    led_control_->LedsSetBufferWithOneColor(color_buffer_fr, *((COLOR*)&req.leds_color), req.leds_count);
    led_control_->LedsSendColorBuffer(fr_buffer, color_buffer_fr, req.leds_count);

    //rear_left
    COLOR color_buffer_rl[req.leds_count];
    led_control_->LedsSetBufferWithOneColor(color_buffer_rl, *((COLOR*)&req.leds_color), req.leds_count);
    led_control_->LedsSendColorBuffer(rl_buffer, color_buffer_rl, req.leds_count);

    //rear_right
    COLOR color_buffer_rr[req.leds_count];
    led_control_->LedsSetBufferWithOneColor(color_buffer_rr, *((COLOR*)&req.leds_color), req.leds_count);
    led_control_->LedsSendColorBuffer(rr_buffer, color_buffer_rr, req.leds_count);

    //additional
    if(req.enable_add)
    {
        COLOR color_buffer_ad[req.leds_count];
        led_control_->LedsSetBufferWithOneColor(color_buffer_ad, *((COLOR*)&req.add_color), req.leds_add_count);
        led_control_->LedsSendColorBuffer(ad_buffer, color_buffer_ad, req.leds_add_count);
    }

    //update led buffer
    led_control_->LedsUpdate();

    res.success = true;
    return true;
}

bool Control::CallbackLedCustomColor(ae_powerboard_control::SetLedCustomColor::Request &req, ae_powerboard_control::SetLedCustomColor::Response &res)
{
    if(i2c_error_)
    {
        res.success = false;
        return true;
    }

    //turn off predefinned effect
    led_effect_run_ = false;
    led_control_->LedsSwitchPredefinedEffect(false);
    
    //update led count
    LEDS_COUNT leds_count;
    led_control_->LedsGetLedsCount(leds_count);
    leds_count.fl_leds_count = req.front_left.color.size();
    leds_count.fr_leds_count = req.front_right.color.size();
    leds_count.rl_leds_count = req.rear_left.color.size();
    leds_count.rr_leds_count = req.rear_right.color.size();
    if(req.enable_add)
    {
        leds_count.ad_leds_count = req.add.color.size();
    }
    led_control_->LedsSetLedsCount(leds_count);

    //front_left
    COLOR color_buffer_fl[req.front_left.color.size()];
    memcpy(color_buffer_fl, req.front_left.color.data(), req.front_left.color.size()*sizeof(COLOR));
    led_control_->LedsSendColorBuffer(fl_buffer, color_buffer_fl, req.front_left.color.size());

    //front_right
    COLOR color_buffer_fr[req.front_right.color.size()];
    memcpy(color_buffer_fr, req.front_right.color.data(), req.front_right.color.size()*sizeof(COLOR));
    led_control_->LedsSendColorBuffer(fr_buffer, color_buffer_fr, req.front_right.color.size());

    //rear_left
    COLOR color_buffer_rl[req.rear_left.color.size()];
    memcpy(color_buffer_rl, req.rear_left.color.data(), req.rear_left.color.size()*sizeof(COLOR));
    led_control_->LedsSendColorBuffer(rl_buffer, color_buffer_rl, req.rear_left.color.size());

    //rear_right
    COLOR color_buffer_rr[req.rear_right.color.size()];
    memcpy(color_buffer_rr, req.rear_right.color.data(), req.rear_right.color.size()*sizeof(COLOR));
    led_control_->LedsSendColorBuffer(rr_buffer, color_buffer_rr, req.rear_right.color.size());

    //additional
    if(req.enable_add)
    {
        COLOR color_buffer_ad[req.add.color.size()];
        memcpy(color_buffer_ad, req.add.color.data(), req.add.color.size()*sizeof(COLOR));
        led_control_->LedsSendColorBuffer(ad_buffer, color_buffer_ad, req.add.color.size());
    }

    //update led buffercolor_buffer_rr
    led_control_->LedsUpdate();

    res.success = true;
    return true;
}

bool Control::CallbackLedEffect(ae_powerboard_control::SetLedEffect::Request &req, ae_powerboard_control::SetLedEffect::Response &res)
{
    //turn off predefinned effect
    led_effect_run_ = false;
    led_control_->LedsSwitchPredefinedEffect(false);

    //update led count
    LEDS_COUNT leds_count;
    led_control_->LedsGetLedsCount(leds_count);
    leds_count.fl_leds_count = LED_COUNT_EFFECT;
    leds_count.fr_leds_count = LED_COUNT_EFFECT;
    leds_count.rl_leds_count = LED_COUNT_EFFECT;
    leds_count.rr_leds_count = LED_COUNT_EFFECT;
    led_control_->LedsSetLedsCount(leds_count);

    led_effect_type_ = req.effect_type;
    led_effect_run_ = true;
    led_effect_update_ = true;

    res.success = true;
    return true;
}

bool Control::CallbackLedCustomEffect(ae_powerboard_control::SetLedCustomEffect::Request &req, ae_powerboard_control::SetLedCustomEffect::Response &res)
{
    //turn off predefinned effect
    led_effect_run_ = false;
    led_control_->LedsSwitchPredefinedEffect(false);

    //update led count
    LEDS_COUNT leds_count;
    led_control_->LedsGetLedsCount(leds_count);
    leds_count.fl_leds_count = req.leds_count;
    leds_count.fr_leds_count = req.leds_count;
    leds_count.rl_leds_count = req.leds_count;
    leds_count.rr_leds_count = req.leds_count;
    led_control_->LedsSetLedsCount(leds_count);

    //front_left
    COLOR color_buffer_fl[req.leds_count];
    led_control_->LedsSetBufferWithOneColor(color_buffer_fl, *((COLOR*)&req.front_left), req.leds_count);

    //front_right
    COLOR color_buffer_fr[req.leds_count];
    led_control_->LedsSetBufferWithOneColor(color_buffer_fr, *((COLOR*)&req.front_right), req.leds_count);

    //rear_leftcolor_buffer_rr
    COLOR color_buffer_rl[req.leds_count];
    led_control_->LedsSetBufferWithOneColor(color_buffer_rl, *((COLOR*)&req.rear_left), req.leds_count);

    //rear_right
    COLOR color_buffer_rr[req.leds_count];
    led_control_->LedsSetBufferWithOneColor(color_buffer_rr, *((COLOR*)&req.rear_right), req.leds_count);

    //set predefined effect
    led_control_->LedsSetPredefinedEffect(*((COLOR*)&req.front_left), *((COLOR*)&req.front_right), *((COLOR*)&req.rear_left), *((COLOR*)&req.rear_right), req.on_led_cycles, req.off_led_cycles, req.effect_type, req.set_default);

    //update led buffer
    led_control_->LedsUpdate();

    //turn on predefinned effect
    led_control_->LedsSwitchPredefinedEffect(true);

    res.success = true;
    return true;
}

bool Control::CallbackEscDeviceInfo(ae_powerboard_control::GetEscDeviceInfo::Request &req, ae_powerboard_control::GetEscDeviceInfo::Response &res)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        ae_powerboard_control::EscDeviceInfo dev_info;
        dev_info.esc_number = esc1 + i;
        dev_info.hw_build = esc_device_info_[i].hw_build;
        dev_info.serial_number = esc_device_info_[i].serial_number;
        dev_info.diagnostic_status = esc_device_info_[i].Diagnostic_status;
        dev_info.address = esc_device_info_[i].device_address;
        dev_info.test = esc_device_info_[i].hw_build & 0x01;
        dev_info.fw_version.high = esc_device_info_[i].fw_number.major;
        dev_info.fw_version.mid = esc_device_info_[i].fw_number.mid;
        dev_info.fw_version.low = esc_device_info_[i].fw_number.minor;
        dev_info.valid = esc_device_info_status_ & (1 << i);
        res.devices_info.push_back(dev_info);
    }
    return true;
}

bool Control::CallbackBoardDeviceInfo(ae_powerboard_control::GetBoardDeviceInfo::Request &req, ae_powerboard_control::GetBoardDeviceInfo::Response &res)
{
    ae_powerboard_control::BoardDeviceInfo dev_info;
    dev_info.hw_build = board_device_info_.hw_build;
    dev_info.serial_number = board_device_info_.serial_number;
    dev_info.test = board_device_info_.hw_build & 0x01;
    dev_info.fw_version.high = board_device_info_.fw_number.major;
    dev_info.fw_version.mid = board_device_info_.fw_number.mid;
    dev_info.fw_version.low = board_device_info_.fw_number.minor;
    dev_info.valid = board_device_info_status_;
    res.device_info = dev_info;

    return true;
}

bool Control::CallbackEscErrorLog(ae_powerboard_control::GetEscErrorLog::Request &req, ae_powerboard_control::GetEscErrorLog::Response &res)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        ae_powerboard_control::EscErrorLog error_log;
        error_log.esc_number = esc1 + i;
        error_log.diagnostic_status = esc_error_log_[i].Diagnostic_status;
        error_log.valid = esc_error_log_status_ & (1 << i);
        error_log.last.error = esc_error_log_[i].Last.Error;
        error_log.last.warning = esc_error_log_[i].Last.Warn;
        error_log.previous.error = esc_error_log_[i].Prev.Error;
        error_log.previous.warning = esc_error_log_[i].Prev.Warn;
        error_log.all.error = esc_error_log_[i].All.Error;
        error_log.all.warning = esc_error_log_[i].All.Warn;
        res.error_log.push_back(error_log);
    }
    return true;
}

bool Control::CallbackEscDataLog(ae_powerboard_control::GetEscDataLog::Request &req, ae_powerboard_control::GetEscDataLog::Response &res)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        ae_powerboard_control::EscDataLog data_log;
        data_log.esc_number = esc1 + i;
        data_log.diagnostic_status = esc_data_log_[i].Diagnostic_status;
        data_log.valid = esc_data_log_status_ & (1 << i);
        data_log.motor_max_is = Utils::ConvertFixedToFloat(esc_data_log_[i].Is_Motor_Max, Utils::I4Q8, 0);
        data_log.motor_avg_is = esc_data_log_[i].Is_Motor_Avg * 0.1f;
        data_log.motor_max_temp = esc_data_log_[i].Temp_Motor_Max - 50;
        data_log.esc_max_temp = esc_data_log_[i].Temp_ESC_Max - 50;
        res.data_log.push_back(data_log);
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
            ROS_ERROR("ESC%d ERROR LOG - problem reading data", (esc1 + i));
        }
        else
        {
            ROS_INFO("ESC%d ERROR LOG - Status: %u, Last E: 0x%x W: 0x%x, Prev E: 0x%x W: 0x%x, All E: 0x%x W: 0x%x", (esc1 + i),  
                er_log.Diagnostic_status, er_log.Last.Error, er_log.Last.Warn, er_log.Prev.Error, er_log.Prev.Warn,
                er_log.All.Error, er_log.All.Warn);
            esc_error_log_[i] = er_log;
            esc_error_log_status_ |= (1 << i);
        }
    }
}

void Control::GetEscDataLog()
{
    esc_data_log_status_ = 0x00;
    if(i2c_error_)
    {
        return;
    }

    for (int i = 0; i < 4; i++)
    {
        RUN_DATA_Struct data_log;
        uint8_t status = drone_control_->EscGetDataLogs(&data_log, (esc1 + i)); 
        if(status)
        {
            ROS_ERROR("ESC%d DATA - problem reading data", (esc1 + i));
        } 
        else
        {
            ROS_INFO("ESC%d DATA - Status: %d, Is_max: %f, Is_avg: %f, Esc_temp_max: %d, Motor_temp_max: %d", (esc1 + i),
                data_log.Diagnostic_status, Utils::ConvertFixedToFloat(data_log.Is_Motor_Max, Utils::I4Q8, 0), 
                data_log.Is_Motor_Avg * 0.1f, data_log.Temp_ESC_Max - 50, data_log.Temp_Motor_Max - 50);
            esc_data_log_[i] = data_log;
            esc_data_log_status_ |= (1 << i);
        }
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
        if(drone_control_->EscGetDeviceInfo(&dev_info, (esc1 + i)))
        {
            ROS_INFO("ESC%d INFO - problem reading data", (esc1 + i));
        }
        else
        {
            ROS_INFO("ESC%d INFO - Status: %u, Fw: %u.%u.%u, Address: %u, Hw build: %u, Sn: %u", (esc1 + i), dev_info.Diagnostic_status,
                dev_info.fw_number.major, dev_info.fw_number.mid, dev_info.fw_number.minor, dev_info.device_address,
                dev_info.hw_build, dev_info.serial_number);
            esc_device_info_[i] = dev_info;
            esc_device_info_status_ |= (1 << i);
        }
    }
}

void Control::GetBoardDeviceInfo()
{
    board_device_info_status_ = false;
    if(i2c_error_)
    {
        return;
    }
    
    POWER_BOARD_INFO dev_info;
    if(drone_control_->PowerBoardInfoGet(&dev_info))
    {
        ROS_INFO("BOARD INFO - problem reading data");
    }
    else
    {
        ROS_INFO("BOARD INFO - Fw: %u.%u.%u, Hw build: %u, Sn: %u", dev_info.fw_number.major, dev_info.fw_number.mid, 
          dev_info.fw_number.minor, dev_info.hw_build, dev_info.serial_number);
        board_device_info_ = dev_info;
        board_device_info_status_ = true;
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