#include "can_handler.hpp"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/sockios.h>
#include <unistd.h>
#include <chrono>
#include "libsocketcan.h"
#include "raw_conversions.hpp"

using namespace std::chrono_literals;

CanHandler::CanHandler(rclcpp::NodeOptions nOpt):Node("CanInterface", "", nOpt)
{
    this->loadRosParams();
    this->variablesInit();

    //initialize can0
    /*if ((this->rosConf.channel0 != "vcan0") && (can_do_stop(this->rosConf.channel0.c_str()) != 0)) {
        RCLCPP_ERROR(this->get_logger(), "Unable to stop can0.");
        return;
    }
    if ((this->rosConf.channel0 != "vcan0") && (can_set_bitrate(this->rosConf.channel0.c_str(), this->rosConf.bitrate0) != 0)) {
        RCLCPP_ERROR(this->get_logger(), "Unable to set the bitrate on can0.");
        return;
    }
    if ((this->rosConf.channel0 != "vcan0") && (can_do_start(this->rosConf.channel0.c_str()) != 0)) {
        RCLCPP_ERROR(this->get_logger(), "Unable to start can0.");
        return;
    }*/  
    this->can0Socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->can0Socket < 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to create can0 socket.");
    }
    strcpy(this->ifr0.ifr_name, this->rosConf.channel0.c_str());
    if (ioctl(this->can0Socket, SIOCGIFINDEX, &this->ifr0) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Unable to perform ioctl for can0.\n");
        return;
    }
    this->addr0.can_family = AF_CAN;
    this->addr0.can_ifindex = this->ifr0.ifr_ifindex;
    if (bind(this->can0Socket, (struct sockaddr*)&this->addr0, sizeof(this->addr0)) < 0){
        RCLCPP_ERROR(this->get_logger(), "Unable to bind socket with can0.\n");
        return;
    }

    //initialize timers
    this->canRecvTimer = this->create_wall_timer(500us, std::bind(&CanHandler::handleCanReceive,    this));
    this->canSendTimer = this->create_wall_timer(1ms,   std::bind(&CanHandler::handleCanTransmite,  this));

    RCLCPP_INFO(this->get_logger(), "Can interface initialized");
}

CanHandler::~CanHandler()
{
    if (close(this->can0Socket) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to close can0.\n");
    }
}

void CanHandler::loadRosParams()
{
    //Channels configuration
    this->get_parameter_or<std::string>("channel0", this->rosConf.channel0, "vcan0");
    this->get_parameter_or<uint32_t>("bitrate0", this->rosConf.bitrate0, 1000000);
    //CAN messages to publish in ROS
    this->get_parameter_or<bool>("publishAMI505", this->rosConf.publishAMI505, true);
    this->get_parameter_or<bool>("publishAux166", this->rosConf.publishAux166, true);
    this->get_parameter_or<bool>("publishAux336", this->rosConf.publishAux336, true);
    this->get_parameter_or<bool>("publishAux357", this->rosConf.publishAux357, true);
    this->get_parameter_or<bool>("publishAux384", this->rosConf.publishAux384, true);
    this->get_parameter_or<bool>("publishAux412", this->rosConf.publishAux412, true);
    this->get_parameter_or<bool>("publishDash101", this->rosConf.publishDash101, true);
    this->get_parameter_or<bool>("publishDash107", this->rosConf.publishDash107, true);
    this->get_parameter_or<bool>("publishDash198", this->rosConf.publishDash198, true);
    this->get_parameter_or<bool>("publishDash204", this->rosConf.publishDash204, true);
    this->get_parameter_or<bool>("publishDash279", this->rosConf.publishDash279, true);
    this->get_parameter_or<bool>("publishEBS550", this->rosConf.publishEBS550, true);
    this->get_parameter_or<bool>("publishSWA510", this->rosConf.publishSWA510, true);
    //CAN messages to transmite
    this->get_parameter_or<bool>("transmiteAPU660", this->rosConf.transmiteAPU660, true);
    this->get_parameter_or<bool>("transmiteSWA539", this->rosConf.transmiteSWA539, true);
}

void CanHandler::variablesInit()
{
    //Initialize publishers
    if (this->rosConf.publishAMI505) {
        this->pubAMI505 = this->create_publisher<turtle_interfaces::msg::Mission>("ami_selected_mission", 10);
        this->msgAMI505 = turtle_interfaces::msg::Mission();
    }
    if (this->rosConf.publishAux166) {
        this->pubAux166 = this->create_publisher<turtle_interfaces::msg::BrakeLight>("brake_light", 10);
        this->msgAux166 = turtle_interfaces::msg::BrakeLight();
    }
    if (this->rosConf.publishAux336) {
        this->pubAux336 = this->create_publisher<turtle_interfaces::msg::EbsTankPressure>("ebs_tank_pressure", 10);
        this->msgAux336 = turtle_interfaces::msg::EbsTankPressure();
    }
    if (this->rosConf.publishAux357) {
        this->pubAux357 = this->create_publisher<turtle_interfaces::msg::RPM>("rpm_rear", 10);
        this->msgAux357 = turtle_interfaces::msg::RPM();
    }
    if (this->rosConf.publishAux384) {
        this->pubAux384 = this->create_publisher<turtle_interfaces::msg::TsalSafeState>("tsal_safe_state", 10);
        this->msgAux384 = turtle_interfaces::msg::TsalSafeState();
    }
    if (this->rosConf.publishAux412) {
        this->pubAux412 = this->create_publisher<turtle_interfaces::msg::CoolingInfo>("cooling_info", 10);
        this->msgAux412 = turtle_interfaces::msg::CoolingInfo();
    }
    if (this->rosConf.publishDash101) {
        this->pubDash101 = this->create_publisher<turtle_interfaces::msg::Apps>("apps", 10);
        this->msgDash101 = turtle_interfaces::msg::Apps();
    }
    if (this->rosConf.publishDash107) {
        this->pubDash107 = this->create_publisher<turtle_interfaces::msg::RPM>("rpm_front", 10);
        this->msgDash107 = turtle_interfaces::msg::RPM();
    }
    if (this->rosConf.publishDash198) {
        this->pubDash198 = this->create_publisher<turtle_interfaces::msg::Brake>("brake", 10);
        this->msgDash198 = turtle_interfaces::msg::Brake();
    }
    if (this->rosConf.publishDash204) {
        this->pubDash204 = this->create_publisher<turtle_interfaces::msg::DashLeds>("dash_leds", 10);
        this->msgDash204 = turtle_interfaces::msg::DashLeds();
    }
    if (this->rosConf.publishDash279) {
        this->pubDash279 = this->create_publisher<turtle_interfaces::msg::DashButtons>("dash_buttons", 10);
        this->msgDash279 = turtle_interfaces::msg::DashButtons();
    }
    if (this->rosConf.publishEBS550) {
        this->pubEBS550= this->create_publisher<turtle_interfaces::msg::EbsSupervisorInfo>("ebs_supervisor_info", 10);
        this->msgEBS550 = turtle_interfaces::msg::EbsSupervisorInfo();
    } 
    if (this->rosConf.publishSWA510) {
        this->pubSWA510 = this->create_publisher<turtle_interfaces::msg::Steering>("steering_actual", 10);
        this->msgSWA510 = turtle_interfaces::msg::Steering();
    }

    //Initialize CAN Tx messages
    if (this->rosConf.transmiteAPU660) {
        this->frameAPU660.as_mission = CAN_AS_DASH_AUX_APU_660_AS_MISSION_NO_MISSION_CHOICE;
        this->frameAPU660.as_state = CAN_AS_DASH_AUX_APU_660_AS_STATE_AS_OFF_CHOICE;
    }
    if (this->rosConf.transmiteSWA539) {
        this->frameSWA530.steering_target = 0;
    }
}

//Functions for CAN receive
void CanHandler::handleCanReceive()
{
    while (recvfrom(this->can0Socket, &this->recvFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, &this->len) >= 8) {
        ioctl(this->can0Socket, SIOCGSTAMP, &this->recvTime);  //get message timestamp

        if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AMI_505_FRAME_ID && this->rosConf.publishAMI505) {
            this->publish_ami_505();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AUX_166_FRAME_ID && this->rosConf.publishAux166) {
            this->publish_aux_166();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AUX_336_FRAME_ID && this->rosConf.publishAux336) {
            this->publish_aux_336();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AUX_357_FRAME_ID && this->rosConf.publishAux357) {
            this->publish_aux_357();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AUX_384_FRAME_ID && this->rosConf.publishAux384) {
            this->publish_aux_384();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AUX_412_FRAME_ID && this->rosConf.publishAux412) {
            this->publish_aux_412();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_DASH_101_FRAME_ID && this->rosConf.publishDash101) {
            this->publish_dash_101();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_DASH_107_FRAME_ID && this->rosConf.publishDash107) {
            this->publish_dash_107();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_DASH_198_FRAME_ID && this->rosConf.publishDash198) {
            this->publish_dash_198();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_DASH_204_FRAME_ID && this->rosConf.publishDash204) {
            this->publish_dash_204();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_DASH_279_FRAME_ID && this->rosConf.publishDash279) {
            this->publish_dash_279();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_EBS_550_FRAME_ID && this->rosConf.publishEBS550) {
            this->publish_ebs_550();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_SWA_510_FRAME_ID && this->rosConf.publishSWA510) {
            this->publish_swa_510();
        }
    }
}

void CanHandler::createHeader(std_msgs::msg::Header *header)
{
    header->frame_id = "";
    header->stamp.sec = this->recvTime.tv_sec;
    header->stamp.nanosec = this->recvTime.tv_usec*1000;
}

void CanHandler::publish_ami_505()
{
    can_as_dash_aux_ami_505_t msg;
    if (can_as_dash_aux_ami_505_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AMI_505");
        return;
    }

    this->createHeader(&this->msgAMI505.header);
    this->msgAMI505.mission = msg.ami_mission;

    this->pubAMI505->publish(this->msgAMI505);

    /********************************************************/
    this->frameAPU660.as_mission = this->msgAMI505.mission;
    /********************************************************/
}

void CanHandler::publish_aux_166()
{
    can_as_dash_aux_aux_166_t msg;
    if (can_as_dash_aux_aux_166_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_166");
        return;
    }

    this->createHeader(&this->msgAux166.header);
    this->msgAux166.brakelight = msg.brakelight;

    this->pubAux166->publish(this->msgAux166);
}

void CanHandler::publish_aux_336()
{
    can_as_dash_aux_aux_336_t msg;
    if (can_as_dash_aux_aux_336_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_336");
        return;
    }

    this->createHeader(&this->msgAux336.header);
    this->msgAux336.ebspressureraw = msg.ebs_pressure_raw;

    this->pubAux336->publish(this->msgAux336);
}

void CanHandler::publish_aux_357()
{
    can_as_dash_aux_aux_357_t msg;
    if (can_as_dash_aux_aux_357_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_357");
        return;
    }

    this->createHeader(&this->msgAux357.header);
    this->msgAux357.left = convertRearRPM(msg.hall_rl);
    this->msgAux357.right = convertRearRPM(msg.hall_rr);

    this->pubAux357->publish(this->msgAux357);
}

void CanHandler::publish_aux_384()
{
    can_as_dash_aux_aux_384_t msg;
    if (can_as_dash_aux_aux_384_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_384");
        return;
    }

    this->createHeader(&this->msgAux384.header);
    this->msgAux384.safestate = msg.safe_state;

    this->pubAux357->publish(this->msgAux357);
}

void CanHandler::publish_aux_412()
{
    can_as_dash_aux_aux_412_t msg;
    if (can_as_dash_aux_aux_412_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_412");
        return;
    }

    this->createHeader(&this->msgAux412.header);
    this->msgAux412.leftpumpsignal = msg.left_pump_signal;
    this->msgAux412.rightpumpsignal = msg.right_pump_signal;
    this->msgAux412.accufanspwm = msg.accu_fans_pwm;
    this->msgAux412.leftfanspwm = msg.left_fans_pwm;
    this->msgAux412.rightfanspwm = msg.right_fans_pwm;
    this->msgAux412.accufans = msg.accu_fans;
    this->msgAux412.leftfans = msg.left_fans;
    this->msgAux412.rightfans = msg.right_fans;

    this->pubAux412->publish(this->msgAux412);
}

void CanHandler::publish_dash_101()
{
    can_as_dash_aux_dash_101_t msg;
    if (can_as_dash_aux_dash_101_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_101");
        return;
    }

    this->createHeader(&this->msgDash101.header);
    this->msgDash101.apps = convertAPPS(msg.apps1_raw, msg.apps2_raw);

    this->pubDash101->publish(this->msgDash101);
}

void CanHandler::publish_dash_107()
{
    can_as_dash_aux_dash_107_t msg;
    if (can_as_dash_aux_dash_107_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_107");
        return;
    }

    this->createHeader(&this->msgDash107.header);
    this->msgDash107.left = convertFrontRPM(msg.hall_fl);
    this->msgDash107.right = convertFrontRPM(msg.hall_fr);

    this->pubDash107->publish(this->msgDash107);
}

void CanHandler::publish_dash_198()
{
    can_as_dash_aux_dash_198_t msg;
    if (can_as_dash_aux_dash_198_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_198");
        return;
    }

    this->createHeader(&this->msgDash198.header);
    this->msgDash198.brake = convertBrakePressure(msg.brake_raw);

    this->pubDash198->publish(this->msgDash198);
}

void CanHandler::publish_dash_204()
{
    can_as_dash_aux_dash_204_t msg;
    if (can_as_dash_aux_dash_204_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_204");
        return;
    }

    this->createHeader(&this->msgDash204.header);
    this->msgDash204.fanpwm = msg.fan_pwm;
    this->msgDash204.buzzer = msg.buzzer;
    this->msgDash204.safestate1 = msg.safe_state_1;
    this->msgDash204.enableout = msg.enable_out;
    this->msgDash204.sensorerror = msg.sensor_error;
    this->msgDash204.scsoftware = msg.sc_software;
    this->msgDash204.plactive = msg.pl_active;

    this->pubDash204->publish(this->msgDash204);
}

void CanHandler::publish_dash_279()
{
    can_as_dash_aux_dash_279_t msg;
    if (can_as_dash_aux_dash_279_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_279");
        return;
    }

    this->createHeader(&this->msgDash279.header);
    this->msgDash279.enabletoggle = msg.enable_toggle;
    this->msgDash279.secondtoggle = msg.second_toggle;
    this->msgDash279.thirdtogglenotused = msg.third_toggle_notused;
    this->msgDash279.start = msg.start;
    this->msgDash279.adact = msg.ad_act;
    this->msgDash279.greentsal = msg.green_tsal;
    this->msgDash279.scstate = msg.sc_state;

    this->pubDash279->publish(this->msgDash279);
}

void CanHandler::publish_ebs_550()
{
    can_as_dash_aux_ebs_550_t msg;
    if (can_as_dash_aux_ebs_550_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of EBS_550");
        return;
    }

    this->createHeader(&this->msgEBS550.header);
    this->msgEBS550.asmsstate = msg.asms_state;
    this->msgEBS550.tsmsout = msg.tsms_out;
    this->msgEBS550.ebsengaged = msg.ebs_engaged;
    this->msgEBS550.ebsisarmed = msg.ebs_is_armed;
    this->msgEBS550.ebsled = msg.ebs_led;
    this->msgEBS550.k2state = msg.k2_state;

    this->pubEBS550->publish(this->msgEBS550);
}

void CanHandler::publish_swa_510()
{
    can_as_dash_aux_swa_510_t msg;
    if (can_as_dash_aux_swa_510_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of SWA_510");
        return;
    }

    this->createHeader(&this->msgSWA510.header);
    this->msgSWA510.steering = msg.steering_actual;

    this->pubSWA510->publish(this->msgSWA510);
}

//Functions for CAN transmite
void CanHandler::handleCanTransmite()
{
    this->canTimerCounter++;                //another 1ms passed
    if (this->canTimerCounter == 1001U)     //after 1sec
        this->canTimerCounter = 1U;         //reset counter to prevent overflow

    if (this->rosConf.transmiteAPU660 && !(this->canTimerCounter % CAN_AS_DASH_AUX_APU_660_CYCLE_TIME_MS)) {
        this->transmite_apu_660();
    }
    if (this->rosConf.transmiteSWA539 && !(this->canTimerCounter % CAN_AS_DASH_AUX_SWA_530_CYCLE_TIME_MS)) {
        this->transmite_swa_530();
    }
}

void CanHandler::transmite_apu_660()
{
    this->sendFrame.can_id = CAN_AS_DASH_AUX_APU_660_FRAME_ID;
    this->sendFrame.can_dlc = CAN_AS_DASH_AUX_APU_660_LENGTH;
    if (can_as_dash_aux_apu_660_pack(this->sendFrame.data, &this->frameAPU660, sizeof(sendFrame.data)) != CAN_AS_DASH_AUX_APU_660_LENGTH){
        RCLCPP_ERROR(this->get_logger(), "Error during pack of APU_660");
        return;
    }

    //send(this->can0Socket, &this->sendFrame, sizeof(this->sendFrame), 0);
    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_AS_DASH_AUX_APU_660_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmite of APU_660");
    }
}

void CanHandler::transmite_swa_530()
{
    this->sendFrame.can_id = CAN_AS_DASH_AUX_SWA_530_FRAME_ID;
    this->sendFrame.can_dlc = CAN_AS_DASH_AUX_SWA_530_LENGTH;
    if (can_as_dash_aux_swa_530_pack(this->sendFrame.data, &this->frameSWA530, sizeof(sendFrame.data)) != CAN_AS_DASH_AUX_SWA_530_LENGTH){
        RCLCPP_ERROR(this->get_logger(), "Error during pack of SWA_530");
        return;
    }

    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_AS_DASH_AUX_SWA_530_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmite of SWA_530");
    }
}