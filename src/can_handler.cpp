#include "can_handler.hpp"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/sockios.h>
#include <unistd.h>
#include <chrono>
#include "libsocketcan.h"
#include "raw_conversions.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

CanHandler::CanHandler(rclcpp::NodeOptions nOpt):Node("CanInterface", "", nOpt)
{
    this->loadRosParams();
    this->variablesInit();

    //initialize channel0
    // if ((this->rosConf.channel0 != "vcan0") && (can_do_stop(this->rosConf.channel0.c_str()) != 0)) {
    //     RCLCPP_ERROR(this->get_logger(), "Unable to stop %s.", this->rosConf.channel0.c_str());
    //     return;
    // }
    // if ((this->rosConf.channel0 != "vcan0") && (can_set_bitrate(this->rosConf.channel0.c_str(), this->rosConf.bitrate0) != 0)) {
    //     RCLCPP_ERROR(this->get_logger(), "Unable to set the bitrate on %s.", this->rosConf.channel0.c_str());
    //     return;
    // }
    // if ((this->rosConf.channel0 != "vcan0") && (can_do_start(this->rosConf.channel0.c_str()) != 0)) {
    //     RCLCPP_ERROR(this->get_logger(), "Unable to start %s.", this->rosConf.channel0.c_str());
    //     return;
    // }  
    this->can0Socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->can0Socket < 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to create socket for %s.", this->rosConf.channel0.c_str());
    }
    strcpy(this->ifr0.ifr_name, this->rosConf.channel0.c_str());
    if (ioctl(this->can0Socket, SIOCGIFINDEX, &this->ifr0) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Unable to perform ioctl for %s.", this->rosConf.channel0.c_str());
        return;
    }
    this->addr0.can_family = AF_CAN;
    this->addr0.can_ifindex = this->ifr0.ifr_ifindex;
    if (bind(this->can0Socket, (struct sockaddr*)&this->addr0, sizeof(this->addr0)) < 0){
        RCLCPP_ERROR(this->get_logger(), "Unable to bind socket with %s.", this->rosConf.channel0.c_str());
        return;
    }
    RCLCPP_INFO(this->get_logger(), "%s interface initialized", this->rosConf.channel0.c_str());

    //initialize channel1
    this->can1Socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->can1Socket < 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to create socket for %s.", this->rosConf.channel1.c_str());
    }
    strcpy(this->ifr1.ifr_name, this->rosConf.channel1.c_str());
    if (ioctl(this->can1Socket, SIOCGIFINDEX, &this->ifr1) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Unable to perform ioctl for %s.", this->rosConf.channel1.c_str());
        return;
    }
    this->addr1.can_family = AF_CAN;
    this->addr1.can_ifindex = this->ifr1.ifr_ifindex;
    if (bind(this->can1Socket, (struct sockaddr*)&this->addr1, sizeof(this->addr0)) < 0){
        RCLCPP_ERROR(this->get_logger(), "Unable to bind socket with %s.", this->rosConf.channel1.c_str());
        return;
    }
    RCLCPP_INFO(this->get_logger(), "%s interface initialized", this->rosConf.channel1.c_str());

    //initialize timers
    this->canRecvTimer = this->create_wall_timer(500us, std::bind(&CanHandler::handleCanReceive,    this));
    this->canSendTimer = this->create_wall_timer(1ms,   std::bind(&CanHandler::handleCanTransmit,   this));

    RCLCPP_INFO(this->get_logger(), "Communication started");
}

CanHandler::~CanHandler()
{
    if (close(this->can0Socket) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to close %s.\n", this->rosConf.channel0.c_str());
    }
}

void CanHandler::loadRosParams()
{
    //Channels configuration
    this->get_parameter_or<std::string>("channel0", this->rosConf.channel0, "vcan0");
    this->get_parameter_or<uint32_t>("bitrate0",    this->rosConf.bitrate0, 1000000);
    this->get_parameter_or<std::string>("channel1", this->rosConf.channel1, "vcan1");
    this->get_parameter_or<uint32_t>("bitrate1",    this->rosConf.bitrate1, 500000);
    //CAN messages to publish in ROS
    this->get_parameter_or<bool>("publishDashApps",             this->rosConf.publishDashApps,              true);
    this->get_parameter_or<bool>("publishDashBrake",            this->rosConf.publishDashBrake,             true);
    this->get_parameter_or<bool>("publishDashButtons",          this->rosConf.publishDashButtons,           true);
    this->get_parameter_or<bool>("pubishDashFrontRPM",          this->rosConf.pubishDashFrontRPM,           true);
    this->get_parameter_or<bool>("publishAuxRearRPM",           this->rosConf.publishAuxRearRPM,            true);
    this->get_parameter_or<bool>("publishAuxTsalSafeState",     this->rosConf.publishAuxTsalSafeState,      true);
    this->get_parameter_or<bool>("publishAuxPumpsFans",         this->rosConf.publishAuxPumpsFans,          true);
    this->get_parameter_or<bool>("publishAuxBrakelight",        this->rosConf.publishAuxBrakelight,         true);
    this->get_parameter_or<bool>("publishDashLEDs",             this->rosConf.publishDashLEDs,              true);
    this->get_parameter_or<bool>("publishAuxTankPressure",      this->rosConf.publishAuxTankPressure,       true);
    this->get_parameter_or<bool>("publishAmiSelectedMission",   this->rosConf.publishAmiSelectedMission,    true);
    this->get_parameter_or<bool>("publishSwaActual",            this->rosConf.publishSwaActual,             true);
    this->get_parameter_or<bool>("publishEbsSupervisor",        this->rosConf.publishEbsSupervisor,         true);
    this->get_parameter_or<bool>("publishMotorRPM",             this->rosConf.publishMotorRPM,              true);
    this->get_parameter_or<bool>("publishResStatus",            this->rosConf.publishResStatus,             true);
    //CAN messages to transmit
    this->get_parameter_or<bool>("transmitApuStateMission", this->rosConf.transmitApuStateMission,  true);
    this->get_parameter_or<bool>("transmitEbsServiceBrake", this->rosConf.transmitEbsServiceBrake,  true);
    this->get_parameter_or<bool>("transmitSwaCommanded",    this->rosConf.transmitSwaCommanded,     true);
    this->get_parameter_or<bool>("transmitApuCommand",      this->rosConf.transmitApuCommand,       true);
}

void CanHandler::variablesInit()
{
    rclcpp::SensorDataQoS sensorQos;
    rclcpp::ServicesQoS serviceQos;

    //Initialize publishers
    if (this->rosConf.publishAmiSelectedMission) {
        this->pubAmiSelectedMission = this->create_publisher<turtle_interfaces::msg::Mission>("ami_selected_mission", serviceQos);
        this->msgAmiSelectedMission = turtle_interfaces::msg::Mission();
    }
    if (this->rosConf.publishAuxBrakelight) {
        this->pubAuxBrakelight = this->create_publisher<turtle_interfaces::msg::BrakeLight>("brake_light", sensorQos);
        this->msgAuxBrakelight = turtle_interfaces::msg::BrakeLight();
    }
    if (this->rosConf.publishAuxTankPressure) {
        this->pubAuxTankPressure = this->create_publisher<turtle_interfaces::msg::EbsTankPressure>("ebs_tank_pressure", sensorQos);
        this->msgAuxTankPressure = turtle_interfaces::msg::EbsTankPressure();
    }
    if (this->rosConf.publishAuxRearRPM) {
        this->pubAuxRearRPM = this->create_publisher<turtle_interfaces::msg::RPM>("rpm_rear", sensorQos);
        this->msgAuxRearRPM = turtle_interfaces::msg::RPM();
    }
    if (this->rosConf.publishMotorRPM) {
        this->pubMotorRPM = this->create_publisher<turtle_interfaces::msg::RPM>("rpm_motor", 10);
        this->msgMotorRPM = turtle_interfaces::msg::RPM();
    }
    if (this->rosConf.publishAuxTsalSafeState) {
        this->pubAuxTsalSafeState = this->create_publisher<turtle_interfaces::msg::TsalSafeState>("tsal_safe_state", sensorQos);
        this->msgAuxTsalSafeState = turtle_interfaces::msg::TsalSafeState();
    }
    if (this->rosConf.publishAuxPumpsFans) {
        this->pubAuxPumpsFans = this->create_publisher<turtle_interfaces::msg::CoolingInfo>("cooling_info", sensorQos);
        this->msgAuxPumpsFans = turtle_interfaces::msg::CoolingInfo();
    }
    if (this->rosConf.publishDashApps) {
        this->pubDashApps = this->create_publisher<turtle_interfaces::msg::Apps>("apps", sensorQos);
        this->msgDashApps = turtle_interfaces::msg::Apps();
    }
    if (this->rosConf.pubishDashFrontRPM) {
        this->pubDashFrontRPM = this->create_publisher<turtle_interfaces::msg::RPM>("rpm_front", sensorQos);
        this->msgDashFrontRPM = turtle_interfaces::msg::RPM();
    }
    if (this->rosConf.publishDashBrake) {
        this->pubDashBrake = this->create_publisher<turtle_interfaces::msg::Brake>("brake", sensorQos);
        this->msgDashBrake = turtle_interfaces::msg::Brake();
    }
    if (this->rosConf.publishDashLEDs) {
        this->pubDashLEDs = this->create_publisher<turtle_interfaces::msg::DashLeds>("dash_leds", serviceQos);
        this->msgDashLEDs = turtle_interfaces::msg::DashLeds();
    }
    if (this->rosConf.publishDashButtons) {
        this->pubDashButtons = this->create_publisher<turtle_interfaces::msg::DashButtons>("dash_buttons", serviceQos);
        this->msgDashButtons = turtle_interfaces::msg::DashButtons();
    }
    if (this->rosConf.publishEbsSupervisor) {
        this->pubEbsSupervisor= this->create_publisher<turtle_interfaces::msg::EbsSupervisorInfo>("ebs_supervisor_info", serviceQos);
        this->msgEbsSupervisor = turtle_interfaces::msg::EbsSupervisorInfo();
    } 
    if (this->rosConf.publishSwaActual) {
        this->pubSwaActual = this->create_publisher<turtle_interfaces::msg::Steering>("steering_actual", sensorQos);
        this->msgSwaActual = turtle_interfaces::msg::Steering();
    }
    if (this->rosConf.publishInvererCommands) {
        this->pubInvCmds = this->create_publisher<turtle_interfaces::msg::InverterCommands>("inverter_commands", 10);
        this->msgInvCmds = turtle_interfaces::msg::InverterCommands();
    }
    if (this->rosConf.publishResStatus) {
        this->pubResStatus = this->create_publisher<turtle_interfaces::msg::ResStatus>("res_status", serviceQos);
        this->msgResStatus = turtle_interfaces::msg::ResStatus();
    }

    //Initialize CAN Tx messages
    if (this->rosConf.transmitApuStateMission) {
        this->subApuState = this->create_subscription<turtle_interfaces::msg::StateMachineState>("state_machine_state", serviceQos, std::bind(&CanHandler::apu_state_callback, this, _1));
        this->subApuMission = this->create_subscription<turtle_interfaces::msg::Mission>("current_mission", serviceQos, std::bind(&CanHandler::apu_mission_callback, this, _1));

        this->frameApuStateMission.as_mission = CAN_AS_DASH_AUX_APU_STATE_MISSION_AS_MISSION_NO_MISSION_CHOICE;
        this->frameApuStateMission.as_state = CAN_AS_DASH_AUX_APU_STATE_MISSION_AS_STATE_AS_OFF_CHOICE;
    }
    if (this->rosConf.transmitEbsServiceBrake) {
        this->frameEbsServiceBrake.servo_commanded_percentage = 0;
    }
    if (this->rosConf.transmitSwaCommanded || this->rosConf.transmitApuCommand) {
        this->subActuatorCmd = this->create_subscription<turtle_interfaces::msg::ActuatorCmd>("cmd", 10, std::bind(&CanHandler::actuator_cmd_callback, this, _1));
    }
    if (this->rosConf.transmitSwaCommanded) {
        this->frameSwaCommanded.steering_angle_commanded = convertSteeringAngleTarget(0.0);
        this->frameSwaCommanded.steering_rate_commanded = convertSteeringRateTarget(0.01);
        this->frameSwaCommanded.steering_mode = CAN_AS_DASH_AUX_SWA_COMMANDED_STEERING_MODE_STEERING_ANGLE_CONTROL_CHOICE;
        this->frameSwaCommanded.steering_rate_direction = 0.01 > 0.0    ? CAN_AS_DASH_AUX_SWA_COMMANDED_STEERING_RATE_DIRECTION_COUNTER_CLOCKWISE_CHOICE
                                                                        : CAN_AS_DASH_AUX_SWA_COMMANDED_STEERING_RATE_DIRECTION_CLOCKWISE_CHOICE;
        this->frameSwaCommanded.steering_rate_is_zero = CAN_AS_DASH_AUX_SWA_COMMANDED_STEERING_RATE_IS_ZERO_TRUE_CHOICE;
    }
    if (this->rosConf.transmitApuCommand) {
        this->frameApuCommand.throttle_brake_commanded = 0.0;
    }
}

//Functions for CAN receive
void CanHandler::handleCanReceive()
{
    while (recvfrom(this->can0Socket, &this->recvFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, &this->len) >= 8) {
        ioctl(this->can0Socket, SIOCGSTAMP, &this->recvTime);  //get message timestamp

        if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AMI_SELECTED_MISSION_FRAME_ID && this->rosConf.publishAmiSelectedMission) {
            this->publish_ami_selected_mission();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AUX_BRAKELIGHT_FRAME_ID && this->rosConf.publishAuxBrakelight) {
            this->publish_aux_brakelight();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AUX_TANK_PRESSURE_FRAME_ID && this->rosConf.publishAuxTankPressure) {
            this->publish_aux_tank_pressure();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AUX_REAR_HALL_FRAME_ID && this->rosConf.publishAuxRearRPM) {
            this->publish_aux_rear_rpm();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AUX_TSAL_SAFE_STATE_FRAME_ID && this->rosConf.publishAuxTsalSafeState) {
            this->publish_aux_tsal_safe_state();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AUX_PUMPS_FANS_FRAME_ID && this->rosConf.publishAuxPumpsFans) {
            this->publish_aux_pumps_fans();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_DASH_APPS_FRAME_ID && this->rosConf.publishDashApps) {
            this->publish_dash_apps();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_DASH_FRONT_HALL_FRAME_ID && this->rosConf.pubishDashFrontRPM) {
            this->publish_dash_front_rpm();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_DASH_BRAKE_FRAME_ID && this->rosConf.publishDashBrake) {
            this->publish_dash_brake();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_DASH_LEDS_FRAME_ID && this->rosConf.publishDashLEDs) {
            this->publish_dash_leds();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_DASH_BUTTONS_FRAME_ID && this->rosConf.publishDashButtons) {
            this->publish_dash_buttons();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_EBS_SUPERVISOR_FRAME_ID && this->rosConf.publishEbsSupervisor) {
            this->publish_ebs_supervisor();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_SWA_STATUS_FRAME_ID && this->rosConf.publishSwaActual) {
            this->publish_swa_actual();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_INV_RESOLVERS_FRAME_ID) {
            if (this->rosConf.publishMotorRPM)
                this->publish_motor_rpm();
            if (this->rosConf.publishInvererCommands)
                this->publish_inverter_commands();
        }
    }

    while (recvfrom(this->can1Socket, &this->recvFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr1, &this->len) >= 8) {
        ioctl(this->can1Socket, SIOCGSTAMP, &this->recvTime);  //get message timestamp
        if (this->recvFrame.can_id == CAN_APU_RES_DLOGGER_RES_STATUS_FRAME_ID && this->rosConf.publishResStatus) {
            this->publish_res_status();
        }
    }
}

void CanHandler::createHeader(std_msgs::msg::Header *header)
{
    header->frame_id = "";
    header->stamp.sec = this->recvTime.tv_sec;
    header->stamp.nanosec = this->recvTime.tv_usec*1000;
}

void CanHandler::publish_ami_selected_mission()
{
    can_as_dash_aux_ami_selected_mission_t msg;
    if (can_as_dash_aux_ami_selected_mission_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AMI_SELECTED_MISSION");
        return;
    }

    this->createHeader(&this->msgAmiSelectedMission.header);
    this->msgAmiSelectedMission.mission = msg.ami_mission;

    this->pubAmiSelectedMission->publish(this->msgAmiSelectedMission);
}

void CanHandler::publish_aux_brakelight()
{
    can_as_dash_aux_aux_brakelight_t msg;
    if (can_as_dash_aux_aux_brakelight_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_BRAKELIGHT");
        return;
    }

    this->createHeader(&this->msgAuxBrakelight.header);
    this->msgAuxBrakelight.brakelight = (msg.brakelight == CAN_AS_DASH_AUX_AUX_BRAKELIGHT_BRAKELIGHT_ON_CHOICE);

    this->pubAuxBrakelight->publish(this->msgAuxBrakelight);
}

void CanHandler::publish_aux_tank_pressure()
{
    can_as_dash_aux_aux_tank_pressure_t msg;
    if (can_as_dash_aux_aux_tank_pressure_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_TANK_PRESSURE");
        return;
    }

    this->createHeader(&this->msgAuxTankPressure.header);
    this->msgAuxTankPressure.ebspressureraw = convertEbsPressure(msg.ebs_pressure_raw);

    this->pubAuxTankPressure->publish(this->msgAuxTankPressure);
}

void CanHandler::publish_aux_rear_rpm()
{
    can_as_dash_aux_aux_rear_hall_t msg;
    if (can_as_dash_aux_aux_rear_hall_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_REAR_HALL");
        return;
    }

    this->createHeader(&this->msgAuxRearRPM.header);
    this->msgAuxRearRPM.left = convertRearRPM(msg.hall_rl);
    this->msgAuxRearRPM.right = convertRearRPM(msg.hall_rr);

    this->pubAuxRearRPM->publish(this->msgAuxRearRPM);
}

void CanHandler::publish_aux_tsal_safe_state()
{
    can_as_dash_aux_aux_tsal_safe_state_t msg;
    if (can_as_dash_aux_aux_tsal_safe_state_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_TSAL_SAFE_STATE");
        return;
    }

    this->createHeader(&this->msgAuxTsalSafeState.header);
    this->msgAuxTsalSafeState.safestate = (msg.safe_state == CAN_AS_DASH_AUX_AUX_TSAL_SAFE_STATE_SAFE_STATE_ON_CHOICE);

    this->pubAuxTsalSafeState->publish(this->msgAuxTsalSafeState);
}

void CanHandler::publish_aux_pumps_fans()
{
    can_as_dash_aux_aux_pumps_fans_t msg;
    if (can_as_dash_aux_aux_pumps_fans_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_PUMPS_FANS");
        return;
    }

    this->createHeader(&this->msgAuxPumpsFans.header);
    this->msgAuxPumpsFans.leftpumpsignal = msg.left_pump_signal;
    this->msgAuxPumpsFans.rightpumpsignal = msg.right_pump_signal;
    this->msgAuxPumpsFans.accufanspwm = msg.accu_fans_pwm;
    this->msgAuxPumpsFans.leftfanspwm = msg.left_fans_pwm;
    this->msgAuxPumpsFans.rightfanspwm = msg.right_fans_pwm;
    this->msgAuxPumpsFans.accufans = (msg.accu_fans == 13);//CAN_AS_DASH_AUX_AUX_PUMPS_FANS_ACCU_FANS_ON_CHOICE);
    this->msgAuxPumpsFans.leftfans = (msg.left_fans == 13);//CAN_AS_DASH_AUX_AUX_PUMPS_FANS_LEFT_FANS_ON_CHOICE);
    this->msgAuxPumpsFans.rightfans = (msg.right_fans == 13);//CAN_AS_DASH_AUX_AUX_PUMPS_FANS_RIGHT_FANS_ON_CHOICE);

    this->pubAuxPumpsFans->publish(this->msgAuxPumpsFans);
}

void CanHandler::publish_dash_apps()
{
    can_as_dash_aux_dash_apps_t msg;
    if (can_as_dash_aux_dash_apps_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_APPS");
        return;
    }

    this->createHeader(&this->msgDashApps.header);
    this->msgDashApps.apps = convertAPPS(msg.apps1_raw, msg.apps2_raw);

    this->pubDashApps->publish(this->msgDashApps);
}

void CanHandler::publish_dash_front_rpm()
{
    can_as_dash_aux_dash_front_hall_t msg;
    if (can_as_dash_aux_dash_front_hall_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_FRONT_HALL");
        return;
    }

    this->createHeader(&this->msgDashFrontRPM.header);
    this->msgDashFrontRPM.left = convertFrontRPM(msg.hall_fl);
    this->msgDashFrontRPM.right = convertFrontRPM(msg.hall_fr);

    this->pubDashFrontRPM->publish(this->msgDashFrontRPM);
}

void CanHandler::publish_dash_brake()
{
    can_as_dash_aux_dash_brake_t msg;
    if (can_as_dash_aux_dash_brake_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_BRAKE");
        return;
    }

    this->createHeader(&this->msgDashBrake.header);
    this->msgDashBrake.brake = convertBrakePressure(msg.brake_raw);

    this->pubDashBrake->publish(this->msgDashBrake);
}

void CanHandler::publish_dash_leds()
{
    can_as_dash_aux_dash_leds_t msg;
    if (can_as_dash_aux_dash_leds_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_LEDS");
        return;
    }

    this->createHeader(&this->msgDashLEDs.header);
    this->msgDashLEDs.fanpwm = msg.fan_pwm;
    this->msgDashLEDs.buzzer = (msg.buzzer == 13);
    this->msgDashLEDs.safestate1 = (msg.safe_state_1 == 13);
    this->msgDashLEDs.enableout = (msg.enable_out == 13);
    this->msgDashLEDs.sensorerror = (msg.sensor_error == 13);
    this->msgDashLEDs.scsoftware = (msg.sc_software == 1);
    this->msgDashLEDs.plactive = (msg.pl_active == 1);

    this->pubDashLEDs->publish(this->msgDashLEDs);
}

void CanHandler::publish_dash_buttons()
{
    can_as_dash_aux_dash_buttons_t msg;
    if (can_as_dash_aux_dash_buttons_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_BUTTONS");
        return;
    }

    this->createHeader(&this->msgDashButtons.header);
    this->msgDashButtons.enabletoggle = (bool)msg.enable_toggle;
    this->msgDashButtons.secondtoggle = (bool)msg.second_toggle;
    this->msgDashButtons.thirdtogglenotused = (bool)msg.third_toggle_notused;
    this->msgDashButtons.start = (bool)msg.start;
    this->msgDashButtons.adact = (bool)msg.ad_act;
    this->msgDashButtons.greentsal = (bool)msg.green_tsal;
    this->msgDashButtons.scstate = (bool)msg.sc_state;

    this->pubDashButtons->publish(this->msgDashButtons);
}

void CanHandler::publish_ebs_supervisor()
{
    can_as_dash_aux_ebs_supervisor_t msg;
    if (can_as_dash_aux_ebs_supervisor_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of EBS_SUPERVISOR");
        return;
    }

    this->createHeader(&this->msgEbsSupervisor.header);
    this->msgEbsSupervisor.asmsstate = (msg.asms_state == CAN_AS_DASH_AUX_EBS_SUPERVISOR_ASMS_STATE_ON_CHOICE);
    this->msgEbsSupervisor.tsmsout = (msg.tsms_out == CAN_AS_DASH_AUX_EBS_SUPERVISOR_TSMS_OUT_SDC_ON_CHOICE);
    this->msgEbsSupervisor.k2state = (msg.k2_state == CAN_AS_DASH_AUX_EBS_SUPERVISOR_K2_STATE_RES_GO_PRESSED_CHOICE);
    this->msgEbsSupervisor.ebsisarmed = (msg.ebs_armed == CAN_AS_DASH_AUX_EBS_SUPERVISOR_EBS_ARMED_YES_CHOICE);
    this->msgEbsSupervisor.ebsengaged = (msg.ebs_engaged == CAN_AS_DASH_AUX_EBS_SUPERVISOR_EBS_ENGAGED_YES_CHOICE);
    this->msgEbsSupervisor.ebsled = (msg.ebs_led == CAN_AS_DASH_AUX_EBS_SUPERVISOR_EBS_LED_ON_CHOICE);
    this->msgEbsSupervisor.initialchecked = (msg.initial_checked == CAN_AS_DASH_AUX_EBS_SUPERVISOR_INITIAL_CHECKED_YES_CHOICE);
    this->msgEbsSupervisor.servicebrakestatus = msg.service_brake_status;
    

    this->pubEbsSupervisor->publish(this->msgEbsSupervisor);
}

void CanHandler::publish_swa_actual()
{
    can_as_dash_aux_swa_status_t msg;
    if (can_as_dash_aux_swa_status_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of SWA_STATUS");
        return;
    }

    this->createHeader(&this->msgSwaActual.header);
    this->msgSwaActual.steering = convertSteeringActual(msg.steering_actual);

    if (msg.analog1_error || msg.analog2_error || msg.stall_occured) {
        RCLCPP_WARN(this->get_logger(), "AN1 %u | AN2 %u | Stall %u", msg.analog1_error, msg.analog2_error, msg.stall_occured);
    }

    this->pubSwaActual->publish(this->msgSwaActual);
}

void CanHandler::publish_motor_rpm()
{
    can_as_dash_aux_inv_resolvers_t msg;
    if (can_as_dash_aux_inv_resolvers_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of INV_RESOLVERS(motor_rpm)");
        return;
    }

    this->createHeader(&this->msgMotorRPM.header);
    this->msgMotorRPM.left = (float)msg.motor_rpm_left;
    this->msgMotorRPM.right = (float)msg.motor_rpm_right;

    this->pubMotorRPM->publish(this->msgMotorRPM);
}

void CanHandler::publish_inverter_commands()
{
    can_as_dash_aux_inv_resolvers_t msg;
    if (can_as_dash_aux_inv_resolvers_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of INV_RESOLVERS(inverter_commands)");
        return;
    }

    this->createHeader(&this->msgInvCmds.header);
    this->msgInvCmds.torqueleft = msg.commanded_torque_left;
    this->msgInvCmds.torqueright = msg.commanded_torque_right;

    this->pubInvCmds->publish(this->msgInvCmds);
}

void CanHandler::publish_res_status()
{
    can_apu_res_dlogger_res_status_t msg;
    if (can_apu_res_dlogger_res_status_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of RES_STATUS");
        return;
    }

    this->createHeader(&this->msgResStatus.header);
    this->msgResStatus.stop = (msg.stop == CAN_APU_RES_DLOGGER_RES_STATUS_STOP_ON_CHOICE);
    this->msgResStatus.toggle = (msg.toggle == CAN_APU_RES_DLOGGER_RES_STATUS_TOGGLE_ON_CHOICE);
    this->msgResStatus.button = (msg.button == CAN_APU_RES_DLOGGER_RES_STATUS_BUTTON_ON_CHOICE);
    this->msgResStatus.signal_strength = msg.signal_strength;

    this->pubResStatus->publish(this->msgResStatus);
}

//Functions for CAN transmit
void CanHandler::apu_state_callback(turtle_interfaces::msg::StateMachineState::SharedPtr msgApuState)
{
    this->frameApuStateMission.as_state = msgApuState->state;
}

void CanHandler::apu_mission_callback(turtle_interfaces::msg::Mission::SharedPtr msgApuMission)
{
    this->frameApuStateMission.as_mission = msgApuMission->mission;
}

void CanHandler::actuator_cmd_callback(turtle_interfaces::msg::ActuatorCmd::SharedPtr msgActuatorCmd)
{
    this->frameSwaCommanded.steering_angle_commanded = convertSteeringAngleTarget(msgActuatorCmd->steering);
    this->frameSwaCommanded.steering_rate_commanded = convertSteeringRateTarget(msgActuatorCmd->steering);
    this->frameSwaCommanded.steering_mode = msgActuatorCmd->steering_mode;
    this->frameSwaCommanded.steering_rate_direction = msgActuatorCmd->steering > 0.0 ? 
                                                    CAN_AS_DASH_AUX_SWA_COMMANDED_STEERING_RATE_DIRECTION_COUNTER_CLOCKWISE_CHOICE :
                                                    CAN_AS_DASH_AUX_SWA_COMMANDED_STEERING_RATE_DIRECTION_CLOCKWISE_CHOICE;
    this->frameSwaCommanded.steering_rate_is_zero = (msgActuatorCmd->steering < 0.01) && (msgActuatorCmd->steering > -0.01) ? 
                                                    CAN_AS_DASH_AUX_SWA_COMMANDED_STEERING_RATE_IS_ZERO_TRUE_CHOICE :
                                                    CAN_AS_DASH_AUX_SWA_COMMANDED_STEERING_RATE_IS_ZERO_FALSE_CHOICE;

    this->frameApuCommand.throttle_brake_commanded = msgActuatorCmd->throttle;
}

void CanHandler::handleCanTransmit()
{
    this->canTimerCounter++;                //another 1ms passed
    if (this->canTimerCounter == 1001U)     //after 1sec
        this->canTimerCounter = 1U;         //reset counter to prevent overflow

    if (this->rosConf.transmitApuStateMission && !(this->canTimerCounter % CAN_AS_DASH_AUX_APU_STATE_MISSION_CYCLE_TIME_MS)) {
        this->transmit_apu_state_mission();
    }
    if (this->rosConf.transmitEbsServiceBrake && !(this->canTimerCounter % CAN_AS_DASH_AUX_EBS_SERVICE_BRAKE_CYCLE_TIME_MS)) {
        this->transmit_ebs_service_brake();
    }
    if (this->rosConf.transmitSwaCommanded && !(this->canTimerCounter % CAN_AS_DASH_AUX_SWA_COMMANDED_CYCLE_TIME_MS)) {
        this->transmit_swa_commanded();
    }
    if (this->rosConf.transmitApuCommand && !(this->canTimerCounter % CAN_AS_DASH_AUX_SWA_COMMANDED_CYCLE_TIME_MS)) {
        this->transmit_apu_command();
    }
}

void CanHandler::transmit_apu_state_mission()
{
    this->sendFrame.can_id = CAN_AS_DASH_AUX_APU_STATE_MISSION_FRAME_ID;
    this->sendFrame.can_dlc = CAN_AS_DASH_AUX_APU_STATE_MISSION_LENGTH;
    if (can_as_dash_aux_apu_state_mission_pack(this->sendFrame.data, &this->frameApuStateMission, sizeof(sendFrame.data)) != CAN_AS_DASH_AUX_APU_STATE_MISSION_LENGTH){
        RCLCPP_ERROR(this->get_logger(), "Error during pack of APU_STATE_MISSION");
        return;
    }

    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_AS_DASH_AUX_APU_STATE_MISSION_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of APU_STATE_MISSION");
    }
}

void CanHandler::transmit_ebs_service_brake()
{
    this->sendFrame.can_id = CAN_AS_DASH_AUX_EBS_SERVICE_BRAKE_FRAME_ID;
    this->sendFrame.can_dlc = CAN_AS_DASH_AUX_EBS_SERVICE_BRAKE_LENGTH;
    if (can_as_dash_aux_ebs_service_brake_pack(this->sendFrame.data, &this->frameEbsServiceBrake, sizeof(sendFrame.data)) != CAN_AS_DASH_AUX_EBS_SERVICE_BRAKE_LENGTH){
        RCLCPP_ERROR(this->get_logger(), "Error during pack of EBS_SERVICE_BRAKE");
        return;
    }

    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_AS_DASH_AUX_EBS_SERVICE_BRAKE_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of EBS_SERVICE_BRAKE");
    }
}

void CanHandler::transmit_swa_commanded()
{
    this->sendFrame.can_id = CAN_AS_DASH_AUX_SWA_COMMANDED_FRAME_ID;
    this->sendFrame.can_dlc = CAN_AS_DASH_AUX_SWA_COMMANDED_LENGTH;
    if (can_as_dash_aux_swa_commanded_pack(this->sendFrame.data, &this->frameSwaCommanded, sizeof(sendFrame.data)) != CAN_AS_DASH_AUX_SWA_COMMANDED_LENGTH){
        RCLCPP_ERROR(this->get_logger(), "Error during pack of SWA_COMMANDED");
        return;
    }

    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_AS_DASH_AUX_SWA_COMMANDED_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of SWA_COMMANDED");
    }
}

void CanHandler::transmit_apu_command()
{
    this->sendFrame.can_id = CAN_AS_DASH_AUX_APU_COMMAND_FRAME_ID;
    this->sendFrame.can_dlc = CAN_AS_DASH_AUX_APU_COMMAND_LENGTH;
    if (can_as_dash_aux_apu_command_pack(this->sendFrame.data, &this->frameApuCommand, sizeof(sendFrame.data)) != CAN_AS_DASH_AUX_APU_COMMAND_LENGTH){
        RCLCPP_ERROR(this->get_logger(), "Error during pack of APU_COMMAND");
        return;
    }

    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_AS_DASH_AUX_APU_COMMAND_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of APU_COMMAND");
    }
}
