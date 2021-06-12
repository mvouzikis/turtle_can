#include "can_handler.hpp"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/sockios.h>
#include <linux/if_link.h>
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
    this->canRecvTimer =    this->create_wall_timer(500us,  std::bind(&CanHandler::handleCanReceive,        this));
    this->canRecvTimeout =  this->create_wall_timer(10ms,   std::bind(&CanHandler::handleReceiveTimeout,    this));
    this->canSendTimer =    this->create_wall_timer(1ms,    std::bind(&CanHandler::handleCanTransmit,       this));

    res_initialized = false;

    RCLCPP_INFO(this->get_logger(), "Communication started");
}

CanHandler::~CanHandler()
{
    if (close(this->can0Socket) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to close %s.\n", this->rosConf.channel0.c_str());
    }
    if (close(this->can1Socket) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to close %s.\n", this->rosConf.channel1.c_str());
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
    this->get_parameter_or<bool>("publishEbsTankPressure",      this->rosConf.publishEbsTankPressure,       true);
    this->get_parameter_or<bool>("publishAmiSelectedMission",   this->rosConf.publishAmiSelectedMission,    true);
    this->get_parameter_or<bool>("publishSwaActual",            this->rosConf.publishSwaStatus,             true);
    this->get_parameter_or<bool>("publishEbsServiceBrake",      this->rosConf.publishEbsServiceBrake,       true);
    this->get_parameter_or<bool>("publishEbsSupervisor",        this->rosConf.publishEbsSupervisor,         true);
    this->get_parameter_or<bool>("publishMotorRPM",             this->rosConf.publishMotorRPM,              true);
    this->get_parameter_or<bool>("publishResStatus",            this->rosConf.publishResStatus,             true);
    this->get_parameter_or<bool>("publishCanStatus",            this->rosConf.publishCanStatus,             true);
    this->get_parameter_or<bool>("publishECUParamsActaul",      this->rosConf.publishECUParamsActaul,       true);
    //CAN messages to transmit
    this->get_parameter_or<uint8_t>("transmitApuStateMission", this->rosConf.transmitApuStateMission,   1);
    this->get_parameter_or<uint8_t>("transmitSwaCommanded",    this->rosConf.transmitSwaCommanded,      1);
    this->get_parameter_or<uint8_t>("transmitApuCommand",      this->rosConf.transmitApuCommand,        1);
    this->get_parameter_or<uint8_t>("transmitECUParams",       this->rosConf.transmitECUParams,         1);
    this->get_parameter_or<bool>("transmitDvSystemStatus",     this->rosConf.transmitDvSystemStatus,    true);
    this->get_parameter_or<bool>("transmitApuResInit",         this->rosConf.transmitApuResInit,        true);
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
    if (this->rosConf.publishEbsTankPressure) {
        this->pubEbsTankPressure = this->create_publisher<turtle_interfaces::msg::EbsTankPressure>("ebs_tank_pressure", sensorQos);
        this->msgEbsTankPressure = turtle_interfaces::msg::EbsTankPressure();
    }
    if (this->rosConf.publishAuxRearRPM) {
        this->pubAuxRearRPM = this->create_publisher<turtle_interfaces::msg::RPM>("rpm_rear", sensorQos);
        this->msgAuxRearRPM = turtle_interfaces::msg::RPM();
    }
    if (this->rosConf.publishMotorRPM) {
        this->pubMotorRPM = this->create_publisher<turtle_interfaces::msg::RPM>("rpm_motor", sensorQos);
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
    if (this->rosConf.publishEbsServiceBrake) {
        this->pubEbsServiceBrake= this->create_publisher<turtle_interfaces::msg::EbsServiceBrake>("ebs_service_brake", serviceQos);
        this->msgEbsServiceBrake = turtle_interfaces::msg::EbsServiceBrake();
    }
    if (this->rosConf.publishEbsSupervisor) {
        this->pubEbsSupervisor= this->create_publisher<turtle_interfaces::msg::EbsSupervisorInfo>("ebs_supervisor_info", serviceQos);
        this->msgEbsSupervisor = turtle_interfaces::msg::EbsSupervisorInfo();
    } 
    if (this->rosConf.publishSwaStatus) {
        this->pubSwaActual = this->create_publisher<turtle_interfaces::msg::Steering>("steering_actual", sensorQos);
        this->msgSwaActual = turtle_interfaces::msg::Steering();
    }
    if (this->rosConf.publishInvererCommands) {
        this->pubInvCmds = this->create_publisher<turtle_interfaces::msg::InverterCommands>("inverter_commands", sensorQos);
        this->msgInvCmds = turtle_interfaces::msg::InverterCommands();
    }
    if (this->rosConf.publishResStatus) {
        this->pubResStatus = this->create_publisher<turtle_interfaces::msg::ResStatus>("res_status", serviceQos);
        this->msgResStatus = turtle_interfaces::msg::ResStatus();
    }
    if (this->rosConf.publishCanStatus) {
        this->pubCanStatus = this->create_publisher<turtle_interfaces::msg::CanStatus>("can_status", serviceQos);
        this->msgCanStatus = turtle_interfaces::msg::CanStatus();
    }
    if (this->rosConf.publishECUParamsActaul) {
        this->pubEcuParams = this->create_publisher<turtle_interfaces::msg::ECUParams>("ecu_params_actual", serviceQos);
        this->msgEcuParams = turtle_interfaces::msg::ECUParams();
    }

    //Initialize CAN Tx messages
    if (this->rosConf.transmitApuStateMission || this->rosConf.transmitDvSystemStatus) {
        this->subApuState = this->create_subscription<turtle_interfaces::msg::StateMachineState>("state_machine_state", serviceQos, std::bind(&CanHandler::apu_state_callback, this, _1));
        this->subApuMission = this->create_subscription<turtle_interfaces::msg::Mission>("current_mission", serviceQos, std::bind(&CanHandler::apu_mission_callback, this, _1));

        this->frameApuStateMission.as_mission = CAN_AS_DASH_AUX_APU_STATE_MISSION_AS_MISSION_NO_MISSION_CHOICE;
        this->frameApuStateMission.as_state = CAN_AS_DASH_AUX_APU_STATE_MISSION_AS_STATE_AS_OFF_CHOICE;
    }
    if (this->rosConf.transmitSwaCommanded || this->rosConf.transmitApuCommand) {
        this->subActuatorCmd = this->create_subscription<turtle_interfaces::msg::ActuatorCmd>("cmd", sensorQos, std::bind(&CanHandler::actuator_cmd_callback, this, _1));
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
    if (this->rosConf.transmitECUParams) {
        this->subECUParams = this->create_subscription<turtle_interfaces::msg::ECUParams>("ecu_params", serviceQos, std::bind(&CanHandler::ecu_params_callback, this, _1));

        this->frameECUParams.inverter_rpm_max = 0;
        this->frameECUParams.inverter_i_max = 0;
        this->frameECUParams.power_target_kw = can_as_dash_aux_ecu_parameters_power_target_kw_encode(0.0);
    }
    if (this->rosConf.transmitDvSystemStatus) {
        this->subControlInfo = this->create_subscription<turtle_interfaces::msg::ControlInfo>("control_info", serviceQos, std::bind(&CanHandler::control_info_callback, this, _1));
        this->subSlamInfo = this->create_subscription<turtle_interfaces::msg::SlamInfo>("slam_info", serviceQos, std::bind(&CanHandler::slam_info_callback, this, _1));

        this->frameDvSystemStatus.assi_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_ASSI_STATE_OFF_CHOICE;
        this->frameDvSystemStatus.ebs_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_EBS_STATE_UNANAILABLE_CHOICE;
        this->frameDvSystemStatus.ami_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_AMI_STATE_ACCELERATION_CHOICE;
        this->frameDvSystemStatus.steering_state = 0;
        this->frameDvSystemStatus.service_brake_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_SERVICE_BRAKE_STATE_DISENGAGED_CHOICE;
        this->frameDvSystemStatus.lap_counter = 0;
        this->frameDvSystemStatus.cones_count_actual = 0;
        this->frameDvSystemStatus.cones_count_all = 0;
    }
    if (this->rosConf.transmitApuResInit) {
        this->frameApuResInit.requested_state = CAN_APU_RES_DLOGGER_APU_RES_INIT_REQUESTED_STATE_OPERATIONAL_CHOICE;
        this->frameApuResInit.addressed_node = CAN_APU_RES_DLOGGER_APU_RES_INIT_ADDRESSED_NODE_RES_ADDRESS_CHOICE;
    }
}

//Functions for CAN receive
void CanHandler::handleCanReceive()
{
    //For channel 0
    while (recvfrom(this->can0Socket, &this->recvFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, &this->len) >= 8) {
        ioctl(this->can0Socket, SIOCGSTAMP, &this->recvTime);  //get message timestamp

        if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AMI_SELECTED_MISSION_FRAME_ID && this->rosConf.publishAmiSelectedMission) {
            this->publish_ami_selected_mission();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AUX_BRAKELIGHT_FRAME_ID && this->rosConf.publishAuxBrakelight) {
            this->publish_aux_brakelight();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_EBS_TANK_PRESSURE_FRAME_ID && this->rosConf.publishEbsTankPressure) {
            this->publish_ebs_tank_pressure();
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
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_EBS_SERVICE_BRAKE_FRAME_ID && this->rosConf.publishEbsServiceBrake) {
            this->publish_ebs_service_brake();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_EBS_SUPERVISOR_FRAME_ID && this->rosConf.publishEbsSupervisor) {
            this->publish_ebs_supervisor();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_SWA_STATUS_FRAME_ID && this->rosConf.publishSwaStatus) {
            this->publish_swa_actual();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_INV_RESOLVERS_FRAME_ID) {
            if (this->rosConf.publishMotorRPM)
                this->publish_motor_rpm();
            if (this->rosConf.publishInvererCommands)
                this->publish_inverter_commands();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_ECU_PARAMS_ACTUAL_FRAME_ID && this->rosConf.publishECUParamsActaul) {
            this->publish_ecu_params_actual();
        }
    }

    //For channel1
    while (recvfrom(this->can1Socket, &this->recvFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr1, &this->len) >= 8) {
        ioctl(this->can1Socket, SIOCGSTAMP, &this->recvTime);  //get message timestamp
        if (this->recvFrame.can_id == CAN_APU_RES_DLOGGER_RES_STATUS_FRAME_ID && this->rosConf.publishResStatus) {
            res_initialized = true;
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

void CanHandler::publish_ebs_tank_pressure()
{
    can_as_dash_aux_ebs_tank_pressure_t msg;
    if (can_as_dash_aux_ebs_tank_pressure_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of EBS_TANK_PRESSURE");
        return;
    }

    this->createHeader(&this->msgEbsTankPressure.header);
    this->msgEbsTankPressure.ebspressureraw = convertEbsPressure(msg.ebs_pressure_raw);

    this->pubEbsTankPressure->publish(this->msgEbsTankPressure);
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
    this->msgDashFrontRPM.left = msg.rpm_fl;
    this->msgDashFrontRPM.right = msg.rpm_fr;

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
    this->msgDashLEDs.buzzer = (msg.buzzer == CAN_AS_DASH_AUX_DASH_LEDS_BUZZER_ON_CHOICE);
    this->msgDashLEDs.safestate1 = (msg.safe_state_1 == CAN_AS_DASH_AUX_DASH_LEDS_SAFE_STATE_1_ON_CHOICE);
    this->msgDashLEDs.enableout = (msg.enable_out == CAN_AS_DASH_AUX_DASH_LEDS_ENABLE_OUT_ON_CHOICE);
    this->msgDashLEDs.ebsled = (msg.ebs_led == CAN_AS_DASH_AUX_DASH_LEDS_EBS_LED_ON_CHOICE);
    this->msgDashLEDs.scsoftware = (msg.sc_software == CAN_AS_DASH_AUX_DASH_LEDS_SC_SOFTWARE_CLOSED_CHOICE);
    this->msgDashLEDs.plactive = (msg.pl_active == CAN_AS_DASH_AUX_DASH_LEDS_PL_ACTIVE_ON_CHOICE);

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

void CanHandler::publish_ebs_service_brake()
{
    can_as_dash_aux_ebs_service_brake_t msg;
    if (can_as_dash_aux_ebs_service_brake_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of EBS_SERVICE_BRAKE");
        return;
    }

    this->createHeader(&this->msgEbsServiceBrake.header); 
    this->msgEbsServiceBrake.servo_commanded_percentage = msg.servo_commanded_percentage;

    this->pubEbsServiceBrake->publish(this->msgEbsServiceBrake);
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
    this->msgEbsSupervisor.ebsstatus = msg.ebs_status;
    this->msgEbsSupervisor.ebsled = (msg.ebs_led == CAN_AS_DASH_AUX_EBS_SUPERVISOR_EBS_LED_ON_CHOICE);
    this->msgEbsSupervisor.servicebrakestatus = msg.service_brake_status;
    this->msgEbsSupervisor.initialchecked = (msg.initial_checked == CAN_AS_DASH_AUX_EBS_SUPERVISOR_INITIAL_CHECKED_YES_CHOICE);
    this->msgEbsSupervisor.initialcheckstage = msg.initial_check_stage;
    this->msgEbsSupervisor.monitortankpressure = (msg.monitor_tank_pressure == CAN_AS_DASH_AUX_EBS_SUPERVISOR_MONITOR_TANK_PRESSURE_OK_CHOICE);
    this->msgEbsSupervisor.monitorbrakepressure = (msg.monitor_brake_pressure == CAN_AS_DASH_AUX_EBS_SUPERVISOR_MONITOR_BRAKE_PRESSURE_OK_CHOICE);
    this->msgEbsSupervisor.monitorservocheck = (msg.monitor_servo_check == CAN_AS_DASH_AUX_EBS_SUPERVISOR_MONITOR_SERVO_CHECK_OK_CHOICE);
    this->msgEbsSupervisor.monitorapu = (msg.monitor_apu == CAN_AS_DASH_AUX_EBS_SUPERVISOR_MONITOR_APU_OK_CHOICE);    

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

    //errors check
    this->msgCanStatus.sensor_errors = msg.analog1_error == CAN_AS_DASH_AUX_SWA_STATUS_ANALOG1_ERROR_ERROR_CHOICE ?
                                       this->msgCanStatus.sensor_errors | this->msgCanStatus.SWA_STATUS_ANALOG1_ERROR :
                                       this->msgCanStatus.sensor_errors & ~this->msgCanStatus.SWA_STATUS_ANALOG1_ERROR;
    this->msgCanStatus.sensor_errors = msg.analog2_error == CAN_AS_DASH_AUX_SWA_STATUS_ANALOG2_ERROR_ERROR_CHOICE ?
                                       this->msgCanStatus.sensor_errors | this->msgCanStatus.SWA_STATUS_ANALOG2_ERROR :
                                       this->msgCanStatus.sensor_errors & ~this->msgCanStatus.SWA_STATUS_ANALOG2_ERROR;
    this->msgCanStatus.sensor_errors = msg.stall_occurred == CAN_AS_DASH_AUX_SWA_STATUS_STALL_OCCURRED_STALL_CHOICE ?
                                       this->msgCanStatus.sensor_errors | this->msgCanStatus.SWA_STATUS_STALL_OCCURED :
                                       this->msgCanStatus.sensor_errors & ~this->msgCanStatus.SWA_STATUS_STALL_OCCURED;                            

    if (msg.analog1_error == CAN_AS_DASH_AUX_SWA_STATUS_ANALOG1_ERROR_ERROR_CHOICE || 
        msg.analog2_error == CAN_AS_DASH_AUX_SWA_STATUS_ANALOG2_ERROR_ERROR_CHOICE || 
        msg.stall_occurred == CAN_AS_DASH_AUX_SWA_STATUS_STALL_OCCURRED_STALL_CHOICE)
    {
        this->createHeader(&this->msgCanStatus.header);
        this->pubCanStatus->publish(this->msgCanStatus);
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

void CanHandler::publish_ecu_params_actual()
{
    can_as_dash_aux_ecu_params_actual_t msg;
    if (can_as_dash_aux_ecu_params_actual_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of ECU_PARAMS_ACTUAL");
        return;
    }

    this->createHeader(&this->msgEcuParams.header);
    this->msgEcuParams.inverter_rpm_max = msg.inverter_rpm_max_mean;
    this->msgEcuParams.inverter_i_rms_max = msg.inverter_i_max_mean*400.0/1070.0;
    this->msgEcuParams.power_target_kw = (float)msg.power_target_k_w_actual/255.0*80.0;
    this->msgEcuParams.ed2_gain = msg.ed2_gain_actual;
    this->msgEcuParams.inverter_i_rms_max_charging_factor = (float)msg.i_rms_max_charging_factor_actual/255.0;

    this->pubEcuParams->publish(this->msgEcuParams);
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

//Fucntions for CAN staus
void CanHandler::handleReceiveTimeout()
{
    rclcpp::Time timeNow = this->now();
    
    if (timeNow - this->msgDashApps.header.stamp > rclcpp::Duration(1s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.DASH_APPS_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.DASH_APPS_TIMEOUT;
    
    if (timeNow - this->msgDashBrake.header.stamp > rclcpp::Duration(1s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.DASH_BRAKE_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.DASH_BRAKE_TIMEOUT;

    if (timeNow - this->msgDashButtons.header.stamp > rclcpp::Duration(1s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.DASH_BUTTONS_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.DASH_BUTTONS_TIMEOUT;

    if (timeNow - this->msgDashFrontRPM.header.stamp > rclcpp::Duration(1s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.DASH_FRONT_HALL_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.DASH_FRONT_HALL_TIMEOUT;

    if (timeNow - this->msgAuxPumpsFans.header.stamp > rclcpp::Duration(1s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.AUX_PUMPS_FANS_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.AUX_PUMPS_FANS_TIMEOUT;

    if (timeNow - this->msgDashLEDs.header.stamp > rclcpp::Duration(1s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.DASH_LEDS_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.DASH_LEDS_TIMEOUT;

    if (timeNow - this->msgEbsTankPressure.header.stamp > rclcpp::Duration(1s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.EBS_TANK_PRESSURE_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.EBS_TANK_PRESSURE_TIMEOUT;

    if (timeNow - this->msgAmiSelectedMission.header.stamp > rclcpp::Duration(1s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.AMI_SELECTED_MISSION_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.AMI_SELECTED_MISSION_TIMEOUT;

    if (timeNow - this->msgSwaActual.header.stamp > rclcpp::Duration(1s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.SWA_STATUS_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.SWA_STATUS_TIMEOUT;

    if (timeNow - this->msgEbsSupervisor.header.stamp > rclcpp::Duration(1s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.EBS_SUPERVISOR_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.EBS_SUPERVISOR_TIMEOUT;

    if (timeNow - this->msgEbsServiceBrake.header.stamp > rclcpp::Duration(1s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.EBS_SERVICE_BRAKE_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.EBS_SERVICE_BRAKE_TIMEOUT;

    if (timeNow - this->msgInvCmds.header.stamp > rclcpp::Duration(1s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.INV_RESOLVERS_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.INV_RESOLVERS_TIMEOUT;
    
    if (timeNow - this->msgEcuParams.header.stamp > rclcpp::Duration(2s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.ECU_PARAMS_ACTUAL_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.ECU_PARAMS_ACTUAL_TIMEOUT;

    can_berr_counter bc;
    if (can_get_berr_counter(this->rosConf.channel0.c_str(), &bc) != 0)
        RCLCPP_INFO(this->get_logger(), "Failed to get current tx and rx errors");
    this->msgCanStatus.tx_berrors = bc.txerr;
    this->msgCanStatus.rx_berrors = bc.rxerr;

    struct rtnl_link_stats64 rls;
    if (can_get_link_stats(this->rosConf.channel0.c_str(), &rls) != 0)
        RCLCPP_INFO(this->get_logger(), "Failed to get total tx and rx errors");
    this->msgCanStatus.tx_terrors = rls.tx_errors;
    this->msgCanStatus.rx_terrors = rls.rx_errors;

    can_device_stats cds;
    if (can_get_device_stats(this->rosConf.channel0.c_str(), &cds) != 0)
        RCLCPP_INFO(this->get_logger(), "Failed to get CAN statistics");
    this->msgCanStatus.bus_errors = cds.bus_error;
    this->msgCanStatus.restarts = cds.restarts;

    if (can_get_state(this->rosConf.channel0.c_str(), &this->msgCanStatus.can_state) != 0)
        RCLCPP_INFO(this->get_logger(), "Failed to get CAN state");

    this->createHeader(&this->msgCanStatus.header);
    this->pubCanStatus->publish(this->msgCanStatus);
}

//Functions for CAN transmit
void CanHandler::apu_state_callback(turtle_interfaces::msg::StateMachineState::SharedPtr msgApuState)
{
    this->frameApuStateMission.as_state = msgApuState->state;
    if (this->rosConf.transmitApuStateMission == 1)
        this->transmit_apu_state_mission();
}

void CanHandler::apu_mission_callback(turtle_interfaces::msg::Mission::SharedPtr msgApuMission)
{
    this->frameApuStateMission.as_mission = msgApuMission->mission;
    // if (this->rosConf.transmitApuStateMission == 1)
    //     this->transmit_apu_state_mission();
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
                                                    
    if (this->rosConf.transmitSwaCommanded == 1) {
        this->transmit_swa_commanded();
    }

    this->frameApuCommand.throttle_brake_commanded = msgActuatorCmd->throttle;
    if (this->rosConf.transmitApuCommand == 1) {
        this->transmit_apu_command();
    }
}

void CanHandler::ecu_params_callback(turtle_interfaces::msg::ECUParams::SharedPtr msgECUParams)
{
    this->frameECUParams.inverter_rpm_max = msgECUParams->inverter_rpm_max;
    this->frameECUParams.inverter_i_max = msgECUParams->inverter_i_rms_max;
    this->frameECUParams.power_target_kw = (uint8_t)(msgECUParams->power_target_kw*255.0/80.0);
    this->frameECUParams.ed2_gain = msgECUParams->ed2_gain;
    this->frameECUParams.i_rms_max_charging_factor = uint8_t(msgECUParams->inverter_i_rms_max_charging_factor*255);

    if (this->rosConf.transmitECUParams == 1) {
        this->transmit_ecu_params();
    }
}

void CanHandler::control_info_callback(turtle_interfaces::msg::ControlInfo::SharedPtr msgControlInfo)
{
    this->frameDvSystemStatus.lap_counter = msgControlInfo->lap;
}

void CanHandler::slam_info_callback(turtle_interfaces::msg::SlamInfo::SharedPtr msgSlamInfo)
{
    this->frameDvSystemStatus.cones_count_actual = msgSlamInfo->sensor_cone_count;
    this->frameDvSystemStatus.cones_count_all = msgSlamInfo->total_cone_count;
}

void CanHandler::handleCanTransmit()
{
    this->canTimerCounter++;                //another 1ms passed
    if (this->canTimerCounter == 1001U)     //after 1sec
        this->canTimerCounter = 1U;         //reset counter to prevent overflow

    if ((this->rosConf.transmitApuStateMission == 2) && !(this->canTimerCounter % CAN_AS_DASH_AUX_APU_STATE_MISSION_CYCLE_TIME_MS)) {
        this->transmit_apu_state_mission();
    }
    if ((this->rosConf.transmitSwaCommanded == 2) && !(this->canTimerCounter % CAN_AS_DASH_AUX_SWA_COMMANDED_CYCLE_TIME_MS)) {
        this->transmit_swa_commanded();
    }
    if ((this->rosConf.transmitApuCommand == 2) && !(this->canTimerCounter % CAN_AS_DASH_AUX_SWA_COMMANDED_CYCLE_TIME_MS)) {
        this->transmit_apu_command();
    }
    if ((this->rosConf.transmitECUParams == 2) && !(this->canTimerCounter % CAN_AS_DASH_AUX_ECU_PARAMETERS_CYCLE_TIME_MS)) {
        this->transmit_ecu_params();
    }
    if (this->rosConf.transmitDvSystemStatus && !(this->canTimerCounter % CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_CYCLE_TIME_MS)) {
        this->transmit_dv_system_status();
    }
    if (this->rosConf.transmitApuResInit && !res_initialized && !(this->canTimerCounter % CAN_APU_RES_DLOGGER_APU_RES_INIT_CYCLE_TIME_MS)) {
        this->transmit_apu_res_init();
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

void CanHandler::transmit_ecu_params()
{
    this->sendFrame.can_id = CAN_AS_DASH_AUX_ECU_PARAMETERS_FRAME_ID;
    this->sendFrame.can_dlc = CAN_AS_DASH_AUX_ECU_PARAMETERS_LENGTH;
    if (can_as_dash_aux_ecu_parameters_pack(this->sendFrame.data, &this->frameECUParams, sizeof(sendFrame.data)) != CAN_AS_DASH_AUX_ECU_PARAMETERS_LENGTH) {
         RCLCPP_ERROR(this->get_logger(), "Error during pack of ECU_PARAMETERS");
        return;
    }

    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_AS_DASH_AUX_ECU_PARAMETERS_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of ECU_PARAMETERS");
    }
}

void fill_datalogger_variables(can_apu_res_dlogger_dv_system_status_t *frameDvSystemStatus, can_as_dash_aux_apu_state_mission_t *frameApuStateMission, turtle_interfaces::msg::EbsSupervisorInfo *msgEbsSupervisor)
{
    switch (frameApuStateMission->as_state)
    {
    case CAN_AS_DASH_AUX_APU_STATE_MISSION_AS_STATE_AS_OFF_CHOICE:
        frameDvSystemStatus->assi_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_ASSI_STATE_OFF_CHOICE;
        frameDvSystemStatus->steering_state = 0;
        break;
    case CAN_AS_DASH_AUX_APU_STATE_MISSION_AS_STATE_AS_READY_CHOICE:
        frameDvSystemStatus->assi_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_ASSI_STATE_READY_CHOICE;
        frameDvSystemStatus->steering_state = 1;
        break;
    case CAN_AS_DASH_AUX_APU_STATE_MISSION_AS_STATE_AS_DRIVING_CHOICE:
        frameDvSystemStatus->assi_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_ASSI_STATE_DRIVING_CHOICE;
        frameDvSystemStatus->steering_state = 1;
        break;
    case CAN_AS_DASH_AUX_APU_STATE_MISSION_AS_STATE_AS_EMERGENCY_CHOICE:
        frameDvSystemStatus->assi_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_ASSI_STATE_EMERGENCY_BRAKE_CHOICE;
        frameDvSystemStatus->steering_state = 1;
        break;
    case CAN_AS_DASH_AUX_APU_STATE_MISSION_AS_STATE_AS_FINISHED_CHOICE:
        frameDvSystemStatus->assi_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_ASSI_STATE_FINISH_CHOICE;
        frameDvSystemStatus->steering_state = 0;
        break;
    default:
        frameDvSystemStatus->assi_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_ASSI_STATE_OFF_CHOICE;
        frameDvSystemStatus->steering_state = 0;
        break;
    }

    switch (msgEbsSupervisor->ebsstatus)
    {
    case msgEbsSupervisor->EBS_UNAVAILABLE:
        frameDvSystemStatus->ebs_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_EBS_STATE_UNANAILABLE_CHOICE;
        break;
    case msgEbsSupervisor->EBS_ARMED:
        frameDvSystemStatus->ebs_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_EBS_STATE_ARMED_CHOICE;
        break;
    case msgEbsSupervisor->EBS_ACTIVATED:
        frameDvSystemStatus->ebs_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_EBS_STATE_TRIGGERED_CHOICE;
        break;  
    default:
        frameDvSystemStatus->ebs_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_EBS_STATE_UNANAILABLE_CHOICE;
        break;
    }

    switch (frameApuStateMission->as_mission)
    {
        case CAN_AS_DASH_AUX_APU_STATE_MISSION_AS_MISSION_ACCELERATION_CHOICE:
            frameDvSystemStatus->ami_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_AMI_STATE_ACCELERATION_CHOICE;
            break;
        case CAN_AS_DASH_AUX_APU_STATE_MISSION_AS_MISSION_SKIDPAD_CHOICE:
            frameDvSystemStatus->ami_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_AMI_STATE_SKIDPAD_CHOICE;
            break;
        case CAN_AS_DASH_AUX_APU_STATE_MISSION_AS_MISSION_TRACKDRIVE_CHOICE:
            frameDvSystemStatus->ami_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_AMI_STATE_TRACKDRIVE_CHOICE;
            break;
        case CAN_AS_DASH_AUX_APU_STATE_MISSION_AS_MISSION_EBSTEST_CHOICE:
            frameDvSystemStatus->ami_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_AMI_STATE_BRAKETEST_CHOICE;
            break;
        case CAN_AS_DASH_AUX_APU_STATE_MISSION_AS_MISSION_INSPECTION_CHOICE:
            frameDvSystemStatus->ami_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_AMI_STATE_INSPECTION_CHOICE;
            break;
        case CAN_AS_DASH_AUX_APU_STATE_MISSION_AS_MISSION_AUTOCROSS_CHOICE:
            frameDvSystemStatus->ami_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_AMI_STATE_AUTOCROSS_CHOICE;
            break;
        default:
            frameDvSystemStatus->ami_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_AMI_STATE_ACCELERATION_CHOICE;
            break;
    }

    switch (msgEbsSupervisor->servicebrakestatus)
    {
    case msgEbsSupervisor->SERVICE_BRAKE_DISENGAGED:
        frameDvSystemStatus->service_brake_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_SERVICE_BRAKE_STATE_DISENGAGED_CHOICE;
        break;
    case msgEbsSupervisor->SERVICE_BRAKE_ENGAGED:
        frameDvSystemStatus->service_brake_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_SERVICE_BRAKE_STATE_ENGAGED_CHOICE;
        break;
    case msgEbsSupervisor->SERVICE_BRAKE_AVAILABLE:
        frameDvSystemStatus->service_brake_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_SERVICE_BRAKE_STATE_AVAILABLE_CHOICE;
        break;
    default:
    frameDvSystemStatus->service_brake_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_SERVICE_BRAKE_STATE_DISENGAGED_CHOICE;
        break;
    }
}

void CanHandler::transmit_dv_system_status()
{
    fill_datalogger_variables(&this->frameDvSystemStatus, &this->frameApuStateMission, &this->msgEbsSupervisor);

    this->sendFrame.can_id = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_FRAME_ID;
    this->sendFrame.can_dlc = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_LENGTH;
    if (can_apu_res_dlogger_dv_system_status_pack(this->sendFrame.data, &this->frameDvSystemStatus, sizeof(sendFrame.data)) != CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_LENGTH) {
         RCLCPP_ERROR(this->get_logger(), "Error during pack of ECU_PARAMETERS");
        return;
    }

    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of DV_SYSTEM_STATUS");
    }
}

void CanHandler::transmit_apu_res_init() 
{
    // send CAN initialization thing
    this->sendFrame.can_id = CAN_APU_RES_DLOGGER_APU_RES_INIT_FRAME_ID;
    this->sendFrame.can_dlc = CAN_APU_RES_DLOGGER_APU_RES_INIT_LENGTH;
    if (can_apu_res_dlogger_apu_res_init_pack(this->sendFrame.data, &this->frameApuResInit, sizeof(sendFrame.data)) != CAN_APU_RES_DLOGGER_APU_RES_INIT_LENGTH){
        RCLCPP_ERROR(this->get_logger(), "Error during pack of APU_RES_INIT");
        return;
    }

    if (sendto(this->can1Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr1, this->len) < CAN_APU_RES_DLOGGER_APU_RES_INIT_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of APU_RES_INIT");
    }
}
