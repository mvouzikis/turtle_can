#ifndef VARIABLES_INIT_HPP
#define VARIABLES_INIT_HPP

#include "can_handler.hpp"


void CanHandler::variablesInit()
{
    rclcpp::SensorDataQoS sensorQos;
    rclcpp::ServicesQoS serviceQos;

    RCLCPP_INFO(this->get_logger(), "steering offset: %.2f", this->steering_offset);

    //--------- Initialize publishers
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
    if (this->rosConf.publishMotorRPM) {
        this->pubMotorRPM = this->create_publisher<turtle_interfaces::msg::RPM>("rpm_motor", sensorQos);
        this->msgMotorRPM = turtle_interfaces::msg::RPM();
    }
     if (this->rosConf.publishAuxTsalSafeState) {
         this->pubAuxTsalSafeState = this->create_publisher<turtle_interfaces::msg::TsalSafeState>("tsal_safe_state", sensorQos);
         this->msgAuxTsalSafeState = turtle_interfaces::msg::TsalSafeState();
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
    if (this->rosConf.publishDashBools) {
        this->pubDashBools = this->create_publisher<turtle_interfaces::msg::DashBools>("dash_bools", serviceQos);
        this->msgDashBools = turtle_interfaces::msg::DashBools();
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

    if (this->rosConf.publishInverterCommands) {
        this->pubInvCmds = this->create_publisher<turtle_interfaces::msg::InverterCommands>("inverter_commands", sensorQos);
        this->msgInvCmds = turtle_interfaces::msg::InverterCommands();
    }

    if (this->rosConf.publishInverterRightInfo) {
        this->pubInvRightInfo = this->create_publisher<turtle_interfaces::msg::InverterInfo>("inverter_right_info", serviceQos);
        this->msgInvRightInfo = turtle_interfaces::msg::InverterInfo();
    }

    if (this->rosConf.publishInverterLeftInfo) {
        this->pubInvLeftInfo = this->create_publisher<turtle_interfaces::msg::InverterInfo>("inverter_left_info", serviceQos);
        this->msgInvLeftInfo = turtle_interfaces::msg::InverterInfo();
    }

    if (this->rosConf.publishResStatus) {
        this->pubResStatus = this->create_publisher<turtle_interfaces::msg::ResStatus>("res_status", serviceQos);
        this->msgResStatus = turtle_interfaces::msg::ResStatus(); 
    }

    if (this->rosConf.publishECUParamGeneral) {
        this->pubEcuParamGeneral = this->create_publisher<turtle_interfaces::msg::ECUParams>("ecu_param_general", serviceQos);
        this->msgECUParamGeneral = turtle_interfaces::msg::ECUParams();
    }

    if (this->rosConf.publishECUParamControl) {
        this->pubEcuParamControl = this->create_publisher<turtle_interfaces::msg::ECUParams>("ecu_params_control", serviceQos);
        this->msgECUParamControl = turtle_interfaces::msg::ECUParams();
    }

    if (this->rosConf.publishIsabellen) {  
        this->pubIsabellen = this->create_publisher<turtle_interfaces::msg::Isabellen>("isabellen", sensorQos);
        this->msgIsabellen= turtle_interfaces::msg::Isabellen();
    }

    if (this->rosConf.publishEcuControlSystems) {  
        this->pubEcuControlSystem = this->create_publisher<turtle_interfaces::msg::ECUControlSystems>("ecu_control_systems", sensorQos);
        this->msgEcuControlSystems= turtle_interfaces::msg::ECUControlSystems();
    }

    if (this->rosConf.publishCanStatus) {
        this->pubCanStatus = this->create_publisher<turtle_interfaces::msg::CanStatus>("can_status", serviceQos);
        this->msgCanStatus = turtle_interfaces::msg::CanStatus();
    }

    if (this->rosConf.publishCanStatus) {
        this->pubBLDC = this->create_publisher<turtle_interfaces::msg::Steering>("bldc", sensorQos);
        this->msgBLDC = turtle_interfaces::msg::Steering();
    }

    if(this->rosConf.publishSbgImu){
        this->pubSbgImu = this->create_publisher<sbg_driver::msg::SbgImuData>("sbg/imu_data", sensorQos);
        this->msgSbgImu = sbg_driver::msg::SbgImuData();
    }

    if(this->rosConf.publishSbgEkfEuler){
        this->pubSbgEkfEuler = this->create_publisher<sbg_driver::msg::SbgEkfEuler>("sbg/ekf_euler", sensorQos);
        this->msgSbgEkfEuler  = sbg_driver::msg::SbgEkfEuler();
    }

    if(this->rosConf.publishSbgGpsVel){
        this->pubSbgGpsVel = this->create_publisher<sbg_driver::msg::SbgGpsVel>("sbg/gps_vel", sensorQos);
        this->msgSbgGpsVel  = sbg_driver::msg::SbgGpsVel();
    }

    if(this->rosConf.publishSbgGpsPos){
        this->pubSbgGpsPos = this->create_publisher<sbg_driver::msg::SbgGpsPos>("sbg/gps_pos", sensorQos);
        this->msgSbgGpsPos  = sbg_driver::msg::SbgGpsPos();
    }

    if(this->rosConf.publishSbgEkfNav){
        this->pubSbgEkfNav = this->create_publisher<sbg_driver::msg::SbgEkfNav>("sbg/ekf_nav", sensorQos);
        this->msgSbgEkfNav  = sbg_driver::msg::SbgEkfNav();
    }

    //--------- Initialize CAN Tx messages
    if (this->rosConf.transmitApuStateMission || this->rosConf.transmitDvSystemStatus) {
        this->subApuState = this->create_subscription<turtle_interfaces::msg::StateMachineState>("state_flowchart_state", serviceQos, std::bind(&CanHandler::apu_state_callback, this, _1));
        this->subApuMission = this->create_subscription<turtle_interfaces::msg::Mission>("current_mission", serviceQos, std::bind(&CanHandler::apu_mission_callback, this, _1));
        this->frameApuStateMission.as_mission = CAN_MCU_APU_STATE_MISSION_AS_MISSION_NO_MISSION_CHOICE;
        this->frameApuStateMission.as_state = CAN_MCU_APU_STATE_MISSION_AS_STATE_AS_OFF_CHOICE;
        this->frameApuStateMission.as_set_finished = CAN_MCU_APU_STATE_MISSION_AS_SET_FINISHED_SET__FINISHED__FALSE_CHOICE;
    }

    if (this->rosConf.transmitApuEstimation) {
        this->subApuEstimation = this->create_subscription<nav_msgs::msg::Odometry>("odom", serviceQos, std::bind(&CanHandler::apu_estimation_callback, this, _1));
        this->frameApuEstimation.vel_x_estimation = 0;
        this->frameApuEstimation.vel_y_estimation = 0;
        this->frameApuEstimation.yaw_rate_estimation = 0;
    }

    if (this->rosConf.transmitSwaCommanded || this->rosConf.transmitApuCommand) {
        this->subActuatorCmd = this->create_subscription<turtle_interfaces::msg::ActuatorCmd>("cmd", sensorQos, std::bind(&CanHandler::actuator_cmd_callback, this, _1));
    }
    
    if (this->rosConf.transmitSwaCommanded) {
        this->frameSwaCommanded.position_target = 0.0;
        this->frameSwaCommanded.steering_mode = 1;  // CAN_MCU_DASH_STEERING_COMMAND ;
    }
    
    if (this->rosConf.transmitApuCommand) {
        this->frameApuCommand.throttle_brake_commanded = 0.0;
    }
    
    if (this->rosConf.transmitECUParamAPU) {
        this->subECUParamAPU = this->create_subscription<turtle_interfaces::msg::ECUParams>("ecu_params_tune", serviceQos, std::bind(&CanHandler::ecu_params_callback, this, _1));

        this->frameECUParamAPU.inverter_rpm_percentage = 100;
        this->frameECUParamAPU.inverter_irms_max = 100;
        this->frameECUParamAPU.power_target = 40.0;
        this->frameECUParamAPU.servo_start_speed = 5;
        this->frameECUParamAPU.regen_min_speed = 5;
        this->frameECUParamAPU.ed_enable = 0;
        this->frameECUParamAPU.tc_enable= 1;
    }

    if (this->rosConf.transmitApuTemp ) {

        this->subGPUTemp = this->create_subscription<turtle_interfaces::msg::GpuStatus>("gpu_status", serviceQos, std::bind(&CanHandler::gpu_temp_callback, this, _1));
      	this->subCPUTemps = this->create_subscription<turtle_interfaces::msg::CpuStatus>("cpu_status", serviceQos, std::bind(&CanHandler::cpu_temps_callback, this, _1));

        this->frameAPUTemps.cpu_temp = 0;
        this->frameAPUTemps.gpu_temp = 0;
        
    }

    //--------- FOR FSG DATA LOGGER
    if (this->rosConf.transmitDvSystemStatus) {
        this->subControlInfo = this->create_subscription<turtle_interfaces::msg::ControlInfo>("control_info", serviceQos, std::bind(&CanHandler::control_info_callback, this, _1));
        this->subSlamInfo = this->create_subscription<turtle_interfaces::msg::SlamInfo>("slam_info", serviceQos, std::bind(&CanHandler::slam_info_callback, this, _1));

        this->frameDvSystemStatus.as_state = CAN_MCU_DV_SYSTEM_STATUS_AS_STATE_OFF_CHOICE;
        this->frameDvSystemStatus.asb_ebs_state = CAN_MCU_DV_SYSTEM_STATUS_ASB_EBS_STATE_DEACTIVATED_CHOICE;
        this->frameDvSystemStatus.ami_state = CAN_MCU_DV_SYSTEM_STATUS_AMI_STATE_ACCELERATION_CHOICE;
        this->frameDvSystemStatus.steering_state = 0;
        this->frameDvSystemStatus.asb_redundancy_state = CAN_MCU_DV_SYSTEM_STATUS_ASB_REDUNDANCY_STATE_DEACTIVATED_CHOICE;
        this->frameDvSystemStatus.lap_counter = 0;
        this->frameDvSystemStatus.cones_count_actual = 0;
        this->frameDvSystemStatus.cones_count_all = 0;
    }
    if (this->rosConf.transmitApuResInit) {
        this->frameApuResInit.requested_state = CAN_MCU_APU_RES_INIT_REQUESTED_STATE_OPERATIONAL_CHOICE;
        this->frameApuResInit.addresed_node = CAN_MCU_APU_RES_INIT_ADDRESED_NODE_RES_ADDRESS_CHOICE;
    }

    if (this->rosConf.transmitDvDrivingDynamics1) {
        this->frameDvDrivingDynamics1.speed_actual = 0;
        this->frameDvDrivingDynamics1.speed_target = 0;
        this->frameDvDrivingDynamics1.steering_angle_actual = 0;
        this->frameDvDrivingDynamics1.steering_angle_target = 0;
        this->frameDvDrivingDynamics1.brake_hydr_acrtual = 0;
        this->frameDvDrivingDynamics1.brake_hydr_target = 0;
        this->frameDvDrivingDynamics1.motor_moment_actual = 0;
        this->frameDvDrivingDynamics1.motor_moment_target = 0;
    }

    if(this->rosConf.transmitDvDrivingDynamics2) {
        this->frameDvDrivingDynamics2.acceleration_longitudinal = 0;
        this->frameDvDrivingDynamics2.acceleration_lateral = 0;
        this->frameDvDrivingDynamics2.yaw_rate = 0;
    }
}

#endif
