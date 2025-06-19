#ifndef CALLBACKS_HPP
#define CALLBACKS_HPP

#include "can_handler.hpp"
#include "raw_conversions.hpp"


void CanHandler::apu_state_callback(turtle_interfaces::msg::StateMachineState::SharedPtr msgApuState)
{
    this->frameApuStateMission.as_state = msgApuState->state;
    this->frameApuStateMission.as_set_finished = msgApuState->setfinished;
    if (this->rosConf.transmitApuStateMission == 1)
        this->transmit_apu_state_mission();
}

void CanHandler::apu_mission_callback(turtle_interfaces::msg::Mission::SharedPtr msgApuMission)
{
    this->frameApuStateMission.as_mission = msgApuMission->mission;
}

void CanHandler::apu_estimation_callback(nav_msgs::msg::Odometry::SharedPtr msgApuEstimation)
{
    this->frameApuEstimation.vel_x_estimation = msgApuEstimation->twist.twist.linear.x;
    this->frameApuEstimation.vel_y_estimation = msgApuEstimation->twist.twist.linear.y;
    this->frameApuEstimation.yaw_rate_estimation = msgApuEstimation->twist.twist.angular.z;
}

void CanHandler::actuator_cmd_callback(turtle_interfaces::msg::ActuatorCmd::SharedPtr msgActuatorCmd)
{
    this->frameSwaCommanded.position_target = msgActuatorCmd->steering; 
    this->frameSwaCommanded.steering_mode = msgActuatorCmd->steering_mode;
   
    if (this->rosConf.transmitSwaCommanded == 1) {
        this->transmit_steering_commanded();
    }

    this->frameApuCommand.throttle_brake_commanded = msgActuatorCmd->throttle;
    if (this->rosConf.transmitApuCommand == 1) {
        this->transmit_apu_command();
    }
}

void CanHandler::ecu_params_callback(turtle_interfaces::msg::ECUParams::SharedPtr msgECUParams) //change to general and control propably
{
    this->frameECUParamAPU.inverter_rpm_percentage = msgECUParams->inverter_rpm_percentage;
    this->frameECUParamAPU.inverter_irms_max = msgECUParams->inverter_i_rms_max;
    this->frameECUParamAPU.power_target = msgECUParams->power_target_kw;
    this->frameECUParamAPU.ed_enable = msgECUParams->ed_enable;
    this->frameECUParamAPU.tc_enable = msgECUParams->tc_enable;

    this->frameECUParamAPU.servo_start_speed = msgECUParams->servo_start_speed;
    this->frameECUParamAPU.regen_min_speed = msgECUParams->regen_min_speed;

    if (this->rosConf.transmitECUParamAPU == 1) {
        this->transmit_ecu_param_apu();
    }
}

void CanHandler::cpu_temps_callback(turtle_interfaces::msg::CpuStatus::SharedPtr msgCPUTemps)
{
     this->frameAPUTemps.cpu_temp = convertCPUTemp(&(msgCPUTemps->core_temperatures[0]),msgCPUTemps->number_of_temperatures);
}

void CanHandler::gpu_temp_callback(turtle_interfaces::msg::GpuStatus::SharedPtr msgGPUTemp)
{

    this->frameAPUTemps.gpu_temp = (uint16_t)(msgGPUTemp->temp_c);   

   if (this->rosConf.transmitApuTemp == 1) 
        this->transmit_apu_temps();

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

#endif
