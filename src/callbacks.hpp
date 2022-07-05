#include "can_handler.hpp"
#include "raw_conversions.hpp"

using std::placeholders::_1;


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
    //if (this->rosConf.transmitApuStateMission == 1)
    //	 this->transmit_apu_state_mission();
}



void CanHandler::actuator_cmd_callback(turtle_interfaces::msg::ActuatorCmd::SharedPtr msgActuatorCmd)
{
    this->frameSwaCommanded.position_target = (uint16_t)((msgActuatorCmd->steering)*100);//convertSteeringAngleTarget(msgActuatorCmd->steering);
    this->frameSwaCommanded.velocity_target = msgActuatorCmd->steering;//convertSteeringRateTarget(msgActuatorCmd->steering);
    this->frameSwaCommanded.steering_mode = msgActuatorCmd->steering_mode;
                                                    
    if (this->rosConf.transmitSwaCommanded == 1) {
        this->transmit_steering_commanded();
    }

    this->frameApuCommand.throttle_brake_commanded = msgActuatorCmd->throttle;
    if (this->rosConf.transmitApuCommand == 1) {
        this->transmit_apu_command();
    }
}

void CanHandler::ecu_params_callback(turtle_interfaces::msg::ECUParams::SharedPtr msgECUParams)
{
    this->frameECUParams.inverter_rpm_percentage = msgECUParams->inverter_rpm_percentage;
    this->frameECUParams.inverter_i_max = msgECUParams->inverter_i_rms_max;
    this->frameECUParams.power_target = msgECUParams->power_target_kw;
    this->frameECUParams.ed_enable = msgECUParams->ed_enable;
    this->frameECUParams.tc_enable = msgECUParams->tc_enable;

    this->frameECUParams.servo_start_speed = msgECUParams->servo_start_speed;
    this->frameECUParams.regen_min_speed = msgECUParams->regen_min_speed;

    if (this->rosConf.transmitECUParams == 1) {
        this->transmit_ecu_params();
    }
 
}
void CanHandler::cpu_temps_callback(turtle_interfaces::msg::CpuStatus::SharedPtr msgCPUTemps)
 {

     this->frameAPUTemps.cpu_temp = convertCPUTemp(&(msgCPUTemps->core_temperatures[0]),msgCPUTemps->number_of_temperatures);
  
    // if (this->rosConf.transmitApuTemp == 1)
	// RCLCPP_INFO(this->get_logger(),"cpu");
    //     this->transmit_apu_temps();

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
