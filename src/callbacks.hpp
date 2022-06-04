#include "can_handler.hpp"
#include "raw_conversions.hpp"

using std::placeholders::_1;


void CanHandler::apu_state_callback(turtle_interfaces::msg::StateMachineState::SharedPtr msgApuState)
{
    this->frameApuStateMission.as_state = msgApuState->state;
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
    this->frameSwaCommanded.position_target = msgActuatorCmd->steering;//convertSteeringAngleTarget(msgActuatorCmd->steering);
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

// void CanHandler::ecu_params_callback(turtle_interfaces::msg::ECUParams::SharedPtr msgECUParams)
// {
//     this->frameECUParams.inverter_rpm_max = msgECUParams->inverter_rpm_max;
//     this->frameECUParams.inverter_i_max = msgECUParams->inverter_i_rms_max;
//     this->frameECUParams.power_target_kw = (uint8_t)(msgECUParams->power_target_kw*255.0/80.0);
//     this->frameECUParams.ed2_gain = msgECUParams->ed2_gain;
//     this->frameECUParams.i_rms_max_charging_factor = uint8_t(msgECUParams->inverter_i_rms_max_charging_factor*255);

//     this->frameECUParams2.servo_min_speed = msgECUParams->servo_min_speed;
//     this->frameECUParams2.regen_min_speed = msgECUParams->regen_min_speed;

//     if (this->rosConf.transmitECUParams == 1) {
//         this->transmit_ecu_params();
//     }
//     if (this->rosConf.transmitECUParams2 == 1) {
//         this->transmit_ecu_params2();
//     }
// }

// void CanHandler::control_info_callback(turtle_interfaces::msg::ControlInfo::SharedPtr msgControlInfo)
// {
//     this->frameDvSystemStatus.lap_counter = msgControlInfo->lap;
// }

// void CanHandler::slam_info_callback(turtle_interfaces::msg::SlamInfo::SharedPtr msgSlamInfo)
// {
//     this->frameDvSystemStatus.cones_count_actual = msgSlamInfo->sensor_cone_count;
//     this->frameDvSystemStatus.cones_count_all = msgSlamInfo->total_cone_count;
// }
