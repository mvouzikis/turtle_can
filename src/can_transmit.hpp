#include "can_handler.hpp"

void CanHandler::transmit_apu_state_mission()
{
    this->sendFrame.can_id = CAN_MCU_APU_STATE_MISSION_FRAME_ID;
    this->sendFrame.can_dlc = CAN_MCU_APU_STATE_MISSION_LENGTH;
    if (can_mcu_apu_state_mission_pack(this->sendFrame.data, &this->frameApuStateMission, sizeof(sendFrame.data)) != CAN_MCU_APU_STATE_MISSION_LENGTH){
        RCLCPP_ERROR(this->get_logger(), "Error during pack of APU_STATE_MISSION");
        return;
    }

    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_MCU_APU_STATE_MISSION_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of APU_STATE_MISSION");
    }
}

void CanHandler::transmit_steering_commanded() 
{
    this->sendFrame.can_id = CAN_MCU_STEERING_COMMAND_FRAME_ID;
    this->sendFrame.can_dlc = CAN_MCU_STEERING_COMMAND_LENGTH;
    if (can_mcu_steering_command_pack(this->sendFrame.data, &this->frameSwaCommanded, sizeof(sendFrame.data)) != CAN_MCU_STEERING_COMMAND_LENGTH){
        RCLCPP_ERROR(this->get_logger(), "Error during pack of STEERING_COMMAND");
        return;
    }
    
    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_MCU_STEERING_COMMAND_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of STEERING_COMMAND");
    }
}

void CanHandler::transmit_apu_command() 
{
    this->sendFrame.can_id = CAN_MCU_APU_COMMAND_FRAME_ID;
    this->sendFrame.can_dlc = CAN_MCU_APU_COMMAND_LENGTH;
    if (can_mcu_apu_command_pack(this->sendFrame.data, &this->frameApuCommand, sizeof(sendFrame.data)) != CAN_MCU_APU_COMMAND_LENGTH){
        RCLCPP_ERROR(this->get_logger(), "Error during pack of APU_COMMAND");
        return;
    }

    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_MCU_APU_COMMAND_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of APU_COMMAND");
    }
}

void CanHandler ::transmit_apu_temps()
{
    this->sendFrame.can_id = CAN_MCU_APU_TEMPS_FRAME_ID;
    this->sendFrame.can_dlc = CAN_MCU_APU_TEMPS_LENGTH;
    if (can_mcu_apu_temps_pack(this->sendFrame.data, &this->frameAPUTemps, sizeof(sendFrame.data)) != CAN_MCU_APU_COMMAND_LENGTH){
        RCLCPP_ERROR(this->get_logger(), "Error during pack of APU_TEMPS");
        return;
    }

    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_MCU_APU_TEMPS_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of APU_TEMPS");
    }
}

void CanHandler::transmit_ecu_params() //TODO
{
    // this->sendFrame.can_id = CAN_AS_DASH_AUX_ECU_PARAMETERS_FRAME_ID;
    // this->sendFrame.can_dlc = CAN_AS_DASH_AUX_ECU_PARAMETERS_LENGTH;
    // if (can_as_dash_aux_ecu_parameters_pack(this->sendFrame.data, &this->frameECUParams, sizeof(sendFrame.data)) != CAN_AS_DASH_AUX_ECU_PARAMETERS_LENGTH) {
    //      RCLCPP_ERROR(this->get_logger(), "Error during pack of ECU_PARAMETERS");
    //     return;
    // }

    // if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_AS_DASH_AUX_ECU_PARAMETERS_LENGTH) {
    //     RCLCPP_ERROR(this->get_logger(), "Error during transmit of ECU_PARAMETERS");
    // }
}

// void CanHandler::transmit_ecu_params2() //TODO
// {
//     // this->sendFrame.can_id = CAN_AS_DASH_AUX_ECU_PARAMETERS2_FRAME_ID;
//     // this->sendFrame.can_dlc = CAN_AS_DASH_AUX_ECU_PARAMETERS2_LENGTH;
//     // if (can_as_dash_aux_ecu_parameters2_pack(this->sendFrame.data, &this->frameECUParams2, sizeof(sendFrame.data)) != CAN_AS_DASH_AUX_ECU_PARAMETERS2_LENGTH) {
//     //      RCLCPP_ERROR(this->get_logger(), "Error during pack of ECU_PARAMETERS2");
//     //     return;
//     // }

//     // if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_AS_DASH_AUX_ECU_PARAMETERS2_LENGTH) {
//     //     RCLCPP_ERROR(this->get_logger(), "Error during transmit of ECU_PARAMETERS2");
//     // }
// }

