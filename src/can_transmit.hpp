#ifndef CAN_TRANSMIT_HPP
#define CAN_TRANSMIT_HPP

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

// void CanHandler::transmit_steering_commanded() 
// {
//     this->sendFrame.can_id = CAN_MCU_BLDC_RX_2_FRAME_ID;
//     this->sendFrame.can_dlc = CAN_MCU_BLDC_RX_2_LENGTH;
//     if (can_mcu_bldc_rx_2_pack(this->sendFrame.data, &this->frameSwaCommanded, sizeof(sendFrame.data)) != CAN_MCU_BLDC_RX_2_LENGTH){
//         RCLCPP_ERROR(this->get_logger(), "Error during pack of STEERING_COMMAND");
//         return;
//     }
//     
//     if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_MCU_BLDC_RX_2_LENGTH) {
//         RCLCPP_ERROR(this->get_logger(), "Error during transmit of STEERING_COMMAND");
//     }
// }

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

void CanHandler::transmit_apu_estimation()
{
    
    this->sendFrame.can_id = CAN_MCU_APU_ESTIMATION_FRAME_ID;
    this->sendFrame.can_dlc = CAN_MCU_APU_ESTIMATION_LENGTH;
    if(can_mcu_apu_estimation_pack(this->sendFrame.data, &this->frameApuEstimation, sizeof(sendFrame.data)) != CAN_MCU_APU_ESTIMATION_LENGTH){
        RCLCPP_ERROR(this->get_logger(), "Error during pack of APU_ESTIMATION");
        return;
    }

    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_MCU_APU_ESTIMATION_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of APU_ESTIMATION");
    }
}

void CanHandler ::transmit_apu_temps()
{
    this->sendFrame.can_id = CAN_MCU_APU_TEMP_FRAME_ID;
    this->sendFrame.can_dlc = CAN_MCU_APU_TEMP_LENGTH;
    if (can_mcu_apu_temp_pack(this->sendFrame.data, &this->frameAPUTemps, sizeof(sendFrame.data)) != CAN_MCU_APU_TEMP_LENGTH){
        RCLCPP_ERROR(this->get_logger(), "Error during pack of APU_TEMPS");
        return;
    }

    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_MCU_APU_TEMP_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of APU_TEMPS");
    }
}

void CanHandler::transmit_ecu_param_apu() 
{
    this->sendFrame.can_id = CAN_MCU_ECU_PARAM_APU_FRAME_ID;
    this->sendFrame.can_dlc = CAN_MCU_ECU_PARAM_APU_LENGTH;
    if (can_mcu_ecu_param_apu_pack(this->sendFrame.data, &this->frameECUParamAPU, sizeof(sendFrame.data)) != CAN_MCU_ECU_PARAM_APU_LENGTH) {
         RCLCPP_ERROR(this->get_logger(), "Error during pack of ECU_PARAM_APU");
        return;
    }

    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_MCU_ECU_PARAM_APU_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of ECU_PARAM_APU");
    }
}


void CanHandler::transmit_apu_res_init() 
{
    //send CAN initialization
    this->sendFrame.can_id = CAN_MCU_APU_RES_INIT_FRAME_ID;
    this->sendFrame.can_dlc = CAN_MCU_APU_RES_INIT_LENGTH;
    if (can_mcu_apu_res_init_pack(this->sendFrame.data, &this->frameApuResInit, sizeof(sendFrame.data)) != CAN_MCU_APU_RES_INIT_LENGTH){
        RCLCPP_ERROR(this->get_logger(), "Error during pack of APU_RES_INIT");
        return;
    }

    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_MCU_APU_RES_INIT_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of APU_RES_INIT");
    }
}

#endif
