#include "can_handler.hpp"
#include "raw_conversions.hpp"


void CanHandler::publish_ami_selected_mission()
{
    can_mcu_ami_t msg;
    if (can_mcu_ami_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AMI_SELECTED_MISSION");
        return;
    }

    this->createHeader(&this->msgAmiSelectedMission.header);
    this->msgAmiSelectedMission.mission = msg.ami;

    this->pubAmiSelectedMission->publish(this->msgAmiSelectedMission);
}

void CanHandler::publish_aux_brakelight()
{
    can_mcu_ecu_bools_t msg;
    if (can_mcu_ecu_bools_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of ECU_BRAKELIGHT");
        return;
    }

    this->createHeader(&this->msgAuxBrakelight.header);
    //this->msgAuxBrakelight.brakelight = msg.brakelight;//CAN_AS_DASH_AUX_AUX_BRAKELIGHT_BRAKELIGHT_ON_CHOICE);// TO ADD IT ON DBC

    this->pubAuxBrakelight->publish(this->msgAuxBrakelight);
}



void CanHandler::publish_ebs_tank_pressure()
{
    can_mcu_asb_t msg;
    if (can_mcu_asb_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of EBS_TANK_PRESSURE"); //thelei unpacking
        return;
    }

    this->createHeader(&this->msgEbsTankPressure.header);
    this->msgEbsTankPressure.ebspressureraw = convertEbsPressure(msg.ebs_tank_pressure); //fix convert function

    this->pubEbsTankPressure->publish(this->msgEbsTankPressure);
}

void CanHandler::publish_aux_tsal_safe_state()
{
    can_mcu_aux_states_t msg;
    if (can_mcu_aux_states_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_States");
        return;
    }

    this->createHeader(&this->msgAuxTsalSafeState.header);
    this->msgAuxTsalSafeState.safestate = (msg.safe_state == 1);

    this->pubAuxTsalSafeState->publish(this->msgAuxTsalSafeState);
}

//--------------den to exoume sto neo dbc, theloume na to exoyme?-----------------

void CanHandler::publish_cooling_info()  
{
    can_mcu_cooling_t msg;
    if (can_mcu_cooling_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of COOLING INFO");
        return;
    }

    this->createHeader(&this->msgCoolingInfo.header);
    this->msgCoolingInfo.pumpsignal = msg.pump_signal;
    this->msgCoolingInfo.accufanspwm = msg.tsac_fans;
    this->msgCoolingInfo.hallfanpwm = msg.hall_fans;
    this->msgCoolingInfo.chassisfans = msg.chassis_fans;

    this->pubCoolingInfo->publish(this->msgCoolingInfo);
}

void CanHandler::publish_dash_apps()
{
    can_mcu_dash_apps_brake_t msg;
    if (can_mcu_dash_apps_brake_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_APPS");
        return;
    }

    this->createHeader(&this->msgDashApps.header);
    this->msgDashApps.apps = convertAPPS(msg.apps1_raw, msg.apps2_raw);

    this->pubDashApps->publish(this->msgDashApps);
}

void CanHandler::publish_dash_front_rpm()
{
    can_mcu_dash_hall_f_t msg;
    if (can_mcu_dash_hall_f_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_FRONT_HALL");
        return;
    }

    this->createHeader(&this->msgDashFrontRPM.header);
    this->msgDashFrontRPM.left = msg.hall_fl;
    this->msgDashFrontRPM.right = msg.hall_fr;

    this->pubDashFrontRPM->publish(this->msgDashFrontRPM);
}

void CanHandler::publish_dash_brake()
{
    can_mcu_dash_apps_brake_t msg;
    if (can_mcu_dash_apps_brake_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_BRAKE");
        return;
    }

    this->createHeader(&this->msgDashBrake.header);
    this->msgDashBrake.brake = convertBrakePressure(msg.brake_pressure);

    this->pubDashBrake->publish(this->msgDashBrake);
}

void CanHandler::publish_dash_bools()
{
    can_mcu_ecu_bools_t msg;
    can_mcu_aux_states_t msg1;
    can_mcu_asb_t msg2;

    
    if (recvFrame.can_id == CAN_MCU_ECU_BOOLS_FRAME_ID){


        if (can_mcu_ecu_bools_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of ECU_BOOLS");
        return;
        }
    }

    if (recvFrame.can_id == CAN_MCU_AUX_STATES_FRAME_ID){

        if (can_mcu_aux_states_unpack(&msg1, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_STATES");
        return;
        }
    }

    if (recvFrame.can_id == CAN_MCU_ECU_BOOLS_FRAME_ID){
    
        if (can_mcu_asb_unpack(&msg2, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of ASB");
        return;
        }
    }

    this->createHeader(&this->msgDashBools.header);
    this->msgDashBools.buzzer=msg.buzzer;
    this->msgDashBools.safestate=msg1.safe_state;
    this->msgDashBools.enableout=msg.enable;
    this->msgDashBools.asbled=msg2.asb_led;
    this->msgDashBools.greentsal=msg1.green_tsal;
    


    this->pubDashBools->publish(this->msgDashBools);

}

//     this->createHeader(&this->msgDashLEDs.header);
//     this->msgDashLEDs.fanpwm = msg.fan_pwm;
//     this->msgDashLEDs.buzzer = (msg.buzzer == CAN_AS_DASH_AUX_DASH_LEDS_BUZZER_ON_CHOICE); //TOOD
//     this->msgDashLEDs.safestate1 = (msg.safe_state_1 == CAN_MCU_AUX_STATES_SAFE_STATE_DISABLE_CHOICE);
//     this->msgDashLEDs.enableout = (msg.enable == CAN_AS_DASH_AUX_DASH_LEDS_ENABLE_OUT_ON_CHOICE);//TOOD
//    // is on the ebs mesage this->msgDashLEDs.ebsled = (msg.asb_led == CAN_MCU_ASB_ASB_LED_ASB_LED_ON_CHOICE);
//     this->msgDashLEDs.plactive = (msg.pl_active == CAN_AS_DASH_AUX_DASH_LEDS_PL_ACTIVE_ON_CHOICE); //TOOD

//     this->pubDashLEDs->publish(this->msgDashLEDs);
// }

void CanHandler::publish_dash_buttons()
{
    can_mcu_dash_bools_t msg;
    if (can_mcu_dash_bools_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_BUTTONS");
        return;
    }

    this->createHeader(&this->msgDashButtons.header);
    this->msgDashButtons.enabletoggle = (bool)msg.enable_request;
    this->msgDashButtons.start = (bool)msg.start;
    this->msgDashButtons.adact = (bool)msg.ad_act;
    this->msgDashButtons.cooling_button = (bool)msg.cooling_button;

    this->pubDashButtons->publish(this->msgDashButtons);
}

void CanHandler::publish_ebs_service_brake()
{
    can_mcu_ecu_bools_t msg;
    if (can_mcu_ecu_bools_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of EBS_SERVICE_BRAKE");
        return;
    }

    this->createHeader(&this->msgEbsServiceBrake.header);  //prepei na bazo kathe fora createheader
    this->msgEbsServiceBrake.servo_commanded_percentage = msg.servo_commanded;

    this->pubEbsServiceBrake->publish(this->msgEbsServiceBrake);
}

void CanHandler::publish_ebs_supervisor()
{
    can_mcu_asb_t msg;
    if (can_mcu_asb_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of EBS_SUPERVISOR");
        return;
    }

    this->createHeader(&this->msgEbsSupervisor.header);
    this->msgEbsSupervisor.asmsstate = (msg.asms_state == CAN_MCU_ASB_ASMS_STATE_ASMS_OPENED_CHOICE);
    this->msgEbsSupervisor.tsmsout = (msg.tsms_out == CAN_MCU_ASB_TSMS_OUT_TSMS_OPENED_CHOICE);
    this->msgEbsSupervisor.ebsstatus = msg.asb_status;
    this->msgEbsSupervisor.ebsled = (msg.asb_led == CAN_MCU_ASB_ASB_LED_ASB_LED_ON_CHOICE);
    this->msgEbsSupervisor.servicebrakestatus = msg.service_brake_status;
    this->msgEbsSupervisor.initialchecked = (msg.initial_checked == CAN_MCU_ASB_INITIAL_CHECKED_INITIAL_CHECK_SUCCESSFUL_CHOICE);
    this->msgEbsSupervisor.initialcheckstage = msg.initial_check_step;
    this->msgEbsSupervisor.monitortankpressure = (msg.monitor_tank_pressure == CAN_MCU_ASB_MONITOR_TANK_PRESSURE_OK_CHOICE);
    this->msgEbsSupervisor.monitorbrakepressure = (msg.monitor_brake_pressure == CAN_MCU_ASB_MONITOR_BRAKE_PRESSURE_OK_CHOICE);
    this->msgEbsSupervisor.monitorservocheck = (msg.monitor_servo_check == CAN_MCU_ASB_MONITOR_SERVO_CHECK_OK_CHOICE);
    this->msgEbsSupervisor.monitorapu = (msg.monitor_apu == CAN_MCU_ASB_MONITOR_APU_OK_CHOICE);    

    this->pubEbsSupervisor->publish(this->msgEbsSupervisor);
}

void CanHandler::publish_swa_actual() //TODO
{
    can_mcu_dash_steering_t msg;
    if (can_mcu_dash_steering_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of SWA_STATUS");
        return;
    }

    this->createHeader(&this->msgSwaActual.header);
    this->msgSwaActual.steering = convertSteeringActual(msg.steering);

    //errors check
    // this->msgCanStatus.sensor_errors = msg.analog1_error == CAN_AS_DASH_AUX_SWA_STATUS_ANALOG1_ERROR_ERROR_CHOICE ?
    //                                    this->msgCanStatus.sensor_errors | this->msgCanStatus.SWA_STATUS_ANALOG1_ERROR :
    //                                    this->msgCanStatus.sensor_errors & ~this->msgCanStatus.SWA_STATUS_ANALOG1_ERROR;
    // this->msgCanStatus.sensor_errors = msg.analog2_error == CAN_AS_DASH_AUX_SWA_STATUS_ANALOG2_ERROR_ERROR_CHOICE ?
    //                                    this->msgCanStatus.sensor_errors | this->msgCanStatus.SWA_STATUS_ANALOG2_ERROR :
    //                                    this->msgCanStatus.sensor_errors & ~this->msgCanStatus.SWA_STATUS_ANALOG2_ERROR;
    // this->msgCanStatus.sensor_errors = msg.stall_occurred == CAN_AS_DASH_AUX_SWA_STATUS_STALL_OCCURRED_STALL_CHOICE ?
    //                                    this->msgCanStatus.sensor_errors | this->msgCanStatus.SWA_STATUS_STALL_OCCURED :
    //                                    this->msgCanStatus.sensor_errors & ~this->msgCanStatus.SWA_STATUS_STALL_OCCURED;                            

    // if (msg.analog1_error == CAN_AS_DASH_AUX_SWA_STATUS_ANALOG1_ERROR_ERROR_CHOICE || 
    //     msg.analog2_error == CAN_AS_DASH_AUX_SWA_STATUS_ANALOG2_ERROR_ERROR_CHOICE || 
    //     msg.stall_occurred == CAN_AS_DASH_AUX_SWA_STATUS_STALL_OCCURRED_STALL_CHOICE)
    // {
    //     this->createHeader(&this->msgCanStatus.header);
    //     this->pubCanStatus->publish(this->msgCanStatus);
    // } CHECK IF WE NEED THEM

    this->pubSwaActual->publish(this->msgSwaActual);
} 

void CanHandler::publish_motor_rpm()
{
    can_mcu_adu_inverter_left_t msg;
    can_mcu_adu_inverter_right_t msg1;
    

    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_LEFT_FRAME_ID){
        if (can_mcu_adu_inverter_left_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of INV_RESOLVERS(inverter_left)");
            return;
        }
    }
    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_RIGHT_FRAME_ID){
        if (can_mcu_adu_inverter_right_unpack(&msg1, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of INV_RESOLVERS(inverter_right)");
        return;
        }
    }
    
    this->createHeader(&this->msgMotorRPM.header);
    this->msgMotorRPM.left = (float)msg.rpm_l;
    this->msgMotorRPM.right = (float)msg1.rpm_r;

    this->pubMotorRPM->publish(this->msgMotorRPM);
}

void CanHandler::publish_inverter_commands()
{
    can_mcu_adu_inverter_left_t msg;
    can_mcu_adu_inverter_right_t msg1;

    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_LEFT_FRAME_ID){    
        if (can_mcu_adu_inverter_left_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of INV_RESOLVERS(inverter_left)");
            return;
        }
    }

    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_RIGHT_FRAME_ID){
        if (can_mcu_adu_inverter_right_unpack(&msg1, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of INV_RESOLVERS(inverter_right)");
            return;
        }
    }

    this->createHeader(&this->msgInvCmds.header);
    this->msgInvCmds.torqueleft = msg.torque_l;
    this->msgInvCmds.torqueright = msg1.torque_r;

    this->pubInvCmds->publish(this->msgInvCmds);
}

// void CanHandler::publish_ecu_params_actual() //TODO
// {
//     can_as_dash_aux_ecu_params_actual_t msg;
//     if (can_as_dash_aux_ecu_params_actual_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
//         RCLCPP_ERROR(this->get_logger(), "Error during unpack of ECU_PARAMS_ACTUAL");
//         return;
//     }

//     this->createHeader(&this->msgEcuParams.header);
//     this->msgEcuParams.inverter_rpm_max = msg.inverter_rpm_max_mean;
//     this->msgEcuParams.inverter_i_rms_max = msg.inverter_i_max_mean*400.0/1070.0;
//     this->msgEcuParams.power_target_kw = (float)msg.power_target_k_w_actual/255.0*80.0;
//     this->msgEcuParams.ed2_gain = msg.ed2_gain_actual;
//     this->msgEcuParams.inverter_i_rms_max_charging_factor = (float)msg.i_rms_max_charging_factor_actual/255.0;

//     this->pubEcuParams->publish(this->msgEcuParams);
// }

// void CanHandler::publish_ecu_params_actual2() //TODO
// {
//     can_as_dash_aux_ecu_params_actual2_t msg;
//     if (can_as_dash_aux_ecu_params_actual2_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
//         RCLCPP_ERROR(this->get_logger(), "Error during unpack of ECU_PARAMS_ACTUAL2");
//         return;
//     }

//     this->createHeader(&this->msgEcuParams.header);
//     this->msgEcuParams.servo_min_speed = msg.servo_min_speed_actual;
//     this->msgEcuParams.regen_min_speed = msg.regen_min_speed_actual;

//     this->pubEcuParams->publish(this->msgEcuParams);
// }

void CanHandler::publish_inverter_right_info() 
{
    can_mcu_adu_inverter_right_t msg;
    can_mcu_inverter_right_info_t msg1;

    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_LEFT_FRAME_ID){
        if (can_mcu_adu_inverter_right_unpack(&msg,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of adu_inverter right");
            return;
        }
    }

    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_RIGHT_FRAME_ID){
        if (can_mcu_inverter_right_info_unpack(&msg1,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of inverter_right_info");
            return;
        }
    }
    
    this->createHeader(&this->msgInvLeftInfo.header);
    this->msgInvLeftInfo.igbts_temp= msg.igbt_r;
    this->msgInvLeftInfo.irms=msg1.irms_max_right;
    this->msgInvLeftInfo.irms=msg1.i_lim_in_use_right;
    this->msgInvLeftInfo.motor_temp=msg.motor_r;
    this->msgInvLeftInfo.irms=msg1.irm_right;
    this->msgInvLeftInfo.max_rpm=msg1.max_rpm_right;

    this->pubInvLeftInfo->publish(this->msgInvLeftInfo);
}

void CanHandler::publish_inverter_left_info() 
{   can_mcu_adu_inverter_left_t msg;
    can_mcu_inverter_left_info_t msg1;

    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_LEFT_FRAME_ID){
        if (can_mcu_adu_inverter_left_unpack(&msg,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of adu_left");
            return;
        } 
    }

    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_RIGHT_FRAME_ID){
      if (can_mcu_inverter_left_info_unpack(&msg1,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of inverter_left_info");
            return;
        }
    }

    this->createHeader(&this->msgInvLeftInfo.header);
    this->msgInvLeftInfo.igbts_temp= msg.igbt_l;
    this->msgInvLeftInfo.irms=msg1.irms_max_left;
    this->msgInvLeftInfo.irms=msg1.i_lim_in_use_left;
    this->msgInvLeftInfo.motor_temp=msg.motor_l;
    this->msgInvLeftInfo.irms=msg1.irm_left;
    this->msgInvLeftInfo.max_rpm=msg1.max_rpm_left;

    this->pubInvLeftInfo->publish(this->msgInvLeftInfo);
 
}

void CanHandler::publish_isabellen()
{
    can_mcu_isabellen_energy_t energy;
    can_mcu_isabellen_idc_t idc;
    can_mcu_isabellen_vdc_t vdc;
    can_mcu_isabellen_pdc_t pdc;

    if (recvFrame.can_id == CAN_MCU_ISABELLEN_ENERGY_LENGTH){
        if (can_mcu_isabellen_energy_unpack(&energy,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of isabellen_energy");
        }
    }
    
    if (recvFrame.can_id == CAN_MCU_ISABELLEN_IDC_LENGTH){
        if (can_mcu_isabellen_idc_unpack(&idc,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of isabellen_idc");
        }
    }

    if (recvFrame.can_id == CAN_MCU_ISABELLEN_VDC_LENGTH){
        if (can_mcu_isabellen_vdc_unpack(&vdc,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of isabellen_vdc");
        }
    }

    if (recvFrame.can_id == CAN_MCU_ISABELLEN_PDC_LENGTH){
        if (can_mcu_isabellen_pdc_unpack(&pdc,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of isabellen_pdc");
        }
    }


    this->createHeader(&this->msgIsabellen.header);
    this->msgIsabellen.energy=energy.energy;
    this->msgIsabellen.idc=idc.idc;
    this->msgIsabellen.vdc=vdc.vdc;
    this->msgIsabellen.pdc=pdc.pdc;

    this->pubIsabellen->publish(this->msgIsabellen);
}

void CanHandler::publish_ecu_control_systems()
{
    can_mcu_ecu_adu_t msg;

    if (can_mcu_ecu_adu_unpack(&msg,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
        RCLCPP_ERROR(this->get_logger(),"Error during unpack of ecu_adu");
    }

    this->createHeader(&this->msgEcuControlSystems.header);
    this->msgEcuControlSystems.ed_active=msg.differential_active;
    this->msgEcuControlSystems.pl_active=msg.pl_active;
    this->msgEcuControlSystems.regen_active=msg.regen_active;
    this->msgEcuControlSystems.tc_active=msg.tc_active;

    this->pubEcuControlSystem->publish(this->msgEcuControlSystems);
}