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
    this->msgAuxBrakelight.brakelight = msg.brakelight;//CAN_AS_DASH_AUX_AUX_BRAKELIGHT_BRAKELIGHT_ON_CHOICE);// TO ADD IT ON DBC

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
    this->msgDashBrake.brake = msg.brake_pressure;//convertBrakePressure(msg.brake_pressure);

    this->pubDashBrake->publish(this->msgDashBrake);
}

void CanHandler::publish_dash_bools()
{
    can_mcu_ecu_bools_t msg;
    can_mcu_aux_states_t msg1;
    can_mcu_asb_t msg2;

   

    if (recvFrame.can_id == CAN_MCU_AUX_STATES_FRAME_ID){

        if (can_mcu_aux_states_unpack(&msg1, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_STATES");
        return;
        }

        this->msgDashBools.safestate=msg1.safe_state;
        this->msgDashBools.greentsal=msg1.green_tsal;

    }

     
    if (recvFrame.can_id == CAN_MCU_ECU_BOOLS_FRAME_ID){


        if (can_mcu_ecu_bools_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of ECU_BOOLS");
        return;
        }
        this->msgDashBools.buzzer=msg.buzzer;
        this->msgDashBools.enableout=msg.enable;


    }

    if (recvFrame.can_id == CAN_MCU_ASB_FRAME_ID){
    
        if (can_mcu_asb_unpack(&msg2, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of ASB");
        return;
        }
        this->msgDashBools.asbled=msg2.asb_led;

        
    }

    this->createHeader(&this->msgDashBools.header);
    this->pubDashBools->publish(this->msgDashBools);

}

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

    this->createHeader(&this->msgEbsServiceBrake.header); 
    this->msgEbsServiceBrake.servo_commanded_enable= msg.servo_commanded;

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
    this->msgEbsSupervisor.asmsstate = (!msg.asms_state == CAN_MCU_ASB_ASMS_STATE_ASMS_OPENED_CHOICE);
    this->msgEbsSupervisor.tsmsout = (!msg.tsms_out == CAN_MCU_ASB_TSMS_OUT_TSMS_OPENED_CHOICE);
    this->msgEbsSupervisor.ebsstatus = msg.asb_status;
    this->msgEbsSupervisor.ebsled = (!msg.asb_led == CAN_MCU_ASB_ASB_LED_ASB_LED_OFF_CHOICE);
    this->msgEbsSupervisor.servicebrakestatus = msg.service_brake_status;
    this->msgEbsSupervisor.initialchecked = (msg.initial_checked == CAN_MCU_ASB_INITIAL_CHECKED_INITIAL_CHECK_SUCCESSFUL_CHOICE);
    this->msgEbsSupervisor.initialcheckstage = msg.initial_check_step;
    this->msgEbsSupervisor.monitortankpressure = (!msg.monitor_tank_pressure == CAN_MCU_ASB_MONITOR_TANK_PRESSURE_ERROR_CHOICE);
    this->msgEbsSupervisor.monitorbrakepressure = (!msg.monitor_brake_pressure == CAN_MCU_ASB_MONITOR_BRAKE_PRESSURE_ERROR_CHOICE);
    this->msgEbsSupervisor.monitorservocheck = (!msg.monitor_servo_check == CAN_MCU_ASB_MONITOR_SERVO_CHECK_ERROR_CHOICE);
    this->msgEbsSupervisor.monitorapu = (!msg.monitor_apu == CAN_MCU_ASB_MONITOR_APU_ERROR_CHOICE);    

    this->pubEbsSupervisor->publish(this->msgEbsSupervisor);
}

void CanHandler::publish_swa_actual() //TODO
{
    can_mcu_dash_steering_t msg;
    if (can_mcu_dash_steering_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of SWA_ACTUAL");
        return;
    }

    this->createHeader(&this->msgSwaActual.header);
    this->msgSwaActual.steering = msg.steering;

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
    this->msgMotorRPM.left = (float)msg.rpm_l;

    }
    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_RIGHT_FRAME_ID){
        if (can_mcu_adu_inverter_right_unpack(&msg1, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of INV_RESOLVERS(inverter_right)");
        return;
        }
    this->msgMotorRPM.right = (float)msg1.rpm_r;
  
    }
    
    this->createHeader(&this->msgMotorRPM.header);

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
    this->msgInvCmds.torqueleft = msg.torque_l;

    }

    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_RIGHT_FRAME_ID){
        if (can_mcu_adu_inverter_right_unpack(&msg1, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of INV_RESOLVERS(inverter_right)");
            return;
        }
    this->msgInvCmds.torqueright = msg1.torque_r;

    }

    this->createHeader(&this->msgInvCmds.header);

    this->pubInvCmds->publish(this->msgInvCmds);
}

void CanHandler::publish_ecu_params_actual() 
{
    can_mcu_ecu_parameters_t msg;
    if (can_mcu_ecu_parameters_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of ECU_PARAMS_ACTUAL");
        return;
    }

    RCLCPP_INFO(this->get_logger(),"MPIKE");

    this->createHeader(&this->msgECUParamsActual.header);
    this->msgECUParamsActual.inverter_rpm_percentage = msg.inverter_rpm_percentage;
    this->msgECUParamsActual.inverter_i_rms_max = msg.inverter_i_max;//400.0/1070.0;
    this->msgECUParamsActual.power_target_kw = msg.power_target;///255.0*80.0;
    this->msgECUParamsActual.ed_enable = msg.ed_enable;
    this->msgECUParamsActual.tc_enable = msg.tc_enable;
    this->msgECUParamsActual.servo_start_speed = msg.servo_start_speed;
    this->msgECUParamsActual.regen_min_speed = msg.regen_min_speed;

    this->pubEcuParams->publish(this->msgECUParamsActual);
}

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

    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_RIGHT_FRAME_ID){ 
        if (can_mcu_adu_inverter_right_unpack(&msg,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of adu_inverter right");
            return;
        }

    this->msgInvRightInfo.igbts_temp= msg.igbt_r;
    this->msgInvRightInfo.motor_temp=msg.motor_r;
    }

    if (recvFrame.can_id == CAN_MCU_INVERTER_RIGHT_INFO_FRAME_ID){
        if (can_mcu_inverter_right_info_unpack(&msg1,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of inverter_right_info");
            return;
        }
    this->msgInvRightInfo.irms=msg1.irms_max_right;
    this->msgInvRightInfo.irms=msg1.i_lim_in_use_right;
    this->msgInvRightInfo.irms=msg1.irm_right;
    this->msgInvRightInfo.max_rpm=msg1.max_rpm_right;

    }
    
    this->createHeader(&this->msgInvLeftInfo.header);
   

    this->pubInvRightInfo->publish(this->msgInvRightInfo);
}

void CanHandler::publish_inverter_left_info() 
{   can_mcu_adu_inverter_left_t msg;
    can_mcu_inverter_left_info_t msg1;

    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_LEFT_FRAME_ID){
        if (can_mcu_adu_inverter_left_unpack(&msg,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of adu_left");
            return;
        } 
    this->msgInvLeftInfo.igbts_temp= msg.igbt_l;
    this->msgInvLeftInfo.motor_temp=msg.motor_l; 
    }

    if (recvFrame.can_id == CAN_MCU_INVERTER_LEFT_INFO_FRAME_ID){
      if (can_mcu_inverter_left_info_unpack(&msg1,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of inverter_left_info");
            return;
        }
    this->msgInvLeftInfo.irms=msg1.irms_max_left;
    this->msgInvLeftInfo.irms=msg1.i_lim_in_use_left;
    this->msgInvLeftInfo.irms=msg1.irm_left;
    this->msgInvLeftInfo.max_rpm=msg1.max_rpm_left;

    }

    this->createHeader(&this->msgInvLeftInfo.header);


    this->pubInvLeftInfo->publish(this->msgInvLeftInfo);
 
}

void CanHandler::publish_isabellen()
{
    can_mcu_isabellen_energy_t energy;
    can_mcu_isabellen_idc_t idc;
    can_mcu_isabellen_vdc_t vdc;
    can_mcu_isabellen_pdc_t pdc;

    this->createHeader(&this->msgIsabellen.header);

    if (recvFrame.can_id == CAN_MCU_ISABELLEN_ENERGY_FRAME_ID){
        if (can_mcu_isabellen_energy_unpack(&energy,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of isabellen_energy");
        }
        this->msgIsabellen.energy=energy.energy;

    }
    
    if (recvFrame.can_id == CAN_MCU_ISABELLEN_IDC_FRAME_ID){
        if (can_mcu_isabellen_idc_unpack(&idc,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of isabellen_idc");
        }
        this->msgIsabellen.idc=idc.idc;

    }

    if (recvFrame.can_id == CAN_MCU_ISABELLEN_VDC_FRAME_ID){
        if (can_mcu_isabellen_vdc_unpack(&vdc,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of isabellen_vdc");
        }
        this->msgIsabellen.vdc=vdc.vdc;

    }

    if (recvFrame.can_id == CAN_MCU_ISABELLEN_PDC_FRAME_ID){
        if (can_mcu_isabellen_pdc_unpack(&pdc,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of isabellen_pdc");
        }
    this->msgIsabellen.pdc=pdc.pdc;
    
    }



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

void CanHandler::publish_can_status(){



}
