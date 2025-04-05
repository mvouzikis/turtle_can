#ifndef CAN_RECEIVE_HPP
#define CAN_RECEIVE_HPP

#include "can_handler.hpp"
#include "raw_conversions.hpp"


void CanHandler::publish_ami_selected_mission()
{
    can_mcu_ami_t msg;

    if (can_mcu_ami_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AMI_SELECTED_MISSION");
        return;
    }

    this->msgAmiSelectedMission.mission = msg.ami;

    this->createHeader(&this->msgAmiSelectedMission.header);
    this->pubAmiSelectedMission->publish(this->msgAmiSelectedMission);
}

void CanHandler::publish_aux_brakelight()
{
    can_mcu_ecu_bools_t msg;

    if (can_mcu_ecu_bools_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of ECU_BRAKELIGHT");
        return;
    }

    this->msgAuxBrakelight.brakelight = msg.brakelight;//CAN_AS_DASH_AUX_AUX_BRAKELIGHT_BRAKELIGHT_ON_CHOICE);// TO ADD IT ON DBC

    this->createHeader(&this->msgAuxBrakelight.header);
    this->pubAuxBrakelight->publish(this->msgAuxBrakelight);
}



void CanHandler::publish_ebs_tank_pressure()
{
    can_mcu_asb_t msg;

    if (can_mcu_asb_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of EBS_TANK_PRESSURE"); //thelei unpacking
        return;
    }

    this->msgEbsTankPressure.ebspressureraw = can_mcu_asb_ebs_tank_pressure_decode(msg.ebs_tank_pressure); //fix convert function

    this->createHeader(&this->msgEbsTankPressure.header);
    this->pubEbsTankPressure->publish(this->msgEbsTankPressure);
}

void CanHandler::publish_aux_tsal_safe_state()
{
    can_mcu_aux_states_t msg;

    if (can_mcu_aux_states_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_States");
        return;
    }

    this->msgAuxTsalSafeState.safestate = (msg.safe_state == 1);

    this->createHeader(&this->msgAuxTsalSafeState.header);
    this->pubAuxTsalSafeState->publish(this->msgAuxTsalSafeState);
}

//--------------den to exoume sto neo dbc, theloume na to exoyme?-----------------

void CanHandler::publish_dash_apps()
{
    can_mcu_dash_apps_t msg;

    if (can_mcu_dash_apps_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_APPS");
        return;
    }

    this->msgDashApps.apps = convertAPPS(msg.apps1_raw, msg.apps2_raw);

    this->createHeader(&this->msgDashApps.header);
    this->pubDashApps->publish(this->msgDashApps);
}

void CanHandler::publish_dash_front_rpm()
{
    can_mcu_dash_hall_f_t msg;

    if (can_mcu_dash_hall_f_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_FRONT_HALL");
        return;
    }

    this->msgDashFrontRPM.left = can_mcu_dash_hall_f_hall_fl_decode(msg.hall_fl);
    this->msgDashFrontRPM.right = can_mcu_dash_hall_f_hall_fr_decode(msg.hall_fr);


    this->createHeader(&this->msgDashFrontRPM.header);
    this->pubDashFrontRPM->publish(this->msgDashFrontRPM);
}

void CanHandler::publish_dash_brake()
{
    can_mcu_dash_brake_t msg;

    if (can_mcu_dash_brake_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_BRAKE");
        return;
    }

    this->msgDashBrake.brake = can_mcu_dash_brake_brake_pressure_decode(msg.brake_pressure);

    this->createHeader(&this->msgDashBrake.header);
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

    this->msgDashButtons.enabletoggle = (bool)msg.enable_request;
    this->msgDashButtons.start = (bool)msg.start;
    this->msgDashButtons.adact = (bool)msg.ad_act;
    this->msgDashButtons.cooling_button = (bool)msg.cooling_button;

    this->createHeader(&this->msgDashButtons.header);
    this->pubDashButtons->publish(this->msgDashButtons);
}

void CanHandler::publish_ebs_service_brake()
{
    can_mcu_ecu_bools_t msg;

    if (can_mcu_ecu_bools_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of EBS_SERVICE_BRAKE");
        return;
    }
 
    this->msgEbsServiceBrake.servo_commanded_enable= msg.servo_commanded;

    this->createHeader(&this->msgEbsServiceBrake.header);
    this->pubEbsServiceBrake->publish(this->msgEbsServiceBrake);
}

void CanHandler::publish_ebs_supervisor()
{
    can_mcu_asb_t msg;

    if (can_mcu_asb_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of EBS_SUPERVISOR");
        return;
    }

    this->msgEbsSupervisor.asmsstate = (!msg.asms_state == CAN_MCU_ASB_ASMS_STATE_ASMS_OPENED_CHOICE);
    this->msgEbsSupervisor.tsmsout = (!msg.tsms_out == CAN_MCU_ASB_TSMS_OUT_TSMS_OPENED_CHOICE);
    this->msgEbsSupervisor.ebsstatus = msg.ebs_status;
    this->msgEbsSupervisor.ebsled = (!msg.asb_led == CAN_MCU_ASB_ASB_LED_ASB_LED_OFF_CHOICE);
    this->msgEbsSupervisor.servicebrakestatus = msg.service_brake_status;
    this->msgEbsSupervisor.initialchecked = (msg.initial_checked == CAN_MCU_ASB_INITIAL_CHECKED_INITIAL_CHECK_SUCCESSFUL_CHOICE);
    this->msgEbsSupervisor.initialcheckstage = msg.initial_check_step;
    this->msgEbsSupervisor.monitortankpressure = (!msg.monitor_tank_pressure == CAN_MCU_ASB_MONITOR_TANK_PRESSURE_ERROR_CHOICE);
    this->msgEbsSupervisor.monitorbrakepressure = (!msg.monitor_brake_pressure == CAN_MCU_ASB_MONITOR_BRAKE_PRESSURE_ERROR_CHOICE);
    this->msgEbsSupervisor.monitorservocheck = (!msg.monitor_servo_check == CAN_MCU_ASB_MONITOR_SERVO_CHECK_ERROR_CHOICE);
    this->msgEbsSupervisor.monitorapu = (!msg.monitor_apu == CAN_MCU_ASB_MONITOR_APU_ERROR_CHOICE);    

    this->createHeader(&this->msgEbsSupervisor.header);
    this->pubEbsSupervisor->publish(this->msgEbsSupervisor);
}

void CanHandler::publish_swa_actual()
{
    can_mcu_dash_steering_t msg;

    if (can_mcu_dash_steering_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of SWA_ACTUAL");
        return;
    }

    this->msgSwaActual.steering = convertSteeringActual(msg.steering, this->msgEcuControlSystems.steering_offset);

    this->createHeader(&this->msgSwaActual.header);
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

        this->leftMotorRPMArrived = true;

    }
    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_RIGHT_FRAME_ID){
        if (can_mcu_adu_inverter_right_unpack(&msg1, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of INV_RESOLVERS(inverter_right)");
            return;
        }
        this->msgMotorRPM.right = (float)msg1.rpm_r;

        this->rightMotorRPMArrived = true;
    }
    
    if (this->leftMotorRPMArrived && this->rightMotorRPMArrived) {
        this->rightMotorRPMArrived = false;
        this->leftMotorRPMArrived = false;

        this->createHeader(&this->msgMotorRPM.header);
        this->pubMotorRPM->publish(this->msgMotorRPM);
    }
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

        this->leftInverterCommand = true;
    }

    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_RIGHT_FRAME_ID){
        if (can_mcu_adu_inverter_right_unpack(&msg1, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of INV_RESOLVERS(inverter_right)");
            return;
        }
        this->msgInvCmds.torqueright = msg1.torque_r;

        this->rightInverterCommand = true;
    }

    if (this->leftInverterCommand && this->rightInverterCommand) {
        this->rightInverterCommand = false;
        this->leftInverterCommand = false;

        this->createHeader(&this->msgInvCmds.header);
        this->pubInvCmds->publish(this->msgInvCmds);
    }
}

void CanHandler::publish_ecu_param_general() 
{
    can_mcu_ecu_param_general_t msg; 

    if (can_mcu_ecu_param_general_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of ECU_PARAM_GENERAL");
        return;
    }

    this->msgECUParamGeneral.inverter_rpm_percentage = msg.inverter_rpm_max_actual;
    this->msgECUParamGeneral.inverter_i_rms_max = msg.inverter_irms_max_actual;//400.0/1070.0;
    this->msgECUParamGeneral.power_target_kw = msg.power_target_actual;//255.0*80.0;
    this->msgECUParamGeneral.servo_start_speed = msg.servo_start_speed_actual;

    this->createHeader(&this->msgECUParamGeneral.header);
    this->pubEcuParamGeneral->publish(this->msgECUParamGeneral);
}

void CanHandler::publish_ecu_param_control() 
{
    can_mcu_ecu_param_control_t msg; 

    if (can_mcu_ecu_param_control_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of ECU_PARAM_CONTROL");
        return;
    }

    this->msgECUParamControl.ed_enable = msg.ed_enable_actual;
    this->msgECUParamControl.tc_enable = msg.tc_enable_actual;
    this->msgECUParamControl.regen_min_speed = msg.regen_min_speed_actual;

    this->createHeader(&this->msgECUParamControl.header);
    this->pubEcuParamControl->publish(this->msgECUParamControl); 
}


void CanHandler::publish_inverter_right_info() 
{
    can_mcu_adu_inverter_right_t msg;
    can_mcu_inverter_right_info_t msg1;

    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_RIGHT_FRAME_ID){ 
        if (can_mcu_adu_inverter_right_unpack(&msg,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of adu_inverter right");
            return;
        }

        this->msgInvRightInfo.igbts_temp= msg.igbt_r*0.4;
        this->msgInvRightInfo.motor_temp=msg.motor_r*0.4;
    }

    if (recvFrame.can_id == CAN_MCU_INVERTER_RIGHT_INFO_FRAME_ID){
        if (can_mcu_inverter_right_info_unpack(&msg1,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of inverter_right_info");
            return;
        }

        this->msgInvRightInfo.irms=msg1.irms_max_r;
        this->msgInvRightInfo.irms=msg1.i_lim_in_use_r;
        this->msgInvRightInfo.irms=msg1.irms_r;
        this->msgInvRightInfo.max_rpm=can_mcu_inverter_right_info_rpm_max_r_decode(msg1.rpm_max_r);
    }
    
    this->createHeader(&this->msgInvRightInfo.header);
    this->pubInvRightInfo->publish(this->msgInvRightInfo);
}

void CanHandler::publish_inverter_left_info() 
{   
    can_mcu_adu_inverter_left_t msg;
    can_mcu_inverter_left_info_t msg1;

    if (recvFrame.can_id == CAN_MCU_ADU_INVERTER_LEFT_FRAME_ID){
        if (can_mcu_adu_inverter_left_unpack(&msg,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of adu_left");
            return;
        } 

        this->msgInvLeftInfo.igbts_temp= msg.igbt_l*0.4;
        this->msgInvLeftInfo.motor_temp=msg.motor_l*0.4; 
    }

    if (recvFrame.can_id == CAN_MCU_INVERTER_LEFT_INFO_FRAME_ID){
      if (can_mcu_inverter_left_info_unpack(&msg1,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(), "Error during unpack of inverter_left_info");
            return;
        }
        
        this->msgInvLeftInfo.irms=msg1.irms_max_l;
        this->msgInvLeftInfo.irms=msg1.i_lim_in_use_l;
        this->msgInvLeftInfo.irms=msg1.irms_l;
        this->msgInvLeftInfo.max_rpm=can_mcu_inverter_left_info_rpm_max_l_decode(msg1.rpm_max_l);

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

    if (recvFrame.can_id == CAN_MCU_ISABELLEN_ENERGY_FRAME_ID){
        if (can_mcu_isabellen_energy_unpack(&energy,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of isabellen_energy");
            return;
        }
        this->msgIsabellen.energy=energy.energy;

        this->energyArrived = true;
    }
    
    if (recvFrame.can_id == CAN_MCU_ISABELLEN_IDC_FRAME_ID){
        if (can_mcu_isabellen_idc_unpack(&idc,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of isabellen_idc");
            return;
        }
        this->msgIsabellen.idc=idc.idc;

        this->idcArrived = true;
    }

    if (recvFrame.can_id == CAN_MCU_ISABELLEN_VDC_FRAME_ID){
        if (can_mcu_isabellen_vdc_unpack(&vdc,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of isabellen_vdc");
            return;
        }
        this->msgIsabellen.vdc=vdc.vdc;

        this->vdcArrived = true;
    }

    if (recvFrame.can_id == CAN_MCU_ISABELLEN_PDC_FRAME_ID){
        if (can_mcu_isabellen_pdc_unpack(&pdc,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of isabellen_pdc");
            return;
        }
        this->msgIsabellen.pdc=pdc.pdc;

        this->pdcArrived = true;
    }

    if (this->energyArrived && this->idcArrived && this->vdcArrived && this->pdcArrived) {
        this->energyArrived = false;
        this->idcArrived = false;
        this->vdcArrived = false;
        this->pdcArrived = false;

        this->createHeader(&this->msgIsabellen.header);
        this->pubIsabellen->publish(this->msgIsabellen);
    }
}

void CanHandler::publish_ecu_control_systems()
{
    can_mcu_ecu_adu_t msg;

    if (can_mcu_ecu_adu_unpack(&msg,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
        RCLCPP_ERROR(this->get_logger(),"Error during unpack of ecu_adu");
        return;
    }

    this->msgEcuControlSystems.ed_active = msg.differential_active;
    this->msgEcuControlSystems.pl_active = msg.pl_active;
    this->msgEcuControlSystems.regen_active = msg.regen_active;
    this->msgEcuControlSystems.tc_active = msg.tc_active;
    this->msgEcuControlSystems.steering_offset = msg.steering_offset;

    this->createHeader(&this->msgEcuControlSystems.header);
    this->pubEcuControlSystem->publish(this->msgEcuControlSystems);
}

void CanHandler::publish_BLDC()
{
    can_mcu_bldc_tx_2_t msg;

    if (can_mcu_bldc_tx_2_unpack(&msg,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
        RCLCPP_ERROR(this->get_logger(),"Error during unpack of BLDC");
        return;
    }

    this->msgBLDC.steering = convertBLDCSteering(msg.position_actual_value);

    this->createHeader(&this->msgBLDC.header);
    this->pubBLDC->publish(this->msgBLDC);
}

// write pos_acc logic
void CanHandler::publish_sbg_imu(){

    can_mcu_sbg_ecan_msg_imu_accel_t msg_accel;
    can_mcu_sbg_ecan_msg_imu_gyro_t msg_gyro;
    
    if(recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_IMU_ACCEL_FRAME_ID){
        if(can_mcu_sbg_ecan_msg_imu_accel_unpack(&msg_accel, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of accel IMU");
            return;
        }
    
        this->msgSbgImu.accel.x = can_mcu_sbg_ecan_msg_imu_accel_accel_x_decode(msg_accel.accel_x);
        this->msgSbgImu.accel.y = can_mcu_sbg_ecan_msg_imu_accel_accel_y_decode(msg_accel.accel_y);
        this->msgSbgImu.accel.z = can_mcu_sbg_ecan_msg_imu_accel_accel_z_decode(msg_accel.accel_z);
    
        this->accelSbgImuArrived = true;
    }
    if(recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_IMU_GYRO_FRAME_ID){
        if(can_mcu_sbg_ecan_msg_imu_gyro_unpack(&msg_gyro, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK){
                RCLCPP_ERROR(this->get_logger(),"Error during unpack of IMU");
                return;
        }
    
        this->msgSbgImu.gyro.x = can_mcu_sbg_ecan_msg_imu_gyro_gyro_x_decode(msg_gyro.gyro_x);
        this->msgSbgImu.gyro.y = can_mcu_sbg_ecan_msg_imu_gyro_gyro_y_decode(msg_gyro.gyro_y);
        this->msgSbgImu.gyro.z = can_mcu_sbg_ecan_msg_imu_gyro_gyro_z_decode(msg_gyro.gyro_z);
    
        this->gyroSbgImuArrived = true;
    }
    if(this->accelSbgImuArrived && this->gyroSbgImuArrived){
        this->accelSbgImuArrived = false;
        this->gyroSbgImuArrived = false;
    
        this->createHeader(&this->msgSbgImu.header);
        this->pubSbgImu->publish(this->msgSbgImu);
    }
}
    
void CanHandler::publish_sbg_ekf_euler(){
    
    can_mcu_sbg_ecan_msg_ekf_euler_t msg;
    
    if(can_mcu_sbg_ecan_msg_ekf_euler_unpack(&msg,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
        RCLCPP_ERROR(this->get_logger(),"Error during unpack of EKF Euler");
        return;
    }
    
    this->msgSbgEkfEuler.angle.z = can_mcu_sbg_ecan_msg_ekf_euler_yaw_decode(msg.yaw);
    
    this->createHeader(&this->msgSbgEkfEuler.header);
    this->pubSbgEkfEuler->publish(this->msgSbgEkfEuler);
}
    
void CanHandler::publish_sbg_gps_vel(){
    
    can_mcu_sbg_ecan_msg_gps1_vel_t msg_vel_ned;
    can_mcu_sbg_ecan_msg_gps1_vel_acc_t msg_vel_acc_ned;
    
    if(recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_GPS1_VEL_FRAME_ID){
        if(can_mcu_sbg_ecan_msg_gps1_vel_unpack(&msg_vel_ned, this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of GPS VEL value");
            return;
        }
    
        this->msgSbgGpsVel.velocity.x = can_mcu_sbg_ecan_msg_gps1_vel_velocity_n_decode(msg_vel_ned.velocity_n); // north
        this->msgSbgGpsVel.velocity.y = can_mcu_sbg_ecan_msg_gps1_vel_velocity_e_decode(msg_vel_ned.velocity_e); // east
        this->msgSbgGpsVel.velocity.z = can_mcu_sbg_ecan_msg_gps1_vel_velocity_d_decode(msg_vel_ned.velocity_d); //down
    
        this->velSbgGpsVelArrived = true;
    }

    if(recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_GPS1_VEL_ACC_FRAME_ID){
        if(can_mcu_sbg_ecan_msg_gps1_vel_acc_unpack(&msg_vel_acc_ned, this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of GPS VEL Accuracy value");
            return;
        }
    
        this->msgSbgGpsVel.velocity_accuracy.x = can_mcu_sbg_ecan_msg_gps1_vel_acc_velocity_acc_n_decode(msg_vel_acc_ned.velocity_acc_n); // north acc
        this->msgSbgGpsVel.velocity_accuracy.y = can_mcu_sbg_ecan_msg_gps1_vel_acc_velocity_acc_e_decode(msg_vel_acc_ned.velocity_acc_e); // east acc
        this->msgSbgGpsVel.velocity_accuracy.z = can_mcu_sbg_ecan_msg_gps1_vel_acc_velocity_acc_d_decode(msg_vel_acc_ned.velocity_acc_d); // down acc
    
        this->velSbgGpsVelAccArrived = true;
    }
    
    // write vel_acc logic
    
    if(this->velSbgGpsVelArrived && this->velSbgGpsVelAccArrived){
        this->velSbgGpsVelArrived = false;
        this->velSbgGpsVelAccArrived = false;
    
        this->createHeader(&this->msgSbgGpsVel.header);
        this->pubSbgGpsVel->publish(this->msgSbgGpsVel);
    }
}
    
void CanHandler::publish_sbg_gps_pos(){
    
    can_mcu_sbg_ecan_msg_gps1_pos_t msg_pos_lla; // lat, lon
    can_mcu_sbg_ecan_msg_gps1_pos_alt_t msg_pos_alt; // alt
    can_mcu_sbg_ecan_msg_gps1_pos_acc_t msg_pos_acc; // lat, lon, alt accuracy
 
    // for long, lat, alt from GPS
    if(recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_GPS1_POS_FRAME_ID){
        if(can_mcu_sbg_ecan_msg_gps1_pos_unpack(&msg_pos_lla, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of SBG Long, Lat Pos");
            return;
        }

        if(can_mcu_sbg_ecan_msg_gps1_pos_alt_unpack(&msg_pos_alt, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of SBG Altitude");
            return;
        }
        
        this->msgSbgGpsPos.longitude = can_mcu_sbg_ecan_msg_gps1_pos_longitude_decode(msg_pos_lla.longitude);
        this->msgSbgGpsPos.latitude = can_mcu_sbg_ecan_msg_gps1_pos_latitude_decode(msg_pos_lla.latitude);
        this->msgSbgGpsPos.altitude = can_mcu_sbg_ecan_msg_gps1_pos_alt_altitude_decode(msg_pos_alt.altitude);

        this->posSbgGpsPosArrived = true;
    }

    if(recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_GPS1_POS_ACC_FRAME_ID){
        if(can_mcu_sbg_ecan_msg_gps1_pos_acc_unpack(&msg_pos_acc, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of SBG Long, Lat Pos");
            return;
        }
        
        this->msgSbgGpsPos.position_accuracy.x = can_mcu_sbg_ecan_msg_gps1_pos_acc_latitude_acc_decode(msg_pos_acc.latitude_acc);
        this->msgSbgGpsPos.position_accuracy.y = can_mcu_sbg_ecan_msg_gps1_pos_acc_longitude_acc_decode(msg_pos_acc.longitude_acc);
        this->msgSbgGpsPos.position_accuracy.z = can_mcu_sbg_ecan_msg_gps1_pos_acc_altitude_acc_decode(msg_pos_acc.altitude_acc);

        this->posSbgGpsPosAccArrived = true;
    }

    // add pos_acc msg
    if(this->posSbgGpsPosArrived && this->posSbgGpsPosAccArrived){
        this->posSbgGpsPosArrived = false;
        this->posSbgGpsPosAccArrived = false;

        this->createHeader(&this->msgSbgGpsPos.header);
        this->pubSbgGpsPos->publish(this->msgSbgGpsPos);
    }
}

void CanHandler::publish_sbg_ekf_nav(){

    can_mcu_sbg_ecan_msg_ekf_vel_ned_t msg_ned_vel;
    can_mcu_sbg_ecan_msg_ekf_vel_ned_acc_t msg_vel_acc_ned;

    can_mcu_sbg_ecan_msg_ekf_pos_t msg_pos_lla;
    can_mcu_sbg_ecan_msg_ekf_pos_acc_t msg_pos_acc;

    can_mcu_sbg_ecan_msg_ekf_altitude_t msg_alt;


    // add body_vel acc and pos acc

    // VELOCITY
    if(recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_EKF_VEL_NED_FRAME_ID){
        if(can_mcu_sbg_ecan_msg_ekf_vel_ned_unpack(&msg_ned_vel, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of SBG Body Vel");
            return;
        }
        
        this->msgSbgEkfNav.velocity.x = can_mcu_sbg_ecan_msg_ekf_vel_ned_velocity_n_decode(msg_ned_vel.velocity_n);
        this->msgSbgEkfNav.velocity.y = can_mcu_sbg_ecan_msg_ekf_vel_ned_velocity_e_decode(msg_ned_vel.velocity_e);
        this->msgSbgEkfNav.velocity.z = can_mcu_sbg_ecan_msg_ekf_vel_ned_velocity_d_decode(msg_ned_vel.velocity_d);

        this->ned_velSbgNavArrived = true;
    }

    // VELOCITY ACCURACY
    if(recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_EKF_VEL_NED_ACC_FRAME_ID){
        if(can_mcu_sbg_ecan_msg_ekf_vel_ned_acc_unpack(&msg_vel_acc_ned, this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of EKF NAV VEL Accuracy value");
            return;
        }
    
        this->msgSbgGpsVel.velocity_accuracy.x = can_mcu_sbg_ecan_msg_ekf_vel_ned_acc_velocity_acc_n_decode(msg_vel_acc_ned.velocity_acc_n); // north acc
        this->msgSbgGpsVel.velocity_accuracy.y = can_mcu_sbg_ecan_msg_ekf_vel_ned_acc_velocity_acc_e_decode(msg_vel_acc_ned.velocity_acc_e); // east acc
        this->msgSbgGpsVel.velocity_accuracy.z = can_mcu_sbg_ecan_msg_ekf_vel_ned_acc_velocity_acc_d_decode(msg_vel_acc_ned.velocity_acc_d); // down acc
    
        this->ned_acc_velSbgNavArrived = true;
    }
    

    // POSITION
    if(recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_EKF_POS_FRAME_ID){
        if(can_mcu_sbg_ecan_msg_ekf_pos_unpack(&msg_pos_lla,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of SBG Pos LLA");
            return;
        }

        this->msgSbgEkfNav.longitude = can_mcu_sbg_ecan_msg_ekf_pos_longitude_decode(msg_pos_lla.longitude);
        this->msgSbgEkfNav.latitude = can_mcu_sbg_ecan_msg_ekf_pos_latitude_decode(msg_pos_lla.latitude);

        this->lla_posSbgNavArrived = true;
    }

    // POSITION ACCURACY
    if(recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_EKF_POS_ACC_FRAME_ID){
        if(can_mcu_sbg_ecan_msg_ekf_pos_acc_unpack(&msg_pos_acc, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of SBG EKF NAV Long, Lat Alt Accuracy");
            return;
        }
        
        this->msgSbgEkfNav.position_accuracy.x = can_mcu_sbg_ecan_msg_ekf_pos_acc_latitude_acc_decode(msg_pos_acc.latitude_acc);
        this->msgSbgEkfNav.position_accuracy.y = can_mcu_sbg_ecan_msg_ekf_pos_acc_longitude_acc_decode(msg_pos_acc.longitude_acc);
        this->msgSbgEkfNav.position_accuracy.z = can_mcu_sbg_ecan_msg_ekf_pos_acc_altitude_acc_decode(msg_pos_acc.altitude_acc);

        this->lla_acc_posSbgNavArrived = true;
    }

    // ALTITUDE
    if(recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_EKF_ALTITUDE_FRAME_ID){
        if(can_mcu_sbg_ecan_msg_ekf_altitude_unpack(&msg_alt,this->recvFrame.data, this->recvFrame.can_dlc) !=CAN_OK){
            RCLCPP_ERROR(this->get_logger(),"Error during unpack of SBG EKF NAV Altitude");
            return;
        }

        this->msgSbgEkfNav.altitude = can_mcu_sbg_ecan_msg_ekf_altitude_altitude_decode(msg_alt.altitude);
        this->msgSbgEkfNav.undulation = can_mcu_sbg_ecan_msg_ekf_altitude_undulation_decode(msg_alt.undulation);

        this->altitudeSbgNavArrived = true;
    }

    if(this->ned_velSbgNavArrived && this->ned_acc_velSbgNavArrived && this->lla_posSbgNavArrived && this->lla_acc_posSbgNavArrived && this->altitudeSbgNavArrived){ // add  acc conditions
        this->ned_velSbgNavArrived = false;
        this->ned_acc_velSbgNavArrived = false;

        this->lla_posSbgNavArrived = false;
        this->lla_acc_posSbgNavArrived = false;

        this->altitudeSbgNavArrived = false;

        this->createHeader(&this->msgSbgEkfNav.header);
        this->pubSbgEkfNav->publish(this->msgSbgEkfNav);
    }
}



//CHANNEL1
void CanHandler::publish_res_status()
{
    can_apu_res_dlogger_res_status_t msg;
    if (can_apu_res_dlogger_res_status_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of RES_STATUS");
        return;
    }

    this->msgResStatus.stop = (msg.stop == CAN_APU_RES_DLOGGER_RES_STATUS_STOP_ON_CHOICE);
    this->msgResStatus.toggle = (msg.toggle == CAN_APU_RES_DLOGGER_RES_STATUS_TOGGLE_ON_CHOICE);
    this->msgResStatus.button = (msg.button == CAN_APU_RES_DLOGGER_RES_STATUS_BUTTON_ON_CHOICE);
    this->msgResStatus.signal_strength = msg.signal_strength;

    this->createHeader(&this->msgResStatus.header);
    this->pubResStatus->publish(this->msgResStatus);
}

#endif