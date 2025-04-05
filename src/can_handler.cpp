#include "can_handler.hpp"
#include "can_receive.hpp"
#include "load_ros_params.hpp"
#include "callbacks.hpp"
#include "variables_init.hpp"
#include "can_transmit.hpp"


CanHandler::CanHandler(rclcpp::NodeOptions nOpt):Node("CanInterface", "", nOpt)
{
    this->loadRosParams();
    this->variablesInit();

    //initialize channel0 
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

    //initialize timers
    this->canRecvTimer =    this->create_wall_timer(500us,  std::bind(&CanHandler::handleCanReceive,        this));
    this->canRecvTimeout =  this->create_wall_timer(10ms,   std::bind(&CanHandler::handleReceiveTimeout,    this));
    this->canSendTimer =    this->create_wall_timer(1ms,    std::bind(&CanHandler::handleCanTransmit,       this));

    //initiazlize can message state variables
    this->leftMotorRPMArrived = false;
    this->rightMotorRPMArrived = false;

    this->leftInverterCommand = false;
    this->rightInverterCommand = false;

    this->energyArrived = false;
    this->idcArrived = false;
    this->vdcArrived = false;
    this->pdcArrived = false;

    res_initialized = false;

    // sbg params
    this->accelSbgImuArrived = false;
    this->gyroSbgImuArrived = false;

    this->velSbgGpsVelArrived = false;
    this->velSbgGpsVelAccArrived = false;

    this->ned_velSbgNavArrived = false;
    this->ned_acc_velSbgNavArrived = false;

    this->posSbgGpsPosArrived = false;
    this->posSbgGpsPosAccArrived = false;

    this->lla_posSbgNavArrived = false;
    this->lla_acc_posSbgNavArrived = false;

    this->altitudeSbgNavArrived = false;
    this->altAccSbgNavArrived = false;


    RCLCPP_INFO(this->get_logger(), "Communication started");
}

CanHandler::~CanHandler()
{
    if (close(this->can0Socket) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to close %s.\n", this->rosConf.channel0.c_str());
    }
}



//Functions for CAN receive
void CanHandler::handleCanReceive()
{
    //For channel 0
    while (recvfrom(this->can0Socket, &this->recvFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, &this->len) >= 8) {
        ioctl(this->can0Socket, SIOCGSTAMP, &this->recvTime);  // get message timestamp

        if (this->recvFrame.can_id == CAN_MCU_AMI_FRAME_ID && this->rosConf.publishAmiSelectedMission) {
            this->publish_ami_selected_mission();
        }
        else if (this->recvFrame.can_id == CAN_MCU_AUX_STATES_FRAME_ID) {
            if (this->rosConf.publishAuxBrakelight)
                this->publish_aux_brakelight();
            if (this->rosConf.publishAuxTsalSafeState)
                this->publish_aux_tsal_safe_state();
            if (this->rosConf.publishDashBools)
                this->publish_dash_bools();
        }    
        else if (this->recvFrame.can_id == CAN_MCU_ASB_FRAME_ID) {
            if (this->rosConf.publishEbsTankPressure)
            this->publish_ebs_tank_pressure();
            if (this->rosConf.publishEbsSupervisor)
                this->publish_ebs_supervisor();
            if (this->rosConf.publishDashBools)
                this->publish_dash_bools();
        }
        else if (this->recvFrame.can_id == CAN_MCU_DASH_HALL_F_FRAME_ID && this->rosConf.pubishDashFrontRPM) {
            this->publish_dash_front_rpm();
        }
        else if (this->recvFrame.can_id == CAN_MCU_DASH_APPS_FRAME_ID) {
            if (this->rosConf.publishDashApps)           
                this->publish_dash_apps();
        }
        else if (this->recvFrame.can_id == CAN_MCU_DASH_BRAKE_FRAME_ID) {
           
            if (this->rosConf.publishDashBrake)
                this->publish_dash_brake();
        }

        else if (this->recvFrame.can_id == CAN_MCU_ECU_BOOLS_FRAME_ID) {
            if (this->rosConf.publishDashBools)  
                this->publish_dash_bools();
            if (this->rosConf.publishEbsServiceBrake)
                this->publish_ebs_service_brake();
        }

         else if (this->recvFrame.can_id == CAN_MCU_DASH_BOOLS_FRAME_ID) {
            if (this->rosConf.publishDashButtons)  
                this->publish_dash_buttons();

        }

        else if (this->recvFrame.can_id == CAN_MCU_DASH_STEERING_FRAME_ID && this->rosConf.publishSwaStatus) {
            this->publish_swa_actual();
        }
        else if (this->recvFrame.can_id == CAN_MCU_ADU_INVERTER_LEFT_FRAME_ID) {
            if (this->rosConf.publishMotorRPM)
                this->publish_motor_rpm(); 
            if (this->rosConf.publishInverterCommands)
                this->publish_inverter_commands();
            if (this->rosConf.publishInverterLeftInfo) 
                this->publish_inverter_left_info();

        }
        else if (this->recvFrame.can_id == CAN_MCU_ADU_INVERTER_RIGHT_FRAME_ID){
            if (this->rosConf.publishMotorRPM)
                this->publish_motor_rpm();
            if (this->rosConf.publishInverterCommands)
                this->publish_inverter_commands();
            if (this->rosConf.publishInverterRightInfo) 
                this->publish_inverter_right_info();
         
        }
        else if (this->recvFrame.can_id == CAN_MCU_INVERTER_RIGHT_INFO_FRAME_ID){
            if (this->rosConf.publishInverterRightInfo) 
                this->publish_inverter_right_info(); 
        }
        else if (this->recvFrame.can_id == CAN_MCU_INVERTER_LEFT_INFO_FRAME_ID){
            if (this->rosConf.publishInverterLeftInfo) 
                this->publish_inverter_left_info(); 
        }
         else if (this->recvFrame.can_id == CAN_MCU_ISABELLEN_ENERGY_FRAME_ID || this->recvFrame.can_id== CAN_MCU_ISABELLEN_IDC_FRAME_ID || this->recvFrame.can_id== CAN_MCU_ISABELLEN_VDC_FRAME_ID || this->recvFrame.can_id== CAN_MCU_ISABELLEN_PDC_FRAME_ID) {
            this->publish_isabellen();
        }
               
        else if (this->recvFrame.can_id == CAN_MCU_ECU_PARAM_GENERAL_FRAME_ID && this->rosConf.publishECUParamGeneral) {
            this->publish_ecu_param_general();
        }

        else if (this->recvFrame.can_id == CAN_MCU_ECU_PARAM_CONTROL_FRAME_ID && this->rosConf.publishECUParamControl) {
            this->publish_ecu_param_control();
        }
         
        else if (this->recvFrame.can_id == CAN_MCU_ECU_ADU_FRAME_ID){
            if (this->rosConf.publishEcuControlSystems)
            this->publish_ecu_control_systems();
        }

        else if (this->recvFrame.can_id == CAN_MCU_BLDC_TX_2_FRAME_ID){
            if (this->rosConf.publishBLDC)
            this->publish_BLDC();
        }

        else if (this->recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_IMU_ACCEL_FRAME_ID || this->recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_IMU_GYRO_FRAME_ID){
            if(this->rosConf.publishSbgImu)
            this->publish_sbg_imu();
        }

        else if (this->recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_EKF_EULER_FRAME_ID){
            if(this->rosConf.publishSbgEkfEuler)
            this->publish_sbg_ekf_euler();
        }

        else if(this->recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_GPS1_VEL_FRAME_ID){ // add vel_acc frame id condition
            if(this->rosConf.publishSbgGpsVel)
            this->publish_sbg_gps_vel();
        }

        else if(this->recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_GPS1_POS_FRAME_ID){ // add pos_acc frame id condition and altitude
            if(this->rosConf.publishSbgGpsPos)
            this->publish_sbg_gps_pos();
        }

        else if(this->recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_EKF_VEL_BODY_FRAME_ID || this->recvFrame.can_id == CAN_MCU_SBG_ECAN_MSG_EKF_POS_FRAME_ID){ // add vel_body_acc and pos_lla_acc
            if(this->rosConf.publishSbgEkfNav)
            this->publish_sbg_ekf_nav();
        }

    }

    while (recvfrom(this->can0Socket, &this->recvFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, &this->len) >= 8) {
        ioctl(this->can0Socket, SIOCGSTAMP, &this->recvTime);  //get message timestamp
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

//Fucntions for CAN staus
void CanHandler::handleReceiveTimeout()
{
    rclcpp::Time timeNow = this->now();

    this->createHeader(&this->msgCanStatus.header);

    
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

//    if (timeNow - this->msgPumpsFans.header.stamp > rclcpp::Duration(1s))
  //      this->msgCanStatus.message_timeouts |= this->msgCanStatus.AUX_PUMPS_FANS_TIMEOUT; //TODO
  //  else
    //    this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.AUX_PUMPS_FANS_TIMEOUT; //TODO

    if (timeNow - this->msgDashBools.header.stamp > rclcpp::Duration(1s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.DASH_BOOLS_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.DASH_BOOLS_TIMEOUT;

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
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.INVERTER_ADU_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.INVERTER_ADU_TIMEOUT;
    
    if (timeNow - this->msgECUParamGeneral.header.stamp > rclcpp::Duration(2s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.ECU_PARAMS_ACTUAL_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.ECU_PARAMS_ACTUAL_TIMEOUT;

    if (timeNow - this->msgECUParamControl.header.stamp > rclcpp::Duration(2s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.ECU_PARAMS_ACTUAL_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.ECU_PARAMS_ACTUAL_TIMEOUT;
    
    if (timeNow - this->msgEcuControlSystems.header.stamp > rclcpp::Duration(1s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.ECU_CONTROL_SYSTEMS_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.ECU_CONTROL_SYSTEMS_TIMEOUT;

    if (timeNow - this->msgIsabellen.header.stamp > rclcpp::Duration(1s))
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.ISABELLEN_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.ISABELLEN_TIMEOUT;

    if (timeNow - this->msgInvLeftInfo.header.stamp > rclcpp::Duration(1s)) 
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.INV_LEFT_INFO_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.INV_LEFT_INFO_TIMEOUT;

    if (timeNow - this->msgInvRightInfo.header.stamp > rclcpp::Duration(1s)) 
        this->msgCanStatus.message_timeouts |= this->msgCanStatus.INV_RIGHT_INFO_TIMEOUT;
    else
        this->msgCanStatus.message_timeouts &= ~this->msgCanStatus.INV_RIGHT_INFO_TIMEOUT;

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

    this->pubCanStatus->publish(this->msgCanStatus);
}

//Functions for CAN transmit
void CanHandler::handleCanTransmit()
{
    this->canTimerCounter++;                //another 1ms passed
    if (this->canTimerCounter == 1001U)     //after 1sec
        this->canTimerCounter = 1U;         //reset counter to prevent overflow

    if ((this->rosConf.transmitApuStateMission == 2) && !(this->canTimerCounter % CAN_MCU_APU_STATE_MISSION_CYCLE_TIME_MS)) {
        this->transmit_apu_state_mission();
    }
    if ((this->rosConf.transmitSwaCommanded == 2) && !(this->canTimerCounter % CAN_MCU_APU_COMMAND_CYCLE_TIME_MS)) {
        this->transmit_steering_commanded();
    }
    if ((this->rosConf.transmitApuCommand == 2) && !(this->canTimerCounter % CAN_MCU_APU_COMMAND_CYCLE_TIME_MS)) {
        this->transmit_apu_command();
    }
    if ((this->rosConf.transmitECUParamAPU == 2) && !(this->canTimerCounter % CAN_MCU_ECU_PARAM_APU_CYCLE_TIME_MS)) {
        this->transmit_ecu_param_apu();
    }

    if (this->rosConf.transmitDvSystemStatus && !(this->canTimerCounter % CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_CYCLE_TIME_MS)) {
        this->transmit_dv_system_status();
    }
    if (this->rosConf.transmitApuResInit && !res_initialized && !(this->canTimerCounter % CAN_APU_RES_DLOGGER_APU_RES_INIT_CYCLE_TIME_MS)) {
        this->transmit_apu_res_init();
    }
}


//--------------------------------FOR DATALOGGER-------------------------------
void fill_datalogger_variables(can_apu_res_dlogger_dv_system_status_t *frameDvSystemStatus, can_mcu_apu_state_mission_t *frameApuStateMission, turtle_interfaces::msg::EbsSupervisorInfo *msgEbsSupervisor)
{
    switch (frameApuStateMission->as_state)
    {
    case CAN_MCU_APU_STATE_MISSION_AS_STATE_AS_OFF_CHOICE:
        frameDvSystemStatus->assi_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_ASSI_STATE_OFF_CHOICE;
        frameDvSystemStatus->steering_state = 0;
        break;
    case CAN_MCU_APU_STATE_MISSION_AS_STATE_AS_READY_CHOICE:
        frameDvSystemStatus->assi_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_ASSI_STATE_READY_CHOICE;
        frameDvSystemStatus->steering_state = 1;
        break;
    case CAN_MCU_APU_STATE_MISSION_AS_STATE_AS_DRIVING_CHOICE:
        frameDvSystemStatus->assi_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_ASSI_STATE_DRIVING_CHOICE;
        frameDvSystemStatus->steering_state = 1;
        break;
    case CAN_MCU_APU_STATE_MISSION_AS_STATE_AS_EMERGENCY_CHOICE:
        frameDvSystemStatus->assi_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_ASSI_STATE_EMERGENCY_BRAKE_CHOICE;
        frameDvSystemStatus->steering_state = 1;
        break;
    case CAN_MCU_APU_STATE_MISSION_AS_STATE_AS_FINISHED_CHOICE:
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
        case CAN_MCU_APU_STATE_MISSION_AS_MISSION_ACCELERATION_CHOICE:
            frameDvSystemStatus->ami_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_AMI_STATE_ACCELERATION_CHOICE;
            break;
        case CAN_MCU_APU_STATE_MISSION_AS_MISSION_SKIDPAD_CHOICE:
            frameDvSystemStatus->ami_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_AMI_STATE_SKIDPAD_CHOICE;
            break;
        case CAN_MCU_APU_STATE_MISSION_AS_MISSION_TRACKDRIVE_CHOICE:
            frameDvSystemStatus->ami_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_AMI_STATE_TRACKDRIVE_CHOICE;
            break;
        case CAN_MCU_APU_STATE_MISSION_AS_MISSION_EBSTEST_CHOICE:
            frameDvSystemStatus->ami_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_AMI_STATE_BRAKETEST_CHOICE;
            break;
        case CAN_MCU_APU_STATE_MISSION_AS_MISSION_INSPECTION_CHOICE:
            frameDvSystemStatus->ami_state = CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_AMI_STATE_INSPECTION_CHOICE;
            break;
        case CAN_MCU_APU_STATE_MISSION_AS_MISSION_AUTOCROSS_CHOICE:
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
         RCLCPP_ERROR(this->get_logger(), "Error during pack of DV_SYSTEM_STATUS");
        return;
    }

    if (sendto(this->can0Socket, &this->sendFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, this->len) < CAN_APU_RES_DLOGGER_DV_SYSTEM_STATUS_LENGTH) {
        RCLCPP_ERROR(this->get_logger(), "Error during transmit of DV_SYSTEM_STATUS");
    }
}

