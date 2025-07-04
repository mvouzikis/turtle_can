#ifndef LOAD_ROS_PARAMS_HPP
#define LOAD_ROS_PARAMS_HPP

#include "can_handler.hpp"


void CanHandler::loadRosParams()
{
    //Channels configuration
    this->get_parameter_or<std::string>("channel0", this->rosConf.channel0, "can0");
    this->get_parameter_or<uint32_t>("bitrate0",    this->rosConf.bitrate0, 500000);
    
    //CAN messages to publish in ROS
    this->get_parameter_or<bool>("publishDashApps",             this->rosConf.publishDashApps,              true);
    this->get_parameter_or<bool>("publishDashBrake",            this->rosConf.publishDashBrake,             true);
    this->get_parameter_or<bool>("publishDashButtons",          this->rosConf.publishDashButtons,           true);
    this->get_parameter_or<bool>("pubishDashFrontRPM",          this->rosConf.pubishDashFrontRPM,           true);
    this->get_parameter_or<bool>("publishAuxTsalSafeState",     this->rosConf.publishAuxTsalSafeState,      true);
    this->get_parameter_or<bool>("publishAuxBrakelight",        this->rosConf.publishAuxBrakelight,         true);
    this->get_parameter_or<bool>("publishDashBools",            this->rosConf.publishDashBools,             true);
    this->get_parameter_or<bool>("publishEbsTankPressure",      this->rosConf.publishEbsTankPressure,       true);
    this->get_parameter_or<bool>("publishAmiSelectedMission",   this->rosConf.publishAmiSelectedMission,    true);
    this->get_parameter_or<bool>("publishSwaActual",            this->rosConf.publishSwaStatus,             true);
    this->get_parameter_or<bool>("publishEbsServiceBrake",      this->rosConf.publishEbsServiceBrake,       true);
    this->get_parameter_or<bool>("publishEbsSupervisor",        this->rosConf.publishEbsSupervisor,         true);
    this->get_parameter_or<bool>("publishMotorRPM",             this->rosConf.publishMotorRPM,              true);
    this->get_parameter_or<bool>("publishResStatus",            this->rosConf.publishResStatus,             true);
    this->get_parameter_or<bool>("publishCanStatus",            this->rosConf.publishCanStatus,             true);
    this->get_parameter_or<bool>("publishECUParamGeneral",      this->rosConf.publishECUParamGeneral,       true);
    this->get_parameter_or<bool>("publishECUParamControl",      this->rosConf.publishECUParamControl,       true);
    this->get_parameter_or<bool>("publishInverterRightInfo",    this->rosConf.publishInverterRightInfo,     true);
    this->get_parameter_or<bool>("publishInverterLeftInfo",     this->rosConf.publishInverterLeftInfo,      true);
    this->get_parameter_or<bool>("publishIsabellen",            this->rosConf.publishIsabellen,             true);
    this->get_parameter_or<bool>("publishECUControlSystems",    this->rosConf.publishEcuControlSystems,     true);
    this->get_parameter_or<bool>("publishpublishCoolingInfo",   this->rosConf.publishCoolingInfo,           true);
    this->get_parameter_or<bool>("publishBLDC"              ,   this->rosConf.publishBLDC,                  true);

    this->get_parameter_or<bool>("publishSbgImu",   this->rosConf.publishSbgImu,           true);
    this->get_parameter_or<bool>("publishSbgEkfEuler", this->rosConf.publishSbgEkfEuler,           true);
    this->get_parameter_or<bool>("publishSbgGpsVel",   this->rosConf.publishSbgGpsVel,           true);
    this->get_parameter_or<bool>("publishSbgGpsPos",   this->rosConf.publishSbgGpsPos,           true);
    this->get_parameter_or<bool>("publishSbgEkfNav",   this->rosConf.publishSbgEkfNav,           true);


    //CAN messages to transmit
    this->get_parameter_or<uint8_t>("transmitApuStateMission", this->rosConf.transmitApuStateMission,   2);
    this->get_parameter_or<uint8_t>("transmitSwaCommanded",    this->rosConf.transmitSwaCommanded,      2);
    this->get_parameter_or<uint8_t>("transmitApuCommand",      this->rosConf.transmitApuCommand,        2);
    this->get_parameter_or<uint8_t>("transmitECUParamAPU",     this->rosConf.transmitECUParamAPU,       1);
    this->get_parameter_or<bool>("transmitDvSystemStatus",     this->rosConf.transmitDvSystemStatus,    true);
    this->get_parameter_or<bool>("transmitApuResInit",         this->rosConf.transmitApuResInit,        true);
    this->get_parameter_or<uint8_t>("transmitApuTemp",         this->rosConf.transmitApuTemp,           1);
    this->get_parameter_or<uint8_t>("transmitApuEstimation",         this->rosConf.transmitApuEstimation,           2);
    this->get_parameter_or<uint8_t>("transmitDrivingDynamics1", this->rosConf.transmitDvDrivingDynamics1, 2);
    this->get_parameter_or<uint8_t>("transmitDrivingDynamics2", this->rosConf.transmitDvDrivingDynamics2, 2);


}

#endif