#ifndef CAN_HANDLER_HPP
#define CAN_HANDLER_HPP

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <net/if.h>

#include "turtle_interfaces/msg/mission.hpp"
#include "turtle_interfaces/msg/brake_light.hpp"
#include "turtle_interfaces/msg/ebs_tank_pressure.hpp"
#include "turtle_interfaces/msg/tsal_safe_state.hpp"
#include "turtle_interfaces/msg/cooling_info.hpp"
#include "turtle_interfaces/msg/apps.hpp"
#include "turtle_interfaces/msg/rpm.hpp"
#include "turtle_interfaces/msg/brake.hpp"
#include "turtle_interfaces/msg/dash_bools.hpp"
#include "turtle_interfaces/msg/dash_buttons.hpp"
#include "turtle_interfaces/msg/ebs_service_brake.hpp"
#include "turtle_interfaces/msg/ebs_supervisor_info.hpp"
#include "turtle_interfaces/msg/res_status.hpp"
#include "turtle_interfaces/msg/steering.hpp"
#include "turtle_interfaces/msg/state_machine_state.hpp"
#include "turtle_interfaces/msg/actuator_cmd.hpp"
#include "turtle_interfaces/msg/inverter_commands.hpp"
#include "turtle_interfaces/msg/ecu_params.hpp"
#include "turtle_interfaces/msg/can_status.hpp"
#include "turtle_interfaces/msg/control_info.hpp"
#include "turtle_interfaces/msg/slam_info.hpp"
#include "turtle_interfaces/msg/inverter_info.hpp"
#include "turtle_interfaces/msg/isabellen.hpp"
#include "turtle_interfaces/msg/ecu_control_systems.hpp"
#include "turtle_interfaces/msg/gpu_status.hpp"
#include "turtle_interfaces/msg/cpu_status.hpp"
#include "turtle_interfaces/msg/mission_status.hpp"





#include "can_mcu.h"
#include "can_apu_res_dlogger.h"


#define CAN_ERROR      -1
#define CAN_OK          0
#define CAN_NOT_READY   1
#define CAN_READY       2

typedef struct {
    //Channels configuration
    std::string channel0;
    uint32_t bitrate0;
    std::string channel1;
    uint32_t bitrate1;
    //CAN messages to publish in ROS
    bool publishDashApps;
    bool publishDashBrake;
    bool publishDashButtons;
    bool pubishDashFrontRPM;
    bool publishAuxTsalSafeState;
    bool publishPumpsFans;
    bool publishAuxBrakelight;
    bool publishDashBools;
    bool publishEbsTankPressure;
    bool publishAmiSelectedMission;
    bool publishSwaStatus;
    bool publishEbsServiceBrake;
    bool publishEbsSupervisor;
    bool publishMotorRPM;
    bool publishInverterCommands;
    bool publishResStatus;
    bool publishCanStatus;
    bool publishECUParamsActaul;
    bool publishInverterRightInfo;
    bool publishInverterLeftInfo;
    bool publishIsabellen;
    bool publishEcuControlSystems;
    bool publishCoolingInfo;


    //CAN messages to transmit
    uint8_t transmitApuStateMission;
    uint8_t transmitSwaCommanded;
    uint8_t transmitApuCommand;
    uint8_t transmitECUParams;
    uint8_t transmitECUParams2;
    bool transmitDvSystemStatus;
    bool transmitApuResInit;
    uint8_t transmitApuTemp;
    uint8_t transmitSetFinished;


} RosConfig;

class CanHandler : public rclcpp::Node
{
    private:
        int can0Socket;
        struct sockaddr_can addr0;
        struct ifreq ifr0;

        int can1Socket;
        struct sockaddr_can addr1;
        struct ifreq ifr1;

        struct timeval recvTime;
        socklen_t len = sizeof(this->addr0);
        struct can_frame recvFrame, sendFrame;

        RosConfig rosConf;
        void loadRosParams();
        void variablesInit();
        void createHeader(std_msgs::msg::Header *header);

        //Variables and functions for CAN receive
        rclcpp::TimerBase::SharedPtr canRecvTimer;
        void handleCanReceive();

        //channel 0
        rclcpp::Publisher<turtle_interfaces::msg::Apps>::SharedPtr pubDashApps;
        turtle_interfaces::msg::Apps msgDashApps;
        void publish_dash_apps();

        rclcpp::Publisher<turtle_interfaces::msg::Brake>::SharedPtr pubDashBrake;
        turtle_interfaces::msg::Brake msgDashBrake;
        void publish_dash_brake();

        rclcpp::Publisher<turtle_interfaces::msg::DashButtons>::SharedPtr pubDashButtons;
        turtle_interfaces::msg::DashButtons msgDashButtons;
        void publish_dash_buttons();

        rclcpp::Publisher<turtle_interfaces::msg::RPM>::SharedPtr pubDashFrontRPM;
        turtle_interfaces::msg::RPM msgDashFrontRPM  ;
        void publish_dash_front_rpm();

        rclcpp::Publisher<turtle_interfaces::msg::RPM>::SharedPtr pubMotorRPM;
        turtle_interfaces::msg::RPM msgMotorRPM;
        void publish_motor_rpm();

        rclcpp::Publisher<turtle_interfaces::msg::InverterCommands>::SharedPtr pubInvCmds;
        turtle_interfaces::msg::InverterCommands msgInvCmds;
        void publish_inverter_commands();

        rclcpp::Publisher<turtle_interfaces::msg::InverterInfo>::SharedPtr pubInvRightInfo;
        turtle_interfaces::msg::InverterInfo msgInvRightInfo;
        void publish_inverter_right_info();

        rclcpp::Publisher<turtle_interfaces::msg::InverterInfo>::SharedPtr pubInvLeftInfo;
        turtle_interfaces::msg::InverterInfo msgInvLeftInfo;
        void publish_inverter_left_info();

        rclcpp::Publisher<turtle_interfaces::msg::TsalSafeState>::SharedPtr pubAuxTsalSafeState;
        turtle_interfaces::msg::TsalSafeState msgAuxTsalSafeState;
        void publish_aux_tsal_safe_state();

        rclcpp::Publisher<turtle_interfaces::msg::BrakeLight>::SharedPtr pubAuxBrakelight;
        turtle_interfaces::msg::BrakeLight msgAuxBrakelight;
        void publish_aux_brakelight();

        rclcpp::Publisher<turtle_interfaces::msg::DashBools>::SharedPtr pubDashBools;
        turtle_interfaces::msg::DashBools msgDashBools;
        void publish_dash_bools();

        rclcpp::Publisher<turtle_interfaces::msg::EbsTankPressure>::SharedPtr pubEbsTankPressure;
        turtle_interfaces::msg::EbsTankPressure msgEbsTankPressure;
        void publish_ebs_tank_pressure();

        rclcpp::Publisher<turtle_interfaces::msg::Mission>::SharedPtr pubAmiSelectedMission;
        turtle_interfaces::msg::Mission msgAmiSelectedMission;
        void publish_ami_selected_mission();

        rclcpp::Publisher<turtle_interfaces::msg::Steering>::SharedPtr pubSwaActual;
        turtle_interfaces::msg::Steering msgSwaActual;
        void publish_swa_actual();

        rclcpp::Publisher<turtle_interfaces::msg::EbsServiceBrake>::SharedPtr pubEbsServiceBrake;
        turtle_interfaces::msg::EbsServiceBrake msgEbsServiceBrake;
        void publish_ebs_service_brake();

        rclcpp::Publisher<turtle_interfaces::msg::EbsSupervisorInfo>::SharedPtr pubEbsSupervisor;
        turtle_interfaces::msg::EbsSupervisorInfo msgEbsSupervisor;
        void publish_ebs_supervisor();

        rclcpp::Publisher<turtle_interfaces::msg::ECUParams>::SharedPtr pubEcuParams;
        turtle_interfaces::msg::ECUParams msgECUParamsActual;
        void publish_ecu_params_actual();
    
        rclcpp::Publisher<turtle_interfaces::msg::Isabellen>::SharedPtr pubIsabellen;
        turtle_interfaces::msg::Isabellen msgIsabellen;
        void publish_isabellen();

        rclcpp::Publisher<turtle_interfaces::msg::ECUControlSystems>::SharedPtr pubEcuControlSystem;
        turtle_interfaces::msg::ECUControlSystems msgEcuControlSystems;
        void publish_ecu_control_systems();

        rclcpp::Publisher<turtle_interfaces::msg::CoolingInfo>::SharedPtr pubCoolingInfo;
        turtle_interfaces::msg::CoolingInfo msgCoolingInfo;
        void publish_cooling_info();

        
        
        //channel 1
        rclcpp::Publisher<turtle_interfaces::msg::ResStatus>::SharedPtr pubResStatus;
        turtle_interfaces::msg::ResStatus msgResStatus;
        void publish_res_status();

        

        //Variables and functions for CAN errors
        rclcpp::TimerBase::SharedPtr canRecvTimeout;
        void handleReceiveTimeout();
        
        rclcpp::Publisher<turtle_interfaces::msg::CanStatus>::SharedPtr pubCanStatus;
        turtle_interfaces::msg::CanStatus msgCanStatus;
        void publish_can_status();


        //Variables and functions for CAN transmit
        uint16_t canTimerCounter;
        rclcpp::TimerBase::SharedPtr canSendTimer;        
        void handleCanTransmit();

        //channel 0
        rclcpp::Subscription<turtle_interfaces::msg::StateMachineState>::SharedPtr subApuState;
        void apu_state_callback(turtle_interfaces::msg::StateMachineState::SharedPtr msgApuState);

        rclcpp::Subscription<turtle_interfaces::msg::Mission>::SharedPtr subApuMission;                
        void apu_mission_callback(turtle_interfaces::msg::Mission::SharedPtr msgApuMission);

        rclcpp::Subscription<turtle_interfaces::msg::MissionStatus>::SharedPtr subMissionStatus;                
        void apu_set_finished_callback(turtle_interfaces::msg::MissionStatus::SharedPtr msgMissionStatus);
        
        rclcpp::Subscription<turtle_interfaces::msg::ActuatorCmd>::SharedPtr subActuatorCmd;        
        void actuator_cmd_callback(turtle_interfaces::msg::ActuatorCmd::SharedPtr msgActuatorCmd);

        rclcpp::Subscription<turtle_interfaces::msg::ECUParams>::SharedPtr subECUParams;        
        void ecu_params_callback(turtle_interfaces::msg::ECUParams::SharedPtr msgECUParams);

        rclcpp::Subscription<turtle_interfaces::msg::CpuStatus>::SharedPtr subCPUTemps;        
        void cpu_temps_callback(turtle_interfaces::msg::CpuStatus::SharedPtr msgCPUTemps);

        rclcpp::Subscription<turtle_interfaces::msg::GpuStatus>::SharedPtr subGPUTemp;        
        void gpu_temp_callback(turtle_interfaces::msg::GpuStatus::SharedPtr msgGPUTemp);


        struct can_mcu_apu_state_mission_t frameApuStateMission;
        void transmit_apu_state_mission();

        struct can_mcu_steering_command_t frameSwaCommanded;
        void transmit_steering_commanded();

        struct can_mcu_apu_command_t frameApuCommand;
        void transmit_apu_command();

        struct can_mcu_ecu_parameters_t frameECUParams; 
        void transmit_ecu_params();

        struct can_mcu_apu_temps_t frameAPUTemps; 
        void transmit_apu_temps();

        // //channel 1
        rclcpp::Subscription<turtle_interfaces::msg::ControlInfo>::SharedPtr subControlInfo;
        void control_info_callback(turtle_interfaces::msg::ControlInfo::SharedPtr msgControlInfo);

        rclcpp::Subscription<turtle_interfaces::msg::SlamInfo>::SharedPtr subSlamInfo;
        void slam_info_callback(turtle_interfaces::msg::SlamInfo::SharedPtr msgSlamInfo);

        struct can_apu_res_dlogger_dv_system_status_t frameDvSystemStatus;
        void transmit_dv_system_status();

        // Send RES initialize message unitl it starts sending CAN messages
        struct can_apu_res_dlogger_apu_res_init_t frameApuResInit;
        bool res_initialized;
        void transmit_apu_res_init();

    public:
        CanHandler(rclcpp::NodeOptions nOpt);
        ~CanHandler();
};

#endif
