#ifndef CAN_HANDLER_HPP
#define CAN_HANDLER_HPP

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <net/if.h>

#include "turtle_interfaces/msg/mission.hpp"
#include "turtle_interfaces/msg/brake_light.hpp"
#include "turtle_interfaces/msg/ebs_tank_pressure.hpp"
#include "turtle_interfaces/msg/rpm.hpp"
#include "turtle_interfaces/msg/tsal_safe_state.hpp"
#include "turtle_interfaces/msg/cooling_info.hpp"
#include "turtle_interfaces/msg/apps.hpp"
#include "turtle_interfaces/msg/rpm.hpp"
#include "turtle_interfaces/msg/brake.hpp"
#include "turtle_interfaces/msg/dash_leds.hpp"
#include "turtle_interfaces/msg/dash_buttons.hpp"
#include "turtle_interfaces/msg/ebs_supervisor_info.hpp"
#include "turtle_interfaces/msg/steering.hpp"

#include "can_as_dash_aux.h"

#define CAN_ERROR      -1
#define CAN_OK          0
#define CAN_NOT_READY   1
#define CAN_READY       2

typedef struct {
    //Channels configuration
    std::string channel0;
    uint32_t bitrate0;
    //CAN messages to publish in ROS
    bool publishAMI505;
    bool publishAux166;
    bool publishAux336;
    bool publishAux357;
    bool publishAux384;
    bool publishAux412;
    bool publishDash101;
    bool publishDash107;
    bool publishDash198;
    bool publishDash204;
    bool publishDash279;
    bool publishEBS550;
    bool publishSWA510;
    //CAN messages to transmite
    bool transmiteAPU660;
    bool transmiteSWA539;

} RosConfig;

class CanHandler : public rclcpp::Node
{
    private:
        int can0Socket;
        struct sockaddr_can addr0;
        struct ifreq ifr0;

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

        rclcpp::Publisher<turtle_interfaces::msg::Mission>::SharedPtr pubAMI505;
        turtle_interfaces::msg::Mission msgAMI505;
        void publish_ami_505();

        rclcpp::Publisher<turtle_interfaces::msg::BrakeLight>::SharedPtr pubAux166;
        turtle_interfaces::msg::BrakeLight msgAux166;
        void publish_aux_166();

        rclcpp::Publisher<turtle_interfaces::msg::EbsTankPressure>::SharedPtr pubAux336;
        turtle_interfaces::msg::EbsTankPressure msgAux336;
        void publish_aux_336();

        rclcpp::Publisher<turtle_interfaces::msg::RPM>::SharedPtr pubAux357;
        turtle_interfaces::msg::RPM msgAux357;
        void publish_aux_357();

        rclcpp::Publisher<turtle_interfaces::msg::TsalSafeState>::SharedPtr pubAux384;
        turtle_interfaces::msg::TsalSafeState msgAux384;
        void publish_aux_384();

        rclcpp::Publisher<turtle_interfaces::msg::CoolingInfo>::SharedPtr pubAux412;
        turtle_interfaces::msg::CoolingInfo msgAux412;
        void publish_aux_412();

        rclcpp::Publisher<turtle_interfaces::msg::Apps>::SharedPtr pubDash101;
        turtle_interfaces::msg::Apps msgDash101;
        void publish_dash_101();

        rclcpp::Publisher<turtle_interfaces::msg::RPM>::SharedPtr pubDash107;
        turtle_interfaces::msg::RPM msgDash107;
        void publish_dash_107();

        rclcpp::Publisher<turtle_interfaces::msg::Brake>::SharedPtr pubDash198;
        turtle_interfaces::msg::Brake msgDash198;
        void publish_dash_198();

        rclcpp::Publisher<turtle_interfaces::msg::DashLeds>::SharedPtr pubDash204;
        turtle_interfaces::msg::DashLeds msgDash204;
        void publish_dash_204();

        rclcpp::Publisher<turtle_interfaces::msg::DashButtons>::SharedPtr pubDash279;
        turtle_interfaces::msg::DashButtons msgDash279;
        void publish_dash_279();

        rclcpp::Publisher<turtle_interfaces::msg::EbsSupervisorInfo>::SharedPtr pubEBS550;
        turtle_interfaces::msg::EbsSupervisorInfo msgEBS550;
        void publish_ebs_550();

        rclcpp::Publisher<turtle_interfaces::msg::Steering>::SharedPtr pubSWA510;
        turtle_interfaces::msg::Steering msgSWA510;
        void publish_swa_510();

        //Variables and functions for CAN transmite
        uint16_t canTimerCounter;
        rclcpp::TimerBase::SharedPtr canSendTimer;        
        void handleCanTransmite();

        struct can_as_dash_aux_apu_660_t frameAPU660;
        void transmite_apu_660();

        struct can_as_dash_aux_swa_530_t frameSWA530;
        void transmite_swa_530();

    public:
        CanHandler(rclcpp::NodeOptions nOpt);
        ~CanHandler();
};

#endif