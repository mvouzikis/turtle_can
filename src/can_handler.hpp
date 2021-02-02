#ifndef CAN_HANDLER_HPP
#define CAN_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <net/if.h>

#include "turtle_interfaces/msg/can_ami505.hpp"
#include "turtle_interfaces/msg/can_aux166.hpp"
#include "turtle_interfaces/msg/can_aux336.hpp"
#include "turtle_interfaces/msg/can_aux357.hpp"
#include "turtle_interfaces/msg/can_aux384.hpp"
#include "turtle_interfaces/msg/can_aux412.hpp"
#include "turtle_interfaces/msg/can_dash101.hpp"
#include "turtle_interfaces/msg/can_dash107.hpp"
#include "turtle_interfaces/msg/can_dash198.hpp"
#include "turtle_interfaces/msg/can_dash204.hpp"
#include "turtle_interfaces/msg/can_dash279.hpp"
#include "turtle_interfaces/msg/can_ebs550.hpp"
#include "turtle_interfaces/msg/can_swa510.hpp"

#define CAN_ERROR      -1
#define CAN_OK          0
#define CAN_NOT_READY   1
#define CAN_READY       2

typedef struct {
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
} RosConfig;

class CanHandler : public rclcpp::Node
{
    private:
        int can0Socket;
        struct sockaddr_can addr0;
        struct ifreq ifr0;

        struct timeval recvTime;
        socklen_t len = sizeof(this->addr0);
        struct can_frame recvFrame;
        int handleCanReceive();

        RosConfig rosConf;
        void loadRosParams();
        void pubAndMsgInit();
        void createHeader(std_msgs::msg::Header *header);

        rclcpp::Publisher<turtle_interfaces::msg::CanAMI505>::SharedPtr pubAMI505;
        turtle_interfaces::msg::CanAMI505 msgAMI505;
        void publish_ami_505();

        rclcpp::Publisher<turtle_interfaces::msg::CanAux166>::SharedPtr pubAux166;
        turtle_interfaces::msg::CanAux166 msgAux166;
        void publish_aux_166();

        rclcpp::Publisher<turtle_interfaces::msg::CanAux336>::SharedPtr pubAux336;
        turtle_interfaces::msg::CanAux336 msgAux336;
        void publish_aux_336();

        rclcpp::Publisher<turtle_interfaces::msg::CanAux357>::SharedPtr pubAux357;
        turtle_interfaces::msg::CanAux357 msgAux357;
        void publish_aux_357();

        rclcpp::Publisher<turtle_interfaces::msg::CanAux384>::SharedPtr pubAux384;
        turtle_interfaces::msg::CanAux384 msgAux384;
        void publish_aux_384();

        rclcpp::Publisher<turtle_interfaces::msg::CanAux412>::SharedPtr pubAux412;
        turtle_interfaces::msg::CanAux412 msgAux412;
        void publish_aux_412();

        rclcpp::Publisher<turtle_interfaces::msg::CanDash101>::SharedPtr pubDash101;
        turtle_interfaces::msg::CanDash101 msgDash101;
        void publish_dash_101();

        rclcpp::Publisher<turtle_interfaces::msg::CanDash107>::SharedPtr pubDash107;
        turtle_interfaces::msg::CanDash107 msgDash107;
        void publish_dash_107();

        rclcpp::Publisher<turtle_interfaces::msg::CanDash198>::SharedPtr pubDash198;
        turtle_interfaces::msg::CanDash198 msgDash198;
        void publish_dash_198();

        rclcpp::Publisher<turtle_interfaces::msg::CanDash204>::SharedPtr pubDash204;
        turtle_interfaces::msg::CanDash204 msgDash204;
        void publish_dash_204();

        rclcpp::Publisher<turtle_interfaces::msg::CanDash279>::SharedPtr pubDash279;
        turtle_interfaces::msg::CanDash279 msgDash279;
        void publish_dash_279();

        rclcpp::Publisher<turtle_interfaces::msg::CanEBS550>::SharedPtr pubEBS550;
        turtle_interfaces::msg::CanEBS550 msgEBS550;
        void publish_ebs_550();

        rclcpp::Publisher<turtle_interfaces::msg::CanSWA510>::SharedPtr pubSWA510;
        turtle_interfaces::msg::CanSWA510 msgSWA510;
        void publish_swa_510();

    public:
        CanHandler(rclcpp::NodeOptions nOpt);
        ~CanHandler();
};

#endif