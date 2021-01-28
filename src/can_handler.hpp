#ifndef CAN_HANDLER_HPP
#define CAN_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <net/if.h>

#include "turtle_interfaces/msg/can_aux357.hpp"
#include "turtle_interfaces/msg/can_aux384.hpp"
#include "turtle_interfaces/msg/can_dash101.hpp"
#include "turtle_interfaces/msg/can_dash107.hpp"
#include "turtle_interfaces/msg/can_dash198.hpp"
#include "turtle_interfaces/msg/can_dash279.hpp"

#define CAN_ERROR      -1
#define CAN_NOT_READY   1
#define CAN_READY       0
#define CAN_OK          2

typedef struct {
    bool publishAux357;
    bool publishAux384;
    bool publishDash101;
    bool publishDash107;
    bool publishDash198;
    bool publishDash279;    
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

        rclcpp::Publisher<turtle_interfaces::msg::CanAux357>::SharedPtr pubAux357;
        turtle_interfaces::msg::CanAux357 msgAux357;
        void publish_aux_357();

        rclcpp::Publisher<turtle_interfaces::msg::CanAux384>::SharedPtr pubAux384;
        turtle_interfaces::msg::CanAux384 msgAux384;
        void publish_aux_384();

        rclcpp::Publisher<turtle_interfaces::msg::CanDash101>::SharedPtr pubDash101;
        turtle_interfaces::msg::CanDash101 msgDash101;
        void publish_dash_101();

        rclcpp::Publisher<turtle_interfaces::msg::CanDash107>::SharedPtr pubDash107;
        turtle_interfaces::msg::CanDash107 msgDash107;
        void publish_dash_107();

        rclcpp::Publisher<turtle_interfaces::msg::CanDash198>::SharedPtr pubDash198;
        turtle_interfaces::msg::CanDash198 msgDash198;
        void publish_dash_198();

        rclcpp::Publisher<turtle_interfaces::msg::CanDash279>::SharedPtr pubDash279;
        turtle_interfaces::msg::CanDash279 msgDash279;
        void publish_dash_279();

    public:
        CanHandler(rclcpp::NodeOptions nOpt);
        ~CanHandler();
};

#endif