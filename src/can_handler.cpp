#include "can_handler.hpp"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/sockios.h>
#include <unistd.h>
#include <string>
#include "libsocketcan.h"
#include "can_as_dash_aux.h"

#define CHANNEL0    "can0"
#define CH0BITRATE  1000000 //channel0 bitrate


CanHandler::CanHandler(rclcpp::NodeOptions nOpt):Node("CanInterface", "", nOpt)
{
    //initialize can0
    if (can_do_stop(CHANNEL0) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to stop can0.");
        return;
    }
    if (can_set_bitrate(CHANNEL0, CH0BITRATE) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to set the bitrate on can0.");
        return;
    }
    if (can_do_start(CHANNEL0) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to start can0.");
        return;
    }    
    this->can0Socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->can0Socket < 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to create can0 socket.");
    }
    strcpy(this->ifr0.ifr_name, CHANNEL0);
    if (ioctl(this->can0Socket, SIOCGIFINDEX, &this->ifr0) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Unable to perform ioctl for can0.\n");
    }
    this->addr0.can_family = AF_CAN;
    this->addr0.can_ifindex = this->ifr0.ifr_ifindex;
    if (bind(this->can0Socket, (struct sockaddr*)&this->addr0, sizeof(this->addr0)) < 0){
        RCLCPP_ERROR(this->get_logger(), "Unable to bind socket with can0.\n");
    }

    this->loadRosParams();
    this->pubAndMsgInit();

    RCLCPP_INFO(this->get_logger(), "Can interface initialized");

    while (1) {
        if (this->handleCanReceive() == CAN_NOT_READY) {
            usleep(500);    //free some CPU waiting new messages to come
        }
    }
}

CanHandler::~CanHandler()
{
    if (close(this->can0Socket) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to close can0.\n");
    }
}

void CanHandler::loadRosParams()
{
    this->get_parameter_or<bool>("publishAux357", this->rosConf.publishAux357, true);
    this->get_parameter_or<bool>("publishAux384", this->rosConf.publishAux384, true);
    this->get_parameter_or<bool>("publishDash101", this->rosConf.publishDash101, true);
    this->get_parameter_or<bool>("publishDash107", this->rosConf.publishDash107, true);
    this->get_parameter_or<bool>("publishDash198", this->rosConf.publishDash198, true);
    this->get_parameter_or<bool>("publishDash279", this->rosConf.publishDash279, true);
}

void CanHandler::pubAndMsgInit()
{
    if (this->rosConf.publishAux357) {
        this->pubAux357 = this->create_publisher<turtle_interfaces::msg::CanAux357>("can_dash_357", 10);
        this->msgAux357 = turtle_interfaces::msg::CanAux357();
    }
    if (this->rosConf.publishAux384) {
        this->pubAux384 = this->create_publisher<turtle_interfaces::msg::CanAux384>("can_dash_384", 10);
        this->msgAux384 = turtle_interfaces::msg::CanAux384();
    }
    if (this->rosConf.publishDash101) {
        this->pubDash101 = this->create_publisher<turtle_interfaces::msg::CanDash101>("can_dash_101", 10);
        this->msgDash101 = turtle_interfaces::msg::CanDash101();
    }
    if (this->rosConf.publishDash107) {
        this->pubDash107 = this->create_publisher<turtle_interfaces::msg::CanDash107>("can_dash_107", 10);
        this->msgDash107 = turtle_interfaces::msg::CanDash107();
    }
    if (this->rosConf.publishDash198) {
        this->pubDash198 = this->create_publisher<turtle_interfaces::msg::CanDash198>("can_dash_198", 10);
        this->msgDash198 = turtle_interfaces::msg::CanDash198();
    }
    if (this->rosConf.publishDash279) {
        this->pubDash279 = this->create_publisher<turtle_interfaces::msg::CanDash279>("can_dash_279", 10);
        this->msgDash279 = turtle_interfaces::msg::CanDash279();
    }    
}

int CanHandler::handleCanReceive()
{
    if (recvfrom(this->can0Socket, &this->recvFrame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&this->addr0, &this->len) >= 8) {
        ioctl(this->can0Socket, SIOCGSTAMP, &this->recvTime);  //get message timestamp

        if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AUX_357_FRAME_ID && this->rosConf.publishAux357) {
            this->publish_aux_357();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_AUX_384_FRAME_ID && this->rosConf.publishAux384) {
            this->publish_aux_384();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_DASH_101_FRAME_ID && this->rosConf.publishDash101) {
            this->publish_dash_101();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_DASH_107_FRAME_ID && this->rosConf.publishDash107) {
            this->publish_dash_107();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_DASH_198_FRAME_ID && this->rosConf.publishDash198) {
            this->publish_dash_198();
        }
        else if (this->recvFrame.can_id == CAN_AS_DASH_AUX_DASH_279_FRAME_ID && this->rosConf.publishDash279) {
            this->publish_dash_279();
        }

        return CAN_READY;
    }

    return CAN_NOT_READY;
}

void CanHandler::createHeader(std_msgs::msg::Header *header)
{
    header->frame_id = "";
    header->stamp.sec = this->recvTime.tv_sec;
    header->stamp.nanosec = this->recvTime.tv_usec*1000;
}

void CanHandler::publish_aux_357()
{
    can_as_dash_aux_aux_357_t msg;
    if (can_as_dash_aux_aux_357_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_357");
        return;
    }

    this->createHeader(&this->msgAux357.header);
    this->msgAux357.hallrl = msg.hall_rl;
    this->msgAux357.hallrr = msg.hall_rr;

    this->pubAux357->publish(this->msgAux357);
}

void CanHandler::publish_aux_384()
{
    can_as_dash_aux_aux_384_t msg;
    if (can_as_dash_aux_aux_384_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of AUX_384");
        return;
    }

    this->createHeader(&this->msgAux384.header);
    this->msgAux384.safestate = msg.safe_state;

    this->pubAux357->publish(this->msgAux357);
}

void CanHandler::publish_dash_101()
{
    can_as_dash_aux_dash_101_t msg;
    if (can_as_dash_aux_dash_101_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_101");
        return;
    }

    this->createHeader(&this->msgDash101.header);
    this->msgDash101.apps1raw = msg.apps1_raw;
    this->msgDash101.apps2raw = msg.apps2_raw;

    this->pubDash101->publish(this->msgDash101);
}

void CanHandler::publish_dash_107()
{
    can_as_dash_aux_dash_107_t msg;
    if (can_as_dash_aux_dash_107_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_107");
        return;
    }

    this->createHeader(&this->msgDash107.header);
    this->msgDash107.hallfl = msg.hall_fl;
    this->msgDash107.hallfr = msg.hall_fr;

    this->pubDash107->publish(this->msgDash107);
}

void CanHandler::publish_dash_198()
{
    can_as_dash_aux_dash_198_t msg;
    if (can_as_dash_aux_dash_198_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_198");
        return;
    }

    this->createHeader(&this->msgDash198.header);
    this->msgDash198.brakeraw = msg.brake_raw;

    this->pubDash198->publish(this->msgDash198);
}

void CanHandler::publish_dash_279()
{
    can_as_dash_aux_dash_279_t msg;
    if (can_as_dash_aux_dash_279_unpack(&msg, this->recvFrame.data, this->recvFrame.can_dlc) != CAN_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error during unpack of DASH_279");
        return;
    }

    this->createHeader(&this->msgDash279.header);
    this->msgDash279.enabletoggle = msg.enable_toggle;
    this->msgDash279.secondtoggle = msg.second_toggle;
    this->msgDash279.thirdtogglenotused = msg.third_toggle_notused;
    this->msgDash279.start = msg.start;
    this->msgDash279.adact = msg.ad_act;
    this->msgDash279.greentsal = msg.green_tsal;
    this->msgDash279.scstate = msg.sc_state;

    this->pubDash279->publish(this->msgDash279);
}
