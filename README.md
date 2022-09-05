# turtle_can

This package is for the communication of the APU with the rest of the vehicle via CAN BUS.

## Description 

In order to connect the APU to the CAN we use the *Kvaser Mini PCI Express 2xHS v2*. It's a device which offers two CAN channels, the one (can0) is used for the 'MCU' channel and the other for the 'RES' channel. For its use we are using the *SocketCAN* which is basically CAN drivers. You can read more about it [here](https://docs.kernel.org/networking/can.html). For the device to start working is necessary to set a bitrate and start it using the 'ip link set canX up/down' command, for us this is done at the *Zygote* which is running when you boot the APU. ![](https://i.imgur.com/ZPAo0p2.png)

## DBC Files

In order to know the messages that the two channels carry we are using DBC files. You can read more about them and explore ours at the respectively  repository [https://gitlab.com/aristurtle/lv/dbc_files](https://gitlab.com/aristurtle/lv/dbc_files). To use the data from inthere you need to generate the .c .h file using cantools. Ιnstructions for how to use it and how to generate the source code [here](https://cantools.readthedocs.io/en/latest/). 

example: by running the command `cantools generate_c_source dbc_files/CAN_MCU.dbc` you generate .c and .h file which have to be included to the /src file of this repository. 

The *cantools* it's very handy as it can do the decode/encode of the messages as well as the packing/unpacking of them.

## CAN Handler

can_handler is responsible for:
- The configuration of the two channels.
- The handling of the received messages by checking constantly what message is received based on the FRAME_ID in order to then use the right publisher to publish the data (CAN BUS -> ROS)
- Checking for CAN timeouts, errors, state and statistics of the channel in order to publish them at the /can_status topic
- Calling the transmit functions if the tranmition is selected to be periodically based on the period which is determined in the dbc file
- Filling the datalogger variables based on the FSEATS handbook and transmit the can message on the RES Channel (can1)

## Variables init

Creating the publishers and the subscriber and initializing the variables of the CAN messages with *default* values


## Callbacks

Includes all the callbacks. Each topic that the turtle_can has created a sub is corresponding to can messages (or more), so each variable of the CAN message have to has a corresponding variable from a ROS topic. 


*example*: Here is the callback of the /state_flowchart_state (as you can see and at the variables_init.hpp). The ROS message has two variables (you can find them by looking the message file at the [https://gitlab.com/aristurtle/dv/turtle_interfaces](https://gitlab.com/aristurtle/dv/turtle_interfaces)). The variable *as_state* from ROS is the corresponding as_state for the CAN message AS_State of the CAN FRAME ID= 0x00a.

![](https://i.imgur.com/nzUSpHf.png)


## CAN Transmit

Take the variables that we have create at the callbacks for the CAN (frameXX) and pack with the right format using the cantools function pack(). Also corresponds the right FRAME ID and DLC to each message and checks if the procedure completed successfully. At the end it sends the message based on the info of the DBC file and checks it transmitted successfully. 

## CAN Receive

Unpacks the CAN messages to variables for the ROS and calls the publish function.
