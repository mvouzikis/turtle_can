#include <signal.h>
#include <unistd.h>
#include "can_handler.hpp"

void terminate_signal(int sig)
{
	if (sig == SIGINT) {
		rclcpp::shutdown();
	}
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	signal(SIGINT, terminate_signal);	//change terminate signal
	rclcpp::NodeOptions nodeOptions;
	nodeOptions.automatically_declare_parameters_from_overrides(true);
	rclcpp::spin(std::make_shared<CanHandler>(nodeOptions));
	rclcpp::shutdown();
	return 0;
}
