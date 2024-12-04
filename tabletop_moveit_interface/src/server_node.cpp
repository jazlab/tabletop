#include <tabletop_moveit_interface/server.hpp>

int main(int argc, char** argv){
	rclcpp::init(argc, argv);

    std::shared_ptr<tabletop_moveit_interface::service> moveit_server = std::make_shared<tabletop_moveit_interface::service>("moveit_interface_server");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(moveit_server);
    executor.spin();
    rclcpp::shutdown();

	return 0;
}