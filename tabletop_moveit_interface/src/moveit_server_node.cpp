#include <tabletop_msgs/srv/plan_request.hpp>
#include <tabletop_moveit_interface/moveit_pose_server.hpp>

int main(int argc, char** argv){
	rclcpp::init(argc, argv);

    std::shared_ptr<tabletop_moveit::service> moveit_server = std::make_shared<tabletop_moveit::service>("moveit_interface_server");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(moveit_server);
    executor.spin();
    rclcpp::shutdown();

	return 0;
}



