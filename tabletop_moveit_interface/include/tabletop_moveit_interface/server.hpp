#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tabletop_msgs/srv/plan_request.hpp>

namespace tabletop_moveit_interface{

	class service : public rclcpp::Node{
		private:
			std::vector<geometry_msgs::msg::Pose> waypoints;
			rclcpp::Service<tabletop_msgs::srv::PlanRequest>::SharedPtr ros_service;
			double eef_step;
			double jump_threshold;
			std::string service_name;
		public:
			service(const std::string& name);
			void callback(const std::shared_ptr<tabletop_msgs::srv::PlanRequest::Request> request, 
					std::shared_ptr<tabletop_msgs::srv::PlanRequest::Response> response);
	};

}



