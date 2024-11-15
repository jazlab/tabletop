#include <rclcpp/rclcpp.hpp>
#include <tabletop_msgs/srv/plan_request.hpp>

namespace tabletop_moveit{

	class service : public rclcpp::Node{
		private:
			std::vector<geometry_msgs::msg::Pose> waypoints;

			rclcpp::Service<tabletop_msgs::srv::PlanRequest>::SharedPtr ros_service;
		public:
			service(const std::string& name);
			void callback(const std::shared_ptr<tabletop_msgs::srv::PlanRequest::Request> request, 
					std::shared_ptr<tabletop_msgs::srv::PlanRequest::Response> response);
	};

}



