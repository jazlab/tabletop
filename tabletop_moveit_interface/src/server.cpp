#include <ctime>

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_trajectory/robot_trajectory.h"

#include "tabletop_moveit_interface/server.hpp"

using namespace std::chrono_literals;

namespace tabletop_moveit_interface{

	service::service(const std::string& name):
            Node(name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)){
        this->declare_parameter("eef_step", 0.06);
        this->declare_parameter("jump_threshold", 6.0);
        // this->declare_parameter("service_name", "tabletop_moveit_interface/goal_pose");
        
        eef_step = this->get_parameter("eef_step").as_double();
        jump_threshold = this->get_parameter("jump_threshold").as_double();
        service_name = this->get_parameter("service_name").as_string();

        ros_service = create_service<tabletop_msgs::srv::PlanRequest>(service_name, 
                std::bind(&service::callback, this, std::placeholders::_1, std::placeholders::_2));
    }

	void service::callback(const std::shared_ptr<tabletop_msgs::srv::PlanRequest::Request> request,
		std::shared_ptr<tabletop_msgs::srv::PlanRequest::Response> response){
		std::cout << "service: received" << std::endl;
        
        // Create a ROS logger
        auto const logger = rclcpp::get_logger("tabletop_moveit_interface_logger");

        // Create the MoveIt MoveGroup Interface
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group_interface = MoveGroupInterface(shared_from_this(), "ur_manipulator"); //recycle the server node as node for MoveGroupInterface
        // move_group_interface.setEndEffector("ee_link");        
        
        // Cartesian Path Planning
        waypoints.resize(1);
        waypoints[0] = (request->goal_pose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        double planning_result = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        // Execute the plan
        if(planning_result) {
            auto const execution_result = move_group_interface.execute(trajectory);     // execute the trajectory
            if (execution_result){
                response->success = true;    //return success to the client if the execution is successful
            }
            else{
                response->success = false;
                RCLCPP_ERROR(logger, "Execution failed!");
                }
        } 
        else {
            response->success = false;
            RCLCPP_ERROR(logger, "Planning failed!");
        }

		std::cout<< "service: done" << std::endl;
	}

}
