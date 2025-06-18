/* ** *****************************************************************
* dogbase.cpp
*
* Forwards a Twist to the sport_client and broadcasts the 
* transform as odometry
*
* Author:  Mauricio Matamoros
* License: MIT
** ** ****************************************************************/
#include <chrono>
#include <memory>
#include <thread>
#include <cstdint>
#include <algorithm>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "sport_client/sport_client.h"
// #include <unitree_api/msg/response.hpp>


using Request          = unitree_api::msg::Request;
// using Response         = unitree_api::msg::Response;
// using ResponsePtr      = std::shared_ptr<Response>;
using Twist            = geometry_msgs::msg::Twist;
using TwistPtr         = std::shared_ptr<Twist>;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Odometry         = nav_msgs::msg::Odometry;
using OdometryPtr      = std::shared_ptr<Odometry>;
using SportClientPtr   = std::shared_ptr<SportClient>;


int main(int argc, char **argv);

class DogBaseNode : public rclcpp::Node{
	private:
		SportClientPtr sc;
		rclcpp::Publisher<Request>::SharedPtr pub_api;
		rclcpp::Publisher<Odometry>::SharedPtr pub_odom;
		rclcpp::Subscription<Twist>::SharedPtr sub_cmd_vel;
		// rclcpp::Subscription<Response>::SharedPtr sub_response;
		std::unique_ptr<tf2_ros::TransformBroadcaster> tbc;
		ros::Time start;

	public:
		DogBaseNode();

	private:
		void handleTwist(const TwistPtr msg);
		void publishOdometry(const TwistPtr msg, const tf2::Quaternion& q);
		void broadcastTransform(const TwistPtr msg, const tf2::Quaternion& q);
		std::chrono::duration<double> getElapsed();
		 // = std::chrono::high_resolution_clock::now();
		// void handleResponse(const ResponsePtr msg);
};



int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DogBaseNode>());
	rclcpp::shutdown();
	return 0;
}



DogBaseNode::DogBaseNode():
	Node("dog_base_node"){
	using namespace std::literals;
	start = std::chrono::system_clock::now() + 24h;
	tbc = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	pub_api = this->create_publisher<Request>("/api/sport/request", 10);
	pub_odom = this->create_publisher<Odometry>("/odom", 10);
	// sub_response = this->create_subscription<Response>("/api/sport/response", 10,
	//	std::bind(&DogBaseNode::handleResponse, this, std::placeholders::_1)
	//);
	sub_cmd_vel = this->create_subscription<Twist>("/cmd_vel", 10,
		std::bind(&DogBaseNode::handleTwist, this, std::placeholders::_1)
	);
	sc  = std::make_shared<SportClient>(pub_api);

	RCLCPP_INFO(this->get_logger(), "Dogbase node running. Standing up...");
	sc->StandDown();
	std::this_thread::sleep_for(std::chrono::seconds(2));
	sc->StandUp();
	std::this_thread::sleep_for(std::chrono::seconds(2));
	sc->BalanceStand();
	RCLCPP_INFO(this->get_logger(), "Dogbase ready");
}


void DogBaseNode::handleTwist(const TwistPtr msg){
	auto elapsed = getElapsed();
	// RCLCPP_INFO(this->get_logger(),
	// 	"Handle twist: (%0.2f,%0.2f,%0.2f) (%0.2f,%0.2f,%0.2f)",
	// 	msg->linear.x,  msg->linear.y,  msg->linear.z,
	// 	msg->angular.x, msg->angular.y, msg->angular.z
	// );

	// Go2 moves on ground: x is front-back, y is left-right
	// Here we forward the Twist data to the Go2 and tranform.
	sc->Move(msg->linear.x, msg->linear.y, msg->angular.z);
	// Rotation is only on theta (vector z)
	tf2::Quaternion q;
	q.setRPY(0, 0, msg->angular.z);
	broadcastTransform(msg, q);
	publishOdometry(msg, q);
}


void DogBaseNode::publishOdometry(const TwistPtr msg, const tf2::Quaternion& q){
	Odometry o;

	// Initialize o variables
	o.header.stamp = this->get_clock()->now();
	o.header.frame_id = "odom";
	o.child_frame_id = "base_link";
}


void DogBaseNode::broadcastTransform(const TwistPtr msg, const tf2::Quaternion& q){
	TransformStamped t;

	// Initialize tf variables
	t.header.stamp = this->get_clock()->now();
	t.header.frame_id = "odom";
	t.child_frame_id = "base_link";

	t.transform.translation.x = msg->linear.x;
	t.transform.translation.y = msg->linear.y;
	t.transform.translation.z = 0.0;
	//t.transform.translation = msg->linear;

	t.transform.rotation.x = q.x();
	t.transform.rotation.y = q.y();
	t.transform.rotation.z = q.z();
	t.transform.rotation.w = q.w();

	// Broadcast the transform
	tbc->sendTransform(t);
}


std::chrono::duration<double> DogBaseNode::getElapsed(){
	auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed = end - start;
	start = end;
	return elapsed;
}

//void DogBaseNode::handleResponse(const ResponsePtr msg){
//
//}