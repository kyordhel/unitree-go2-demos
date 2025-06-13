/* ** *****************************************************************
* walk.cpp
*
* Second test attempt to control the unitree Go2 using the SportClient
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
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "sport_client/sport_client.h"



using Request          = unitree_api::msg::Request;
using Twist            = geometry_msgs::msg::Twist;
using TwistPtr         = std::shared_ptr<Twist>;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using SportClientPtr   = std::shared_ptr<SportClient>;


void task();
bool walk(SportClient& sc, float speed, uint8_t seconds);
bool turn(SportClient& sc, float arad);
int main(int argc, char **argv);

class DogBaseNode : public rclcpp::Node{
	private:
		SportClientPtr sc;
		rclcpp::Publisher<Request>::SharedPtr pub;
		rclcpp::Subscription<Twist>::SharedPtr sub;
		std::unique_ptr<tf2_ros::TransformBroadcaster> tbc;

	public:
		DogBaseNode();

	private:
		void handleTwist(const TwistPtr msg);
};



int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DogBaseNode>());
	rclcpp::shutdown();
	return 0;
}



DogBaseNode::DogBaseNode():
	Node("dog_base_node"){
	pub = this->create_publisher<Request>("/api/sport/request", 10);
	sub = this->create_subscription<Twist>("", 10,
	std::bind(&DogBaseNode::handleTwist, this, std::placeholders::_1));
	sc  = std::make_shared<SportClient>(pub);
}



void DogBaseNode::handleTwist(const TwistPtr msg){
	TransformStamped t;

	// Initialize tf variables
	t.header.stamp = this->get_clock()->now();
	t.header.frame_id = "odom";
	t.child_frame_id = "base_link";

	// Go2 moves on ground: x is front-back, y is left-right
	// Here we forward the Twist data to the Go2 and tranform.
	sc.Move(msg->linear.x, msg->linear.y, msg->angular.z);
	t.transform.translation.x = msg->linear->x;
	t.transform.translation.y = msg->linear->y;
	t.transform.translation.z = 0.0;
	//t.transform.translation = msg->linear;

	// Rotation is only on theta (vector z)
	tf2::Quaternion q;
	q.setRPY(0, 0, msg->angular.z);
	t.transform.rotation.x = q.x();
	t.transform.rotation.y = q.y();
	t.transform.rotation.z = q.z();
	t.transform.rotation.w = q.w();

	// Broadcast the transform
	tbc->sendTransform(t);
}



bool walk(SportClient& sc, float speed, uint8_t seconds){
	if(speed >  1) speed =  1;
	if(speed < -1) speed = -1;
	uint32_t ds = 500;
	for(uint32_t ms = 0; ms < 1000*seconds; ms+=ds){
		if( !rclcpp::ok() ) return false;
		sc.Move(speed, 0, 0);
		std::this_thread::sleep_for(std::chrono::milliseconds(ds));
	}
	return true;
}

bool turn(SportClient& sc, float arad){
	if(arad >  3.141592) arad =  3.141592;
	if(arad < -3.141592) arad = -3.141592;
	uint16_t time = std::min<uint16_t>(1500, 1500 * std::abs(arad));

	if( !rclcpp::ok() ) return false;
	sc.Move(0, 0, arad);
	std::this_thread::sleep_for(std::chrono::milliseconds(time));
	return true;
}
