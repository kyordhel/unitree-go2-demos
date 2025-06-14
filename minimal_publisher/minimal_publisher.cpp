/* ** *****************************************************************
* minimal_publisher.cpp
*
* ROS2 MinimalPublisher but with a Twist
* Bsaed on https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
*
* Author:  Mauricio Matamoros
* License: MIT
** ** ****************************************************************/
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

using Twist            = geometry_msgs::msg::Twist;
using TwistPtr         = std::shared_ptr<Twist>;

int main(int argc, char **argv);
class MinimalPublisher : public rclcpp::Node{
	private:
		rclcpp::TimerBase::SharedPtr        tmr;
		rclcpp::TimerBase::SharedPtr        tmrSM;
		rclcpp::Publisher<Twist>::SharedPtr pub;
		size_t count;

	public:
		MinimalPublisher();

	private:
	void timerCallback();
	void timerSMCallback();
};



int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalPublisher>());
	rclcpp::shutdown();
	return 0;
}



MinimalPublisher::MinimalPublisher():
	Node("minimal_publisher"), count(0){
	pub = this->create_publisher<Twist>("/cmd_vel", 10);
	tmr = this->create_wall_timer(500ms,
		std::bind(&MinimalPublisher::timerCallback, this)
	);
	tmrSM= this->create_wall_timer(3s,
		std::bind(&MinimalPublisher::timerSMCallback, this)
	);
}


void MinimalPublisher::timerCallback(){
	Twist msg = Twist();
	switch(count){
		case 0: msg.linear.x  =  0.3; break;
		case 1: msg.linear.x  = -0.3; break;
		case 2: msg.angular.z =  0.4; break;
		case 3: msg.angular.z = -0.4; break;
		case 4: msg.angular.z = -0.4; break;
		case 5: msg.angular.z =  0.4; break;
		case 6:
			msg.linear.x  =  0.3;
			msg.angular.z =  0.4;
			break;
	}
	
	pub->publish(msg);
}

void MinimalPublisher::timerSMCallback(){
	if(++count > 6) rclcpp::shutdown();
}
