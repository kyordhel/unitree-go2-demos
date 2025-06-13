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
#include <rclcpp/rclcpp.hpp>

#include "sport_client/sport_client.h"

using Request = unitree_api::msg::Request;

rclcpp::Publisher<Request>::SharedPtr publisher;
void task();
bool walk(SportClient& sc, float speed, uint8_t seconds);
bool turn(SportClient& sc, float arad);

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	rclcpp::Node::SharedPtr nodePtr = std::make_shared<rclcpp::Node>("req_sender");
	publisher = nodePtr->create_publisher<Request>("/api/sport/request", 10);

	std::thread task_thread( task );
	rclcpp::spin(nodePtr);
	task_thread.join();
	return 0;
}

void task(){
	SportClient sc(publisher);

	printf("Dog down\n");
	sc.StandDown();
	std::this_thread::sleep_for(std::chrono::seconds(2));
	if( !rclcpp::ok() ) return;

	printf("Dog stand\n");
	sc.StandUp();
	std::this_thread::sleep_for(std::chrono::seconds(2));
	if( !rclcpp::ok() ) return;

	sc.SpeedLevel(-1);

	printf("Dog move forward\n");
	if( ! walk(sc, 0.3, 5) ) return;
	
	printf("Dog move backwards\n");
	if( ! walk(sc, -0.3, 5) ) return;

	printf("Turn and sleep\n");
	if( ! turn(sc, -1) ) return;
	if( ! turn(sc,  2) ) return;
	if( ! turn(sc, -1) ) return;
	sc.StandDown();
}

bool walk(SportClient& sc, float speed, uint8_t seconds){
	if(speed >  1) speed =  1;
	if(speed < -1) speed = -1;
	for(uint8_t s = 0; s < seconds; ++s){
		if( !rclcpp::ok() ) return false;
		sc.Move(speed, 0, 0);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	return true;
}

bool turn(SportClient& sc, float arad){
	if(arad >  3.141592) arad =  3.141592;
	if(arad < -3.141592) arad = -3.141592;
	uint16_t time = std::min<uint16_t>(500, 1000 * std::abs(arad));

	if( !rclcpp::ok() ) return false;
	sc.Move(0, 0, arad);
	std::this_thread::sleep_for(std::chrono::milliseconds(time));
	return true;
}