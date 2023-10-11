/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */
#include <chrono>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdint.h>

#include <rclcpp/rclcpp.hpp>
#include <cmath>
// #include <Eigen/Eigen>
// #include <Eigen/Geometry>

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffbRecv : public rclcpp::Node
{
public:
	OffbRecv() : Node("offb_recv")
	{
		offboard_setpoint_counter_ = 0;

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		target_name = "/px4_1/fmu";
		std::cout << target_name + "/out/vehicle_status" << std::endl;
		// vehicle_status_sub_ 			= this->create_subscription<VehicleStatus>(
		// 	target_name + "/out/vehicle_status", qos,
		// 	std::bind(&OffbRecv::vehicle_status_cb, this, std::placeholders::_1));
		vehicle_global_position_sub_	= this->create_subscription<VehicleGlobalPosition>(
			target_name + "/out/vehicle_global_position", qos,
			std::bind(&OffbRecv::vehicle_global_position_cb, this, std::placeholders::_1));
		// vehicle_local_position_sub_		= this->create_subscription<VehicleLocalPosition>(
		// 	target_name + "/out/vehicle_local_position", qos,
		// 	std::bind(&OffbRecv::vehicle_local_position_cb, this, std::placeholders::_1));
		// vehicle_attitude_sub_			= this->create_subscription<VehicleAttitude>(
		// 	target_name + "/out/vehicle_attitude", qos,
		// 	std::bind(&OffbRecv::vehicle_attitude_cb, this, std::placeholders::_1));
		// vehicle_control_mode_sub_ 		= this->create_subscription<VehicleControlMode>(	target_name + "/out/vehicle_control_mode", 1);
		// vehicle_global_position_sub_ 	= this->create_subscription<VehicleGlobalPosition>(target_name + "/out/vehicle_global_position", 1);
		// vehicle_local_position_sub_ 	= this->create_subscription<VehicleLocalPosition>(	target_name + "/out/vehicle_local_position", 1);
	}

	void vehicle_status_cb(const VehicleStatus::UniquePtr msg) const{
		// this->offboard_setpoint_counter_++;
		std::cout << "Receivec Vehicle Status : " << std::endl;
		std::cout << "timestamp							" << msg->timestamp << std::endl;
		std::cout << "nav_state_timestamp				" << msg->nav_state_timestamp << std::endl;
		std::cout << "nav_state							" << msg->nav_state << std::endl;
		std::cout << "failure_detector_status			" << msg->failure_detector_status << std::endl;
		std::cout << "vehicle_type						" << msg->vehicle_type << std::endl;
		std::cout << "failsafe							" << msg->failsafe << std::endl;
		std::cout << "is_vtol							" << msg->is_vtol << std::endl;
		std::cout << "is_vtol_tailsitter				" << msg->is_vtol_tailsitter << std::endl;
		std::cout << "in_transition_mode				" << msg->in_transition_mode << std::endl;
		std::cout << "in_transition_to_fw				" << msg->in_transition_to_fw << std::endl;
		std::cout << "system_type						" << msg->system_type << std::endl;
		std::cout << "system_id							" << msg->system_id << std::endl;
		std::cout << "component_id						" << msg->component_id << std::endl;
		std::cout << "power_input_valid					" << msg->power_input_valid << std::endl;
		std::cout << "usb_connected						" << msg->usb_connected << std::endl;
	}

	void vehicle_global_position_cb(const VehicleGlobalPosition::UniquePtr msg){
		timestamp = msg->timestamp;
		lat = msg->lat;
		lon = msg->lon;
		alt = msg->alt;
		double sec 	= double(msg->timestamp/1000000);
		double usec = double(msg->timestamp - ((msg->timestamp/1000000)*1000000));
		// printf("Time %20.0f.%6.0f / ", sec, usec);
		printf("Time %20.6f ", double(msg->timestamp)/1000000.0);
		printf("LLA %10.6f %10.6f %8.2f\n", msg->lat, msg->lon, msg->alt);
	}
	void vehicle_local_position_cb(const VehicleLocalPosition::UniquePtr msg){
		acc[0] = float(msg->ax);
		acc[1] = float(msg->ay);
		acc[2] = float(msg->az);
		vel[0] = float(msg->vx);
		vel[1] = float(msg->vy);
		vel[2] = float(msg->vz);
		// this->vel = Eigen::Vector3d(msg->vx, msg->vy, msg->vz);
		// this->acc = Eigen::Vector3d(msg->ax, msg->ay, msg->az);
	}
	void vehicle_attitude_cb(const VehicleAttitude::UniquePtr msg){
		q[0] = msg->q[0];
		q[1] = msg->q[1];
		q[2] = msg->q[2];
		q[3] = msg->q[3];
		// this->q = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
		// this->euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
	}

private:
	std::string target_name;
	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	rclcpp::Subscription<VehicleStatus>::SharedPtr 			vehicle_status_sub_;
	rclcpp::Subscription<VehicleControlMode>::SharedPtr		vehicle_control_mode_sub_;
	rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr 	vehicle_global_position_sub_;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr 		vehicle_attitude_sub_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr 	vehicle_local_position_sub_;

	uint64_t timestamp;
	double lat;
	double lon;
	float alt;
	float q[4];
	float acc[3];
	float vel[3];
	float euler[3];
	// Eigen::Quaterniond q;
	// Eigen::Vector3d vel;
	// Eigen::Vector3d acc;
	// Eigen::Vector3d euler;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
};

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffbRecv>());

	rclcpp::shutdown();
	return 0;
}
