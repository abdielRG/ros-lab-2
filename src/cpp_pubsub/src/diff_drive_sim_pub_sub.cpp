// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * CSCI 497, Spring 2025, Lab 2
 * Name: Abdiel Ramirez
 * Start Date: April 18, 2025
 * End Date:   April
 * Description: 
 *
 */

#include <chrono>
#include <memory>
#include <string>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class DiffDriveSimPubSub : public rclcpp::Node
{
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
	size_t count_;

	float fwd, trn;    // Twist motion params
	char pbuf[1024];   // print buffer
	char* pbp;         // ptr into print buffer

    public:
        DiffDriveSimPubSub() : Node("ddsPubSub"), count_(0) 
	{
	    // Initialize motion variables (start by moving forward)
	    fwd = 0.5;
	    trn = 0.0;
		
	    /* publisher -- send cmd_vel messages to diff_drive */
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
			    "/ramire37/diff_drive/cmd_vel", 10);
    
            /* lambda function passed to subscription_ 
	     *
	     * scan.ranges[1] = left-facing range
	     * scan.ranges[0] = front-facing range 
	     */
            auto topic_feedback = [this](sensor_msgs::msg::LaserScan scan) -> void { 
		pbp = pbuf;  // Reset buffer write head
		pbp += sprintf(pbp, "L-range: %f. F-range: %f. ", scan.ranges[1], scan.ranges[0]);

		if (scan.ranges[0] < 2.0) {
		    // Close to wall in front
		    fwd = 0.0;
		    trn = -1.0;
		    pbp += sprintf(pbp, "Turning right...");
		    pbp = 0;
		}
		else if (scan.ranges[1] > 2.0 && scan.ranges[0] > 3.0) {
		    // Wall on right too far, get closer
		    fwd = 0.0;
		    trn = 1.0;
		    pbp += sprintf(pbp, "Turning left...");
		    pbp = 0;
		}
		else {
		    fwd = 0.5;
		    trn = 0.0;
		    pbp += sprintf(pbp, "Moving forward...");
		    pbp = 0;
		}

		RCLCPP_INFO(this->get_logger(), "%s", pbuf);
            };
    
            /* subscription -- recv laser rangefinder data from diff_drive */
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			    "/ramire37/diff_drive/scan", 10, topic_feedback);


            /* lambda function passed to timer_ */
            auto timer_callback = [this]() -> void {
                auto message = geometry_msgs::msg::Twist();
	
		// Update Twist message with latest calculated params
	        message.linear.x = fwd;
	        message.angular.z = trn;
	               
                this->publisher_->publish(message);
            };

            /* Interval at which the Twist message is sent */
	    timer_ = this->create_wall_timer(500ms, timer_callback);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveSimPubSub>());
    rclcpp::shutdown();
    return 0;
}
