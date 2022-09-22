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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Talker : public rclcpp::Node
{
public:
  Talker()
  : Node("talker"), count_(0)
  {
    // Params
    v_ = this->declare_parameter<double>("v", 0.0);
    d_ = this->declare_parameter<double>("d", 0.0);
    

    // Publisher
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    
    // Timer
    timer_ = this->create_wall_timer(
      0ms, std::bind(&Talker::timer_callback, this));
  }

private:
  void timer_callback()
  {
    this->get_parameter("v", v_);
    this->get_parameter("d", d_);
    auto message = ackermann_msgs::msg::AckermannDriveStamped();
    message.drive.speed = v_;
    message.drive.steering_angle = d_;
    RCLCPP_INFO(
      this->get_logger(), 
      "Publishing: speed: %f steering_angle: %f", 
      message.drive.speed,
      message.drive.steering_angle
      );
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  size_t count_;
  double v_;
  double d_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
