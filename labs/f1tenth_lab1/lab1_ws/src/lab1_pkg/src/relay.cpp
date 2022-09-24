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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
using std::placeholders::_1;
using ackermann_msgs::msg::AckermannDriveStamped;

class Relay : public rclcpp::Node
{
public:
  Relay()
  : Node("relay")
  {
    // Subscriber
    subscription_ = this->create_subscription<AckermannDriveStamped>(
      "drive", 10, std::bind(&Relay::topic_callback, this, _1));

    // Publisher
    publisher_ = this->create_publisher<AckermannDriveStamped>("drive_relay", 10);
  }

private:
  void topic_callback(const AckermannDriveStamped::SharedPtr msg) 
  {
    RCLCPP_DEBUG(
      this->get_logger(), 
      "Received: speed: %f steering_angle: %f", 
      msg->drive.speed,
      msg->drive.steering_angle
      );
    auto publish_msg = AckermannDriveStamped();
    publish_msg.drive.speed = msg->drive.speed;
    publish_msg.drive.steering_angle = msg->drive.steering_angle;
    update_values_(publish_msg);
    // updated_msg.drive.speed = v_to_send_;
    // updated_msg.drive.steering_angle = d_to_send_;
    RCLCPP_DEBUG(
      this->get_logger(), 
      "Publishing: speed: %f steering_angle: %f", 
      publish_msg.drive.speed,
      publish_msg.drive.steering_angle
      );
    publisher_->publish(publish_msg);
  }

  void update_values_(AckermannDriveStamped& publish_msg){
    publish_msg.drive.speed *= 3;
    publish_msg.drive.steering_angle *= 3;
  }

  rclcpp::Subscription<AckermannDriveStamped>::SharedPtr subscription_;
  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Relay>());
  rclcpp::shutdown();
  return 0;
}
