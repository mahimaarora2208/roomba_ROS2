/************************************************************************************
 * Apache License 2.0
 * Copyright (c) 2021, Mahima Arora
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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
 ************************************************************************************/
/**
 *  @file    obstacle_avoidance.cpp
 *  @author  Mahima Arora
 *  @date    12/03/2022
 *  @version 1.0
 *
 *  @brief Source file to implement a simple ROS publisher node and a service
 *         server node
 *
 *  @section DESCRIPTION
 *
 *  Source file to implement a simple ROS2 Obstacle avoidance node publishing a
 * custom message and facilitate change in message content upon a request
 *
 */

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using IMAGE = sensor_msgs::msg::Image;
using TWIST = geometry_msgs::msg::Twist;

typedef enum {
  FORWARD = 0,
  STOP,
  TURN,
} StateType;

/**
 * @brief RoomBa Class
 *
 */
class RoomBa : public rclcpp::Node {
 public:
  RoomBa() : Node("walker"), state_(STOP) {
    // creates publisher to publish /cmd_vel topic
    auto pubTopicName = "cmd_vel";
    publisher_ = this->create_publisher<TWIST>(pubTopicName, 10);

    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    // creates subscriber to get /demo_cam/camera/depth_demo topic
    auto subTopicName = "/demo_cam/camera/depth_demo";
    auto subCallback = std::bind(&RoomBa::subscribe_callback, this, _1);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subTopicName, default_qos, subCallback);

    // create a 10Hz timer for processing

    auto processCallback = std::bind(&RoomBa::process_callback, this);
    timer_ = this->create_wall_timer(100ms, processCallback);
  }

 private:
  void subscribe_callback(const IMAGE& msg) { lastImg_ = msg; }

  void process_callback() {
    // Do nothing until the first data read
    if (lastImg_.header.stamp.sec == 0) {
      return;
    }

    // Create the message to publish (initialized to all 0)
    auto message = TWIST();
    // state machine (Mealy -- output on transition)
    switch (state_) {
      case FORWARD:
        if (hasObstacle()) {  // check transition
          state_ = STOP;
          publisher_->publish(message);
          RCLCPP_INFO_STREAM(this->get_logger(), "State = STOP");
        }
        break;
      case STOP:
        if (hasObstacle()) {  // check transition
          state_ = TURN;
          message.angular.z = 0.1;
          publisher_->publish(message);
          RCLCPP_INFO_STREAM(this->get_logger(), "State = TURN");
        } else {
          state_ = FORWARD;
          message.linear.x = 0.1;
          publisher_->publish(message);
          RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
        }
        break;
      case TURN:
        if (!hasObstacle()) {  // check transition
          state_ = FORWARD;
          message.linear.x = 0.1;
          publisher_->publish(message);
          RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
        }
        break;
    }
  }

  /**
   * @brief hasObstacle() function return true if obstacle detected
   *
   *
   */

  bool hasObstacle() {
    unsigned char* dataPtr = lastImg_.data.data();
    float* floatData = (float*)dataPtr;

    int idx;
    for (unsigned int row = 0; row < lastImg_.height - 40; row++)
      for (unsigned int col = 0; col < lastImg_.width; col++) {
        idx = (row * lastImg_.width) + col;
        if (floatData[idx] < 1.0) {
          RCLCPP_INFO(this->get_logger(),
                      "row=%d, col=%d, floatData[idx] = %.2f", row, col,
                      floatData[idx]);
          return true;
        }
      }

    return false;
  }

  ////////////////////////////////////////
  // member variables
  ////////////////////////////////////////
  rclcpp::Subscription<IMAGE>::SharedPtr subscription_;
  rclcpp::Publisher<TWIST>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  IMAGE lastImg_;
  StateType state_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoomBa>());
  rclcpp::shutdown();
  return 0;
}