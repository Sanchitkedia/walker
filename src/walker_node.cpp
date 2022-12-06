/******************************************************************************
  *MIT License

  *Copyright (c) 2022 Sanchit Kedia

  *Permission is hereby granted, free of charge, to any person obtaining a copy
  *of this software and associated documentation files (the "Software"), to deal
  *in the Software without restriction, including without limitation the rights
  *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  *copies of the Software, and to permit persons to whom the Software is
  *furnished to do so, subject to the following conditions:

  *The above copyright notice and this permission notice shall be included in all
  *copies or substantial portions of the Software.

  *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  *SOFTWARE.
  ******************************************************************************/
/**
 * @file walker_node.cpp
 * @author Sanchit Kedia (sanchit@terpmail.umd.edu)
 * @brief Walker algorithm implementation to emulate the Roomba robot with obstacle avoidance
 * @version 0.3
 * @date 2022-12-04
 *
 * @copyright MIT Copyright (c) 2022
 *
 */
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using IMAGE = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

typedef enum {
  FORWARD = 0,
  STOP,
  TURN,
} StateType;

/**
 * @brief A simple walker node that moves the robot forward until it detects an obstacle
 *
 * 
 */
class RoomBa : public rclcpp::Node {
 public:
  RoomBa() : Node("walker"), state_(FORWARD) {
    auto pubTopicName = "cmd_vel";
    publisher_ = this->create_publisher<TWIST>(pubTopicName, 10);

    // Publisher callback
    auto processCallback = std::bind(&RoomBa::callback, this);
    timer_ = this->create_wall_timer(100ms, processCallback);

    auto subCallback = std::bind(&RoomBa::subscribe_callback, this, _1);
    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    subscription_ = this->create_subscription<IMAGE>(
        "scan", default_qos, subCallback);
  }

 private:
 /**
  * @brief Callback function for to publish the velocity commands to the robot depending on the state and the distance to the obstacle
  * 
  */
  void callback() {
    auto message = TWIST();

    if (lastImg_.header.stamp.sec == 0) {
      message.linear.x = 0.3;
      publisher_->publish(message);
      return;
    }
    // state machine (Mealy -- output on transition)
    switch (state_) {
      case FORWARD:
        message.linear.x = 0.1;
        RCLCPP_INFO(this->get_logger(), "Moving");
        if (hasObstacle()) {  // check transition
          state_ = STOP;
          publisher_->publish(message);
          RCLCPP_INFO_STREAM(this->get_logger(), "State = STOP");
        }
        break;
      case STOP:
        RCLCPP_INFO(this->get_logger(), "Stopped");

        if (hasObstacle()) {  // check transition
          state_ = TURN;
          // message.linear.x = 0;
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
        RCLCPP_INFO(this->get_logger(), "Turning");
        if (!hasObstacle()) {  // check transition
          state_ = FORWARD;
          message.linear.x = 0.1;
          publisher_->publish(message);
          RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
        }
        break;
    }

    /**
     * @brief Subscribes to the laser scan topic and stores the data in the lastImg_ variable
     *
     */
  }
  void subscribe_callback(const IMAGE& msg) { lastImg_ = msg; }
  /**
   * @brief Checks if the robot is close to an obstacle or not by checking the output of the laser scan topic
   *
   * @return true
   * @return false
   */
  bool hasObstacle() {
    for (int i{}; i < 10; i++) {
      if (lastImg_.ranges[i] < 1 || (lastImg_.ranges[359 - i] < 1)) {
        auto dist = (lastImg_.ranges[i] + lastImg_.ranges[359 - i]) / 2;
        RCLCPP_INFO(this->get_logger(), "Distance to Obstacle  '%f'm ", dist);
        return true;
      }
    }
    return false;
  }

  ////////////////////////////////////////
  // member variables
  ////////////////////////////////////////
  rclcpp::Subscription<IMAGE>::SharedPtr  subscription_;
  rclcpp::Publisher<TWIST>::SharedPtr     publisher_;
  rclcpp::TimerBase::SharedPtr            timer_;
  IMAGE                                   lastImg_;
  StateType                               state_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoomBa>());
  rclcpp::shutdown();
  return 0;
}
