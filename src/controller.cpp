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

#include <functional>
#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rmd_test/msg/desired_trajectory.hpp"
#include "rmd_test/msg/control.hpp"

using std::placeholders::_1;

class Controller : public rclcpp::Node {
public:

  Controller():Node("controller"){
    // TODO: will need subscribed to x as well.

    // subscribe to topic desired_trajectory as input
    subscription_ = this->create_subscription<rmd_test::msg::DesiredTrajectory>(
      "desired_trajectory", 10, std::bind(&Controller::traj_receive_callback, this, _1));

    // publish to control topic
    publisher_ = this->create_publisher<rmd_test::msg::Control>("control", 10);

  }

private:

  rclcpp::Subscription<rmd_test::msg::DesiredTrajectory>::SharedPtr subscription_;
  rclcpp::Publisher<rmd_test::msg::Control>::SharedPtr publisher_;

  void traj_receive_callback(const rmd_test::msg::DesiredTrajectory &desired_traj_msg){

    // out msg:
    auto control_msg = rmd_test::msg::Control();

    // -- desired trajectory inputs --
    float qd = desired_traj_msg.qd;
    float qd_dot = desired_traj_msg.qd_dot;
    float qd_ddot = desired_traj_msg.qd_ddot;

    // TODO: how to handle multiple subscriptions?
    // put input x here.

    // controller calcs
    // TODO
    float u = 0.0f;

    // put into msg and publish control
    //RCLCPP_INFO(this->get_logger(), "Publishing desired trajectory...");

    control_msg.u = u;

    publisher_->publish(control_msg);

  }
  
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<Controller>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
