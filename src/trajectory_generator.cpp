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
#include "rmd_test/msg/time.hpp"
#include "rmd_test/msg/desired_trajectory.hpp"

using std::placeholders::_1;

class TrajectoryGenerator : public rclcpp::Node {
public:

  TrajectoryGenerator():Node("trajectory_generator"){

    // subscribe to topic t as input
    subscription_ = this->create_subscription<rmd_test::msg::Time>(
      "t", 10, std::bind(&TrajectoryGenerator::t_received_callback, this, _1));

    // publish to desired_trajectory topic
    publisher_ = this->create_publisher<rmd_test::msg::DesiredTrajectory>("desired_trajectory", 10);

  }

private:

  rclcpp::Subscription<rmd_test::msg::Time>::SharedPtr subscription_;
  rclcpp::Publisher<rmd_test::msg::DesiredTrajectory>::SharedPtr publisher_;

  void t_received_callback(const rmd_test::msg::Time &msg){

    auto desired_traj_msg = rmd_test::msg::DesiredTrajectory();
    float t = msg.curr_t;

    RCLCPP_INFO(this->get_logger(), "Current t: %f s", msg.curr_t);

    // -- desired trajectory calcs --
    float qd = sin(t);
    float qd_dot = cos(t);
    float qd_ddot = -sin(t);

    desired_traj_msg.qd = qd;
    desired_traj_msg.qd_dot = qd_dot;
    desired_traj_msg.qd_ddot = qd_ddot;

    RCLCPP_INFO(this->get_logger(), "Publishing desired trajectory...");

    publisher_->publish(desired_traj_msg);

  }
  
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<TrajectoryGenerator>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
