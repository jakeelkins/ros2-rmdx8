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

    //RCLCPP_INFO(this->get_logger(), "Current t: %f s", msg.curr_t);

    // -- desired trajectory calcs --
    // float qd = sin(t);
    // float qd_dot = cos(t);
    // float qd_ddot = -sin(t);

    float qd = 0.0f;
    float qd_dot = 0.0f;
    float qd_ddot = 0.0f;

    if (t > 5.0f){
      qd = 30*(3.14159265/180);
    }
    
    if (t > 10.0f) {
      qd = 0.0f;
    }

    desired_traj_msg.qd = qd;
    desired_traj_msg.qd_dot = qd_dot;
    desired_traj_msg.qd_ddot = qd_ddot;

    //RCLCPP_INFO(this->get_logger(), "Publishing desired trajectory...");

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
