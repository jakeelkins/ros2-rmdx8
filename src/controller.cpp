#include <chrono>
#include <functional>
#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rmd_test/msg/desired_trajectory.hpp"
#include "rmd_test/msg/control.hpp"
#include "rmd_test/msg/state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Controller : public rclcpp::Node {
public:

  // nominal control frequency (s) CURRENT: ensure same as time publisher
  float control_period = 0.02;

  // needed for our timer: ms duration cast
  std::chrono::duration<long double, std::milli> control_period_ms = control_period*1000ms;


  Controller():Node("controller"){
    // subscribe to current state x as input
    subscription_x = this->create_subscription<rmd_test::msg::State>(
      "state", 10, std::bind(&Controller::state_receive_callback, this, _1));

    // subscribe to topic desired_trajectory as input
    subscription_xd = this->create_subscription<rmd_test::msg::DesiredTrajectory>(
      "desired_trajectory", 10, std::bind(&Controller::traj_receive_callback, this, _1));

    // publish to control topic
    publisher_ = this->create_publisher<rmd_test::msg::Control>("control", 10);

    // wall timer for publishing control
    timer_ = this->create_wall_timer(
        control_period_ms, std::bind(&Controller::control_callback, this));

  }

private:

  rclcpp::Subscription<rmd_test::msg::State>::SharedPtr subscription_x;
  rclcpp::Subscription<rmd_test::msg::DesiredTrajectory>::SharedPtr subscription_xd;
  rclcpp::Publisher<rmd_test::msg::Control>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --  state ICs --
  // match these to the desireds so u is 0 before the experiment runs
  float q = 0.0f;
  float q_dot = 0.0f;

  // -- desired trajectory ICs --
  float qd = 0.0f;
  float qd_dot = 0.0f;
  float qd_ddot = 0.0f;

  void state_receive_callback(const rmd_test::msg::State &state_msg){

    // -- record current state inputs --
    q = state_msg.q;
    q_dot = state_msg.q_dot;

  }

  void traj_receive_callback(const rmd_test::msg::DesiredTrajectory &desired_traj_msg){

    // -- record desired trajectory inputs --
    qd = desired_traj_msg.qd;
    qd_dot = desired_traj_msg.qd_dot;
    qd_ddot = desired_traj_msg.qd_ddot;

  }

  // THE MEAT: controller.
  void control_callback(){

    // out msg:
    auto control_msg = rmd_test::msg::Control();

    RCLCPP_INFO(this->get_logger(), "curr q: %f  curr q_dot: %f  curr qd: %f", q, q_dot, qd);

    // ---- control calcs ----
    float e = qd - q;
    float e_dot = qd_dot - q_dot;

    float Kp = 10.0f;
    float Kd = 1.2f;

    float u = Kp*e + Kd*e_dot;

    //float u = 0.0f;

    // clip
    float clip = 10.0f;

    if (u > clip){
      u = clip;
    } else if (u < -clip){
      u = -clip;
    }

    // ------------------------

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
