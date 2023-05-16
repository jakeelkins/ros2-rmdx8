#include <chrono>
#include <functional>
#include <memory>
#include <math.h>
#include <cstdio>
#include <vector>
#include <random>
#include <cmath>
// #include <string>
// #include <fstream>
// #include <vector>
// #include <utility> // std::pair

#include "rclcpp/rclcpp.hpp"
#include "rmd_test/msg/desired_trajectory.hpp"
#include "rmd_test/msg/control.hpp"
#include "rmd_test/msg/state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

class Controller : public rclcpp::Node {
public:

  // nominal control frequency (s) CURRENT: ensure same as time publisher
  float control_period = 0.02;

  // needed for our timer: ms duration cast
  chrono::duration<long double, std::milli> control_period_ms = control_period*1000ms;


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

  float e_int = 0.0f;

  // // ----- saving -----
  // float tf = 15;
  // int save_idx = 0;
  // int not_saved = true;  // flag for saving csv
  // int save_length = (int)(tf/control_period);

  // vector<float> y_save (
  //       save_length,
  //       0.0f
  //   );

  //   vector<float> yd_save (
  //       save_length,
  //       0.0f
  //   );

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

    //RCLCPP_INFO(this->get_logger(), "curr q: %f  curr q_dot: %f  curr qd: %f", q, q_dot, qd);

    // ---- control calcs ----
    float e = qd - q;
    float e_dot = qd_dot - q_dot;
    e_int += e*control_period;

    // worked with our usual internal gains
    float Kp = 10.0f;
    float Ki = 0.2f;
    float Kd = 1.6f;
    // float Kp = 8.0f;
    // float Kd = 1.0f;

    float int_clip = 0.2;
    float int_term = Ki*e_int;

    if (int_term > int_clip){
      int_term = int_clip;
    } else if (int_term < -int_clip){
      int_term = -int_clip;
    }

    float u = Kp*e + int_term + Kd*e_dot;

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

    // // record
    // if (save_idx < save_length){
    //   y_save[save_idx] = q;
    //   yd_save[save_idx] = qd;
    //   ++save_idx;
    // } else if (not_saved){
    //   // save now
    //   // wrap and save
    //   vector<std::pair<string, vector<float>>> out_dataset = {{"y", y_save}, {"yd", yd_save}, {"yd", yd_save}, {"s", s_save}};
      
    //   // Write the vector to CSV
    //   write_csv("data/sim_out.csv", out_dataset);

    //   not_saved = false
    // }
    
  }
  
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<Controller>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
