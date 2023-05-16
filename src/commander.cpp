#include <functional>
#include <memory>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <cmath>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "rmd_test/msg/control.hpp"

using std::placeholders::_1;


class Commander : public rclcpp::Node{
public:

    // -- servo connection stuff --
    int s;
    int stat;
    struct sockaddr_can addr;
    struct ifreq ifr;

    struct can_frame frame;
    int nbytes;
    socklen_t len;

    int servo_id = 0x141;
    int can_dlc = 8;
    int gear_ratio = 6;

    // save these for future PID experiment restarts
    unsigned int default_pos_kp = 100;
    unsigned int default_pos_ki = 100;
    unsigned int default_speed_kp = 50;
    unsigned int default_speed_ki = 40;
    unsigned int default_torque_kp = 50;
    unsigned int default_torque_ki = 50;

    unsigned int pos_kp;
    unsigned int pos_ki;
    unsigned int speed_kp;
    unsigned int speed_ki;
    unsigned int torque_kp;
    unsigned int torque_ki; 


    Commander():Node("commander"){

        // -- connect to servo --
        connect();

        if (stat){
            disconnect();
        } else {
            //printf("Socket connected. \n");
            RCLCPP_INFO(this->get_logger(), "Commander connected to socket");
        }

        // // ready the experiment: set default PI gains for slew
        // set_pid_gains(default_pos_kp, default_pos_ki,
        //               default_speed_kp, default_speed_ki, 
        //               default_torque_kp, default_torque_ki);
        
        // // slew to zero at 30 deg/s
        // go_to_angle(0.0f, 30.0f);

        // RCLCPP_INFO(this->get_logger(), "Slewing to initial angle...");

        // sleep(7);

        // zero PI gains on servo to ready for torque control only
        // set_pid_gains(0, 0,
        //               0, 0, 
        //               5, 1);

        RCLCPP_INFO(this->get_logger(), "Servo ready for experiment.");

        // -- ros stuff --

        // subscribe to control topic
        subscription_ = this->create_subscription<rmd_test::msg::Control>(
            "control", 10, std::bind(&Commander::control_receive_callback, this, _1));
    }


    // --- servo connection commands ---

    void connect(){
        // connect and bind to the servo

        s = -1;
        stat = EXIT_SUCCESS;
        len = sizeof(addr);

        s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        if (s < 0){
            perror("Error while creating socket");
            stat = EXIT_FAILURE;
        }

        // bind
    	/* get interface index */
        strcpy(ifr.ifr_name, "slcan0");
        ioctl(s, SIOCGIFINDEX, &ifr);

        // NOTE: this has to come after the strcpy and ioctl.
        // this gave me fits.
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        
        /* bind CAN socket to the given interface */
        int bind_stat = bind(s, (struct sockaddr *)&addr, sizeof(addr));

        if (bind_stat < 0){
            perror("Error in socket bind");
            stat = EXIT_FAILURE;
        }

        //return stat;
    }

    void disconnect(){
        // disconnect when something happens
        if (s >= 0){
            close(s);
            //printf("Socket disconnected. \n");
            RCLCPP_INFO(this->get_logger(), "Commander disconnected from socket");
        }
    }

    // ----------- commands -----------
    void turn_off_motor(){

        frame.can_id = servo_id;
        frame.can_dlc = can_dlc;

        // pid set cmnd
        frame.data[0] = 0x81;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;

        nbytes = sendto(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, sizeof(addr));

        if (nbytes <= 0){
            perror("Error in turn off motor send");
            stat = EXIT_FAILURE;
        }

        // // now read reply
        // nbytes = recvfrom(s, &frame, sizeof(struct can_frame),
        //         0, (struct sockaddr*)&addr, &len);

        // if (nbytes == 0){
        //     perror("Read zero bytes in turn off motor");
        //     stat = EXIT_FAILURE;
        // }

        // reply is same as we sent.

        //return stat;
    }



    void read_pid_gains(){

        frame.can_id = servo_id;
        frame.can_dlc = can_dlc;

        // pid read cmnd
        frame.data[0] = 0x30;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;

        nbytes = sendto(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, sizeof(addr));

        if (nbytes <= 0){
            perror("Error in read PID send");
            stat = EXIT_FAILURE;
        }

        // now read reply
        nbytes = recvfrom(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, &len);

        if (nbytes == 0){
            perror("Read zero bytes in read PID");
            stat = EXIT_FAILURE;
        }

        // torque is current here, but i dont
        // want to say current bc it sounds like contemporary
        pos_kp = frame.data[2]; 
        pos_ki = frame.data[3]; 

        speed_kp = frame.data[4]; 
        speed_ki = frame.data[5];

        torque_kp = frame.data[6];
        torque_ki = frame.data[7];

        //return stat;
    }


    void set_pid_gains(unsigned int pos_kp_, unsigned int pos_ki_,
                        unsigned int speed_kp_, unsigned int speed_ki_,
                        unsigned int torque_kp_, unsigned int torque_ki_){

        frame.can_id = servo_id;
        frame.can_dlc = can_dlc;

        // pid set cmnd
        frame.data[0] = 0x31;
        frame.data[1] = 0x00;
        frame.data[2] = pos_kp_;
        frame.data[3] = pos_ki_;
        frame.data[4] = speed_kp_;
        frame.data[5] = speed_ki_;
        frame.data[6] = torque_kp_;
        frame.data[7] = torque_ki_;

        nbytes = sendto(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, sizeof(addr));

        if (nbytes <= 0){
            perror("Error in set PID send");
            stat = EXIT_FAILURE;
        }

        // // now read reply
        // nbytes = recvfrom(s, &frame, sizeof(struct can_frame),
        //         0, (struct sockaddr*)&addr, &len);

        // if (nbytes == 0){
        //     perror("Read zero bytes in set PID");
        //     stat = EXIT_FAILURE;
        // }

        // // reply is same as we sent.
        // pos_kp = frame.data[2]; 
        // pos_ki = frame.data[3]; 

        // speed_kp = frame.data[4]; 
        // speed_ki = frame.data[5];

        // torque_kp = frame.data[6];
        // torque_ki = frame.data[7];

        //return stat;
    }


    void go_to_angle(float desired_angle, float desired_speed){

        // input: float of desired angle, in degrees.
        int desired_angle_int = (int)(desired_angle*100);
        desired_angle_int *= gear_ratio;
        unsigned char const * desired_angle_array = reinterpret_cast<unsigned char const *>(&desired_angle_int);

        int desired_speed_int = (int)desired_speed;
        desired_speed_int *= gear_ratio;
        unsigned char const * desired_speed_array = reinterpret_cast<unsigned char const *>(&desired_speed_int);

        frame.can_id = servo_id;
        frame.can_dlc = can_dlc;

        // angle slew cmnd
        frame.data[0] = 0xA4;
        frame.data[1] = 0x00;
        frame.data[2] = desired_speed_array[0];
        frame.data[3] = desired_speed_array[1];
        frame.data[4] = desired_angle_array[0];
        frame.data[5] = desired_angle_array[1];
        frame.data[6] = desired_angle_array[2];
        frame.data[7] = desired_angle_array[3];

        nbytes = sendto(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, sizeof(addr));

        if (nbytes <= 0){
            perror("Error in go-to-angle send");
            stat = EXIT_FAILURE;
        }

        // // now read reply
        // nbytes = recvfrom(s, &frame, sizeof(struct can_frame),
        //         0, (struct sockaddr*)&addr, &len);

        // if (nbytes == 0){
        //     perror("Read zero bytes in go-to-angle");
        //     stat = EXIT_FAILURE;
        // }

        // reply is a motor status command, which we dont care about here.

        //return stat;
    }


    void torque_command(float desired_current){

        // input: float of desired current, in Amps.
        // current is +/- 2000, corresp to +/- 32A
        // TODO: handle warnings of max control input in ROS (in controller)
        // !! NEEDS TESTED W PD !!

        // clamp
        if (desired_current > 32.0f){
            desired_current = 32.0f;
        } else if (desired_current < -32.0f) {
            desired_current = -32.0f;
        }

        // convert to the +/- 2000 range the servo requires
        float desired_current_send = desired_current*(2000/32);

        int desired_current_int = (int)(desired_current_send);
        unsigned char const * desired_current_array = reinterpret_cast<unsigned char const *>(&desired_current_int);

        frame.can_id = servo_id;
        frame.can_dlc = can_dlc;

        // torque current cmnd
        frame.data[0] = 0xA1;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = desired_current_array[0];
        frame.data[5] = desired_current_array[1];
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;

        nbytes = sendto(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, sizeof(addr));

        if (nbytes <= 0){
            //perror("Error in torque cmnd send");
            stat = EXIT_FAILURE;
        }

        // // now read reply
        // nbytes = recvfrom(s, &frame, sizeof(struct can_frame),
        //         0, (struct sockaddr*)&addr, &len);

        // if (nbytes == 0){
        //     perror("Read zero bytes in torque cmnd");
        //     stat = EXIT_FAILURE;
        // }

        // reply is a motor status command, which we dont care about here.

        //return stat;
    }


private:

    rclcpp::Subscription<rmd_test::msg::Control>::SharedPtr subscription_;

    // -- Commander ICs --
    float u = 0.0f;

    void control_receive_callback(const rmd_test::msg::Control &control_msg){

        // -- record control command --
        u = control_msg.u;

        // send command to servo
        torque_command(u);

    }


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<Commander>();

  rclcpp::spin(node);

  // shutdown servo using the public methods
  //node->turn_off_motor();
  //node->disconnect(); // disconnect from servo, that is.

  rclcpp::shutdown();
  return 0;
}