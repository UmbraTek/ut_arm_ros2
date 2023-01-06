/* Copyright 2022 Umbratek Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: jimy <jimy.zhang@umbratek.com>
 ============================================================================*/
#include <rclcpp/rclcpp.hpp>
#include <ut_msg/msg/robot_status.hpp>

#include "utra/utra_api_tcp.h"
#include "utra/utra_report_status.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ArmControllerTest : public rclcpp::Node {
 public:
  ArmControllerTest() : Node("arm_controller_test") {
    rxcnt_ = 0;
    std::string arm_ns = this->declare_parameter<std::string>("arm_ns", "utarm");
    std::string test_type = this->declare_parameter<std::string>("test_type", "type1");

    std::string ns = arm_ns + "/report/states";

    subscription_ = this->create_subscription<ut_msg::msg::RobotStatus>(
        ns, 1000, std::bind(&ArmControllerTest::topic_callback, this, _1));
  }
  int rxcnt_;

 private:
  void topic_callback(ut_msg::msg::RobotStatus::SharedPtr msg) {
    rxcnt_ = rxcnt_ + 1;
    printf("rxcnt    = %d\n", rxcnt_);
    printf("axis     = %d\n", msg->axis);
    printf("status   = %d\n", msg->motion_status);
    printf("mode     = %d\n", msg->motion_mode);
    printf("mt_brake = 0x%x\n", msg->mt_brake);
    printf("mt_able  = 0x%x\n", msg->mt_able);
    printf("err_code = %d\n", msg->err_code);
    printf("war_code = %d\n", msg->war_code);
    printf("cmd_num  = %d\n", msg->cmd_num);

    printf("joint   = ");
    for (int i = 0; i < msg->axis; i++) printf("%f ", msg->joint[i]);
    printf("\n");
    printf("pose    = ");
    for (int i = 0; i < 6; i++) printf("%f ", msg->pose[i]);
    printf(" \n");
    printf("tau     = ");
    for (int i = 0; i < msg->axis; i++) printf("%f ", msg->tau[i]);
    printf("\n\n");
  }

  rclcpp::Subscription<ut_msg::msg::RobotStatus>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmControllerTest>());
  rclcpp::shutdown();
  return 0;
}