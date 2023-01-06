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

class UtArmReportPublisher : public rclcpp::Node {
 public:
  UtArmReportPublisher() : Node("utarm_report_publisher") {
    std::string arm_ns = this->declare_parameter<std::string>("arm_ns", "utarm");
    std::string arm_ip = this->declare_parameter<std::string>("arm_ip", "192.168.1.1");
    int report_hz = this->declare_parameter<std::int32_t>("report_hz", 10);
    RCLCPP_INFO(this->get_logger(), "arm_ns: %s", arm_ns.c_str());
    RCLCPP_INFO(this->get_logger(), "arm_ip: %s", arm_ip.c_str());
    RCLCPP_INFO(this->get_logger(), "report_hz: %d", report_hz);

    uint8_t axis;
    UtraApiTcp *armapi = new UtraApiTcp((char *)(arm_ip.c_str()));
    if (armapi->is_error()) {
      RCLCPP_ERROR(this->get_logger(), "arm connection failed: %s", arm_ip.c_str());
      return;
    }

    int ret = armapi->get_axis(&axis);
    if (ret != 0 && ret != -4) {
      RCLCPP_ERROR(this->get_logger(), "arm connection failed: %s", arm_ip.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "arm connection successful: %s", arm_ip.c_str());

    publisher_ = this->create_publisher<ut_msg::msg::RobotStatus>(arm_ns + "/report/states", 1000);
    timer_ = this->create_wall_timer(5ms, std::bind(&UtArmReportPublisher::timer_callback, this));

    if (report_hz == 100)
      arm_report_ = new UtraReportStatus100Hz((char *)(arm_ip.c_str()), axis);
    else
      arm_report_ = new UtraReportStatus10Hz((char *)(arm_ip.c_str()), axis);
  }

 private:
  void timer_callback() {
    if (arm_report_->is_update()) {
      // RCLCPP_INFO(this->get_logger(), "arm_report_->is_update");
      arm_report_->get_data(&rx_data_);
      // arm_report_->print_data(&rx_data_);
      robotMsg_.len = rx_data_.len;
      robotMsg_.axis = rx_data_.axis;
      robotMsg_.motion_status = rx_data_.motion_status;
      robotMsg_.motion_mode = rx_data_.motion_mode;
      robotMsg_.mt_brake = rx_data_.mt_brake;
      robotMsg_.mt_able = rx_data_.mt_able;
      robotMsg_.err_code = rx_data_.err_code;
      robotMsg_.war_code = rx_data_.war_code;
      robotMsg_.cmd_num = rx_data_.cmd_num;
      for (size_t i = 0; i < 32; i++) robotMsg_.joint[i] = rx_data_.joint[i];
      for (size_t i = 0; i < 6; i++) robotMsg_.pose[i] = rx_data_.pose[i];
      for (size_t i = 0; i < 32; i++) robotMsg_.tau[i] = rx_data_.tau[i];

      publisher_->publish(robotMsg_);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ut_msg::msg::RobotStatus>::SharedPtr publisher_;

  arm_report_status_t rx_data_;
  ArmReportStatus *arm_report_;
  ut_msg::msg::RobotStatus robotMsg_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UtArmReportPublisher>());
  rclcpp::shutdown();
  return 0;
}