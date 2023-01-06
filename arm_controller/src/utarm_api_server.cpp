/* Copyright 2022 Umbratek Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: jimy <jimy.zhang@umbratek.com>
 ============================================================================*/
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ut_msg/srv/check_connect.hpp"
#include "ut_msg/srv/connect.hpp"
#include "ut_msg/srv/disconnect.hpp"
#include "ut_msg/srv/get_float32.hpp"
#include "ut_msg/srv/get_float32s.hpp"
#include "ut_msg/srv/get_gripper_state.hpp"
#include "ut_msg/srv/get_int16.hpp"
#include "ut_msg/srv/get_uint16s.hpp"
#include "ut_msg/srv/moveto_cartesian_line.hpp"
#include "ut_msg/srv/moveto_cartesian_lineb.hpp"
#include "ut_msg/srv/moveto_joint_p2p.hpp"
#include "ut_msg/srv/moveto_servo_joint.hpp"
#include "ut_msg/srv/set_enable.hpp"
#include "ut_msg/srv/set_float32.hpp"
#include "ut_msg/srv/set_gripper_state.hpp"
#include "ut_msg/srv/set_int16.hpp"
#include "utra/utra_api_tcp.h"
#include "utra/utra_flxie_api.h"

std::string arm_ip = "";
UtraApiTcp *armapi = NULL;
UtraFlxiE2Api *fixie = NULL;
constexpr unsigned int hash(const char *s, int off = 0) { return !s[off] ? 5381 : (hash(s, off + 1) * 33) ^ s[off]; }

#define CHECK_ARM_CONNECT()                                                      \
  if (armapi == NULL) {                                                          \
    res->ret = -3;                                                               \
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "server have not connected arm"); \
    return;                                                                      \
  }

void check_connect(std::shared_ptr<ut_msg::srv::CheckConnect::Request> req,
                   std::shared_ptr<ut_msg::srv::CheckConnect::Response> res) {
  res->ip_address = "";
  CHECK_ARM_CONNECT()

  uint8_t axis;
  res->ret = armapi->get_axis(&axis);
  res->ip_address = arm_ip;
}

void connect(std::shared_ptr<ut_msg::srv::Connect::Request> req, std::shared_ptr<ut_msg::srv::Connect::Response> res) {
  res->ret = 0;
  if (armapi != NULL) return;

  uint8_t axis;
  arm_ip = req->ip_address;
  char *ip = new char[req->ip_address.length() + 1];
  std::strcpy(ip, req->ip_address.c_str());
  armapi = new UtraApiTcp(ip);
  res->ret = armapi->get_axis(&axis);
}

void disconnect(std::shared_ptr<ut_msg::srv::Disconnect::Request> req,
                std::shared_ptr<ut_msg::srv::Disconnect::Response> res) {
  CHECK_ARM_CONNECT()

  delete armapi;
  armapi = NULL;
  res->ret = 0;
}

void moveto_servo_joint(std::shared_ptr<ut_msg::srv::MovetoServoJoint::Request> req,
                        std::shared_ptr<ut_msg::srv::MovetoServoJoint::Response> res) {
  CHECK_ARM_CONNECT()

  int ret = 0;
  int frames_index = 0;
  int frames_per_cmd = 3;  // how many points with one time to send

  int axiz = req->axiz;
  int frames_num = req->num;
  float time_per_cmd = req->time;
  int have_cmd_s = frames_num / frames_per_cmd;
  int have_cmd_m = frames_num % frames_per_cmd;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[ArmContr] mv_servo_joint %d %f", frames_num, time_per_cmd);

  float frames[3 * 12] = {0};
  float mvtime[3] = {0};
  for (int i = 0; i < frames_per_cmd; i++) mvtime[i] = time_per_cmd;

  // set arm sleep to wait for command
  if (req->plan_delay > 0) {
    ret = armapi->plan_sleep(req->plan_delay);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[ArmContr] plan_sleep %d, delya:%f", ret, req->plan_delay);
  }

  for (int i = 0; i < have_cmd_s; i++) {
    for (int j = 0; j < frames_per_cmd * axiz; j++) frames[j] = req->frames[frames_index + j];
    frames_index = frames_index + frames_per_cmd * axiz;
    ret = armapi->moveto_servo_joint(frames_per_cmd, frames, mvtime);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[ArmContr] moveto_servo_joint %d, i: %d", ret, i);
    if (ret != 0) {
      res->ret = ret;
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] moveto_servo_joint failed %d", ret);
      return;
    }
  }

  // last time send
  for (size_t i = 0; i < have_cmd_m * axiz; i++) frames[i] = req->frames[frames_index + i];
  ret = armapi->moveto_servo_joint(have_cmd_m, frames, mvtime);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[ArmContr] moveto_servo_joint %d, i: %d", ret, have_cmd_s);
  if (ret != 0) {
    res->ret = ret;
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] moveto_servo_joint failed %d", ret);
    return;
  }

  res->ret = ret;
}

void set_motion_status(std::shared_ptr<ut_msg::srv::SetInt16::Request> req,
                       std::shared_ptr<ut_msg::srv::SetInt16::Response> res) {
  CHECK_ARM_CONNECT()

  int ret = armapi->set_motion_status(req->data);
  if (ret != 0 && ret != -4) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] set_motion_status failed %d", ret);
  res->ret = ret;
}

void get_motion_status(std::shared_ptr<ut_msg::srv::GetInt16::Request> req,
                       std::shared_ptr<ut_msg::srv::GetInt16::Response> res) {
  CHECK_ARM_CONNECT()

  uint8_t status;
  int ret = armapi->get_motion_status(&status);
  if (ret != 0 && ret != -4) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] get_motion_status failed %d", ret);
  res->ret = ret;
  res->data = status;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[ArmContr] status_get %d, status %d", ret, status);
}

void set_motion_mode(std::shared_ptr<ut_msg::srv::SetInt16::Request> req,
                     std::shared_ptr<ut_msg::srv::SetInt16::Response> res) {
  CHECK_ARM_CONNECT()

  int ret = armapi->set_motion_mode(req->data);
  if (ret != 0 && ret != -4) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] set_motion_mode failed %d", ret);
  res->ret = ret;
}

void get_motion_mode(std::shared_ptr<ut_msg::srv::GetInt16::Request> req,
                     std::shared_ptr<ut_msg::srv::GetInt16::Response> res) {
  CHECK_ARM_CONNECT()

  uint8_t mode;
  int ret = armapi->get_motion_mode(&mode);
  if (ret != 0 && ret != -4) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] get_motion_mode failed %d", ret);
  res->ret = ret;
  res->data = mode;
}

void set_motion_enable(std::shared_ptr<ut_msg::srv::SetEnable::Request> req,
                       std::shared_ptr<ut_msg::srv::SetEnable::Response> res) {
  CHECK_ARM_CONNECT()

  int ret = armapi->set_motion_enable(req->axis, req->enable);
  if (ret != 0 && ret != -4) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] set_motion_enable failed %d", ret);
  res->ret = ret;
}

void get_motion_enable(std::shared_ptr<ut_msg::srv::GetInt16::Request> req,
                       std::shared_ptr<ut_msg::srv::GetInt16::Response> res) {
  CHECK_ARM_CONNECT()

  int enable;
  int ret = armapi->get_motion_enable(&enable);
  if (ret != 0 && ret != -4) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] get_motion_enable failed %d", ret);
  res->ret = ret;
  res->data = enable;
}

void get_error_code(std::shared_ptr<ut_msg::srv::GetUint16s::Request> req,
                    std::shared_ptr<ut_msg::srv::GetUint16s::Response> res) {
  CHECK_ARM_CONNECT()

  uint8_t array[24] = {0};
  int ret = armapi->get_error_code(array);
  if (ret != 0 && ret != -4) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] get_error_code failed %d", ret);
  res->ret = ret;
  for (size_t j = 0; j < 24; j++) res->data.push_back(array[j]);
}

void get_servo_msg(std::shared_ptr<ut_msg::srv::GetUint16s::Request> req,
                   std::shared_ptr<ut_msg::srv::GetUint16s::Response> res) {
  CHECK_ARM_CONNECT()

  uint8_t array[24] = {0};
  int ret = armapi->get_servo_msg(array);
  if (ret != 0 && ret != -4) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] get_servo_msg failed %d", ret);
  res->ret = ret;
  for (size_t j = 0; j < 24; j++) res->data.push_back(array[j]);
}

void moveto_joint_p2p(std::shared_ptr<ut_msg::srv::MovetoJointP2p::Request> req,
                      std::shared_ptr<ut_msg::srv::MovetoJointP2p::Response> res) {
  CHECK_ARM_CONNECT()

  int axis = req->joints.size();
  if (axis == 6) {
    float joints[6] = {req->joints[0], req->joints[1], req->joints[2], req->joints[3], req->joints[4], req->joints[5]};
    int ret = armapi->moveto_joint_p2p(joints, req->speed, req->acc, 0);
    if (ret != 0 && ret != -4)
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] moveto_joint_p2p1 failed %d", ret);
    res->ret = ret;
  } else if (axis == 7) {
    float joints[7] = {req->joints[0], req->joints[1], req->joints[2], req->joints[3],
                       req->joints[4], req->joints[5], req->joints[6]};
    int ret = armapi->moveto_joint_p2p(joints, req->speed, req->acc, 0);
    if (ret != 0 && ret != -4)
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] moveto_joint_p2p2 failed %d", ret);
    res->ret = ret;
  } else {
    res->ret = -3;
  }
}

void moveto_cartesian_line(std::shared_ptr<ut_msg::srv::MovetoCartesianLine::Request> req,
                           std::shared_ptr<ut_msg::srv::MovetoCartesianLine::Response> res) {
  CHECK_ARM_CONNECT()

  int axis = req->pose.size();
  if (axis == 6 || axis == 7) {
    float pose[6] = {req->pose[0], req->pose[1], req->pose[2], req->pose[3], req->pose[4], req->pose[5]};
    int ret = armapi->moveto_cartesian_line(pose, req->speed, req->acc, 0);
    if (ret != 0 && ret != -4)
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] moveto_cartesian_line failed %d", ret);
    res->ret = ret;
  } else {
    res->ret = -3;
  }
}

void plan_sleep(std::shared_ptr<ut_msg::srv::SetFloat32::Request> req,
                std::shared_ptr<ut_msg::srv::SetFloat32::Response> res) {
  CHECK_ARM_CONNECT()

  int ret = armapi->plan_sleep(req->data);
  if (ret != 0 && ret != -4) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] plan_sleep failed %d", ret);
  res->ret = ret;
}

void moveto_cartesian_lineb(std::shared_ptr<ut_msg::srv::MovetoCartesianLineb::Request> req,
                            std::shared_ptr<ut_msg::srv::MovetoCartesianLineb::Response> res) {
  CHECK_ARM_CONNECT()

  int axis = req->pose.size();
  if (axis == 6 || axis == 7) {
    float pose[6] = {req->pose[0], req->pose[1], req->pose[2], req->pose[3], req->pose[4], req->pose[5]};
    int ret = armapi->moveto_cartesian_lineb(pose, req->speed, req->acc, 0, req->radii);
    if (ret != 0 && ret != -4)
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] moveto_cartesian_lineb failed %d", ret);
    res->ret = ret;

  } else {
    res->ret = -3;
  }
}

void get_joint_actual_pos(std::shared_ptr<ut_msg::srv::GetFloat32s::Request> req,
                          std::shared_ptr<ut_msg::srv::GetFloat32s::Response> res) {
  CHECK_ARM_CONNECT()

  float array[10] = {0};
  int ret = armapi->get_joint_target_pos(array);
  if (ret != 0 && ret != -4)
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] get_joint_target_pos failed %d", ret);
  res->ret = ret;
  for (size_t j = 0; j < 10; j++) res->data.push_back(array[j]);
}

void set_limit_angle_enable(std::shared_ptr<ut_msg::srv::SetInt16::Request> req,
                            std::shared_ptr<ut_msg::srv::SetInt16::Response> res) {
  CHECK_ARM_CONNECT()

  int ret = armapi->set_limit_angle_enable(req->data);
  if (ret != 0 && ret != -4)
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] set_limit_angle_enable failed %d", ret);
  res->ret = ret;
}

void set_limit_geometry_enable(std::shared_ptr<ut_msg::srv::SetInt16::Request> req,
                               std::shared_ptr<ut_msg::srv::SetInt16::Response> res) {
  CHECK_ARM_CONNECT()

  int ret = armapi->set_limit_geometry_enable(req->data);
  if (ret != 0 && ret != -4)
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] set_limit_geometry_enable failed %d", ret);
  res->ret = ret;
}

//------------------------------------------------------------------------------------
//                                      FLXIE
//------------------------------------------------------------------------------------
#define CHECK_FLXIE_CONNECT()                                                                          \
  if (armapi == NULL || fixie == NULL) {                                                               \
    res->ret = -3;                                                                                     \
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] server have not connected arm or gripper"); \
    return;                                                                                            \
  }

void gripper_set_state(std::shared_ptr<ut_msg::srv::SetGripperState::Request> req,
                       std::shared_ptr<ut_msg::srv::SetGripperState::Response> res) {
  if (armapi == NULL) {
    res->ret = -3;
    return;
  }

  if (fixie == NULL) fixie = new UtraFlxiE2Api(armapi, 101);

  if (req->state == 1) {
    float value;
    int ret = fixie->set_motion_mode(1);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[ArmContr] gripper_state_set1: %d\n", ret);
    ret = fixie->set_motion_enable(1);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[ArmContr] gripper_state_set2: %d\n", ret);
    ret = fixie->get_pos_target(&value);
    if (ret != 0 && ret != -4)
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] flixe get_pos_target failed %d", ret);

    res->pos = value;
    res->ret = ret;
  } else {
    int ret = fixie->set_motion_mode(0);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[ArmContr] gripper_state_set3: %d\n", ret);
    ret = fixie->set_motion_enable(0);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[ArmContr] gripper_state_set4: %d\n", ret);
    if (ret != 0 && ret != -4)
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] flixe set_motion_enable failed %d", ret);
    res->ret = ret;
  }
}

void gripper_get_state(std::shared_ptr<ut_msg::srv::GetGripperState::Request> req,
                       std::shared_ptr<ut_msg::srv::GetGripperState::Response> res) {
  res->enable = 0;
  CHECK_FLXIE_CONNECT()

  float value;
  int ret = fixie->get_pos_target(&value);
  if (ret != -3) {
    // ros::NodeHandle n;
    float pos = (value / 100.0) - 0.4;
    // TODO:Jimy
    // n.setParam("utra_gripper_pos", pos);
  }

  uint8_t enable;
  ret = fixie->get_motion_enable(&enable);
  if (ret != 0 && ret != -4)
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] flixe get_motion_enable failed %d", ret);
  res->ret = ret;
  res->enable = enable;
  res->pos = value;
}

void gripper_set_pos_target(std::shared_ptr<ut_msg::srv::SetFloat32::Request> req,
                            std::shared_ptr<ut_msg::srv::SetFloat32::Response> res) {
  CHECK_FLXIE_CONNECT()

  int ret = fixie->set_pos_target(req->data);
  if (ret != 0 && ret != -4)
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] flixe set_pos_target failed %d", ret);
  if (ret != -3) {
    // ros::NodeHandle n;
    float pos = (req->data / 100) - 0.4;
    // TODO:Jimy
    // n.setParam("utra_gripper_pos", pos);
  }
  res->ret = ret;
}

void gripper_set_vel_limit(std::shared_ptr<ut_msg::srv::SetFloat32::Request> req,
                           std::shared_ptr<ut_msg::srv::SetFloat32::Response> res) {
  CHECK_FLXIE_CONNECT()

  int ret1 = fixie->set_vel_limit_min(-req->data, true);
  int ret2 = fixie->set_vel_limit_max(req->data, true);
  res->ret = ret1 + ret2;
  if (res->ret != 0)
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] flixe set_vel_limit_max failed %d %d", ret1, ret2);
}

void gripper_get_vel_limit(std::shared_ptr<ut_msg::srv::GetFloat32::Request> req,
                           std::shared_ptr<ut_msg::srv::GetFloat32::Response> res) {
  CHECK_FLXIE_CONNECT()

  float value;
  int ret1 = fixie->get_vel_limit_min(&value);
  int ret2 = fixie->get_vel_limit_max(&value);
  res->ret = ret1 + ret2;
  if (res->ret != 0)
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] flixe get_vel_limit_max failed %d %d", ret1, ret2);
  res->data = value;
}

void gripper_set_acc(std::shared_ptr<ut_msg::srv::SetFloat32::Request> req,
                     std::shared_ptr<ut_msg::srv::SetFloat32::Response> res) {
  CHECK_FLXIE_CONNECT()

  float data = req->data;
  if (req->data < 0) {
    res->ret = -3;
    return;
  }
  if (req->data > 300) data = 300;

  int ret = fixie->set_pos_adrc_param(3, data);
  if (ret != 0 && ret != -4)
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] flixe set_pos_adrc_param failed %d", ret);
  res->ret = ret;
}

void gripper_get_acc(std::shared_ptr<ut_msg::srv::GetFloat32::Request> req,
                     std::shared_ptr<ut_msg::srv::GetFloat32::Response> res) {
  CHECK_FLXIE_CONNECT()

  float value;
  int ret = fixie->get_pos_adrc_param(3, &value);
  if (ret != 0 && ret != -4)
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] flixe get_pos_adrc_param failed %d", ret);
  res->ret = ret;
  res->data = value;
}

void gripper_get_error_code(std::shared_ptr<ut_msg::srv::GetInt16::Request> req,
                            std::shared_ptr<ut_msg::srv::GetInt16::Response> res) {
  CHECK_FLXIE_CONNECT()

  uint8_t value;
  int ret = fixie->get_error_code(&value);
  if (ret != 0 && ret != -4)
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] flixe get_error_code failed %d", ret);
  res->ret = ret;
  res->data = value;
}

void gripper_reset_err(std::shared_ptr<ut_msg::srv::GetInt16::Request> req,
                       std::shared_ptr<ut_msg::srv::GetInt16::Response> res) {
  CHECK_FLXIE_CONNECT()

  int ret = fixie->reset_err();
  if (ret != 0 && ret != -4) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] flixe reset_err failed %d", ret);
  res->ret = ret;
  res->data = 0;
}

void gripper_set_unlock(std::shared_ptr<ut_msg::srv::SetInt16::Request> req,
                        std::shared_ptr<ut_msg::srv::SetInt16::Response> res) {
  CHECK_FLXIE_CONNECT()

  int data = req->data;
  int ret = fixie->set_unlock_function(data);
  if (ret != 0 && ret != -4)
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[ArmContr] flixe set_unlock_function failed %d", ret);
  res->ret = ret;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("utarm_api_server");
  std::string arm_ns = node->declare_parameter<std::string>("arm_ns", "utarm");
  arm_ip = node->declare_parameter<std::string>("arm_ip", "192.168.1.1");
  std::string ns = arm_ns + "/" + "apisrv/";
  RCLCPP_INFO(node->get_logger(), "arm_ns: %s", ns.c_str());
  RCLCPP_INFO(node->get_logger(), "arm_ip: %s", arm_ip.c_str());

  uint8_t axis;
  armapi = new UtraApiTcp((char *)(arm_ip.c_str()));
  if (armapi->is_error()) {
    RCLCPP_ERROR(node->get_logger(), "arm connection failed: %s", arm_ip.c_str());
    return 0;
  }

  int ret = armapi->get_axis(&axis);
  if (ret != 0 && ret != -4) {
    RCLCPP_ERROR(node->get_logger(), "arm connection failed: %s", arm_ip.c_str());
    return 0;
  }
  RCLCPP_INFO(node->get_logger(), "arm connection successful: %s", arm_ip.c_str());

  rclcpp::Service<ut_msg::srv::CheckConnect>::SharedPtr check_connect_ =
      node->create_service<ut_msg::srv::CheckConnect>(ns + "check_connect", &check_connect);
  rclcpp::Service<ut_msg::srv::Connect>::SharedPtr connect_ =
      node->create_service<ut_msg::srv::Connect>(ns + "connect", &connect);
  rclcpp::Service<ut_msg::srv::Disconnect>::SharedPtr disconnect_ =
      node->create_service<ut_msg::srv::Disconnect>(ns + "disconnect", &disconnect);

  rclcpp::Service<ut_msg::srv::SetInt16>::SharedPtr set_motion_status_ =
      node->create_service<ut_msg::srv::SetInt16>(ns + "set_motion_status", &set_motion_status);
  rclcpp::Service<ut_msg::srv::GetInt16>::SharedPtr get_motion_status_ =
      node->create_service<ut_msg::srv::GetInt16>(ns + "get_motion_status", &get_motion_status);
  rclcpp::Service<ut_msg::srv::SetInt16>::SharedPtr set_motion_mode_ =
      node->create_service<ut_msg::srv::SetInt16>(ns + "set_motion_mode", &set_motion_mode);
  rclcpp::Service<ut_msg::srv::GetInt16>::SharedPtr get_motion_mode_ =
      node->create_service<ut_msg::srv::GetInt16>(ns + "get_motion_mode", &get_motion_mode);

  rclcpp::Service<ut_msg::srv::SetEnable>::SharedPtr set_motion_enable_ =
      node->create_service<ut_msg::srv::SetEnable>(ns + "set_motion_enable", &set_motion_enable);
  rclcpp::Service<ut_msg::srv::GetInt16>::SharedPtr get_motion_enable_ =
      node->create_service<ut_msg::srv::GetInt16>(ns + "get_motion_enable", &get_motion_enable);

  rclcpp::Service<ut_msg::srv::GetUint16s>::SharedPtr get_error_code_ =
      node->create_service<ut_msg::srv::GetUint16s>(ns + "get_error_code", &get_error_code);
  rclcpp::Service<ut_msg::srv::GetUint16s>::SharedPtr get_servo_msg_ =
      node->create_service<ut_msg::srv::GetUint16s>(ns + "get_servo_msg", &get_servo_msg);

  rclcpp::Service<ut_msg::srv::MovetoJointP2p>::SharedPtr moveto_joint_p2p_ =
      node->create_service<ut_msg::srv::MovetoJointP2p>(ns + "moveto_joint_p2p", &moveto_joint_p2p);
  rclcpp::Service<ut_msg::srv::MovetoCartesianLine>::SharedPtr moveto_cartesian_line_ =
      node->create_service<ut_msg::srv::MovetoCartesianLine>(ns + "moveto_cartesian_line", &moveto_cartesian_line);
  rclcpp::Service<ut_msg::srv::MovetoServoJoint>::SharedPtr moveto_servo_joint_ =
      node->create_service<ut_msg::srv::MovetoServoJoint>(ns + "moveto_servo_joint", &moveto_servo_joint);
  rclcpp::Service<ut_msg::srv::MovetoCartesianLineb>::SharedPtr moveto_cartesian_lineb_ =
      node->create_service<ut_msg::srv::MovetoCartesianLineb>(ns + "moveto_cartesian_lineb", &moveto_cartesian_lineb);
  rclcpp::Service<ut_msg::srv::SetFloat32>::SharedPtr plan_sleep_ =
      node->create_service<ut_msg::srv::SetFloat32>(ns + "plan_sleep", &plan_sleep);

  rclcpp::Service<ut_msg::srv::GetFloat32s>::SharedPtr get_joint_actual_pos_ =
      node->create_service<ut_msg::srv::GetFloat32s>(ns + "get_joint_actual_pos", &get_joint_actual_pos);
  rclcpp::Service<ut_msg::srv::SetInt16>::SharedPtr set_limit_angle_enable_ =
      node->create_service<ut_msg::srv::SetInt16>(ns + "set_limit_angle_enable", &set_limit_angle_enable);
  rclcpp::Service<ut_msg::srv::SetInt16>::SharedPtr set_limit_geometry_enable_ =
      node->create_service<ut_msg::srv::SetInt16>(ns + "set_limit_geometry_enable", &set_limit_geometry_enable);

  rclcpp::Service<ut_msg::srv::SetGripperState>::SharedPtr gripper_set_state_ =
      node->create_service<ut_msg::srv::SetGripperState>(ns + "gripper_set_state", &gripper_set_state);
  rclcpp::Service<ut_msg::srv::GetGripperState>::SharedPtr gripper_get_state_ =
      node->create_service<ut_msg::srv::GetGripperState>(ns + "gripper_get_state", &gripper_get_state);
  rclcpp::Service<ut_msg::srv::SetFloat32>::SharedPtr gripper_set_pos_target_ =
      node->create_service<ut_msg::srv::SetFloat32>(ns + "gripper_set_pos_target", &gripper_set_pos_target);
  rclcpp::Service<ut_msg::srv::SetFloat32>::SharedPtr gripper_set_vel_limit_ =
      node->create_service<ut_msg::srv::SetFloat32>(ns + "gripper_set_vel_limit", &gripper_set_vel_limit);
  rclcpp::Service<ut_msg::srv::GetFloat32>::SharedPtr gripper_get_vel_limit_ =
      node->create_service<ut_msg::srv::GetFloat32>(ns + "gripper_get_vel_limit", &gripper_get_vel_limit);
  rclcpp::Service<ut_msg::srv::SetFloat32>::SharedPtr gripper_set_acc_ =
      node->create_service<ut_msg::srv::SetFloat32>(ns + "gripper_set_acc", &gripper_set_acc);
  rclcpp::Service<ut_msg::srv::GetFloat32>::SharedPtr gripper_get_acc_ =
      node->create_service<ut_msg::srv::GetFloat32>(ns + "gripper_get_acc", &gripper_get_acc);
  rclcpp::Service<ut_msg::srv::GetInt16>::SharedPtr gripper_get_error_code_ =
      node->create_service<ut_msg::srv::GetInt16>(ns + "gripper_get_error_code", &gripper_get_error_code);
  rclcpp::Service<ut_msg::srv::GetInt16>::SharedPtr gripper_reset_err_ =
      node->create_service<ut_msg::srv::GetInt16>(ns + "gripper_reset_err", &gripper_reset_err);
  rclcpp::Service<ut_msg::srv::SetInt16>::SharedPtr gripper_set_unlock_ =
      node->create_service<ut_msg::srv::SetInt16>(ns + "gripper_set_unlock", &gripper_set_unlock);

  rclcpp::spin(node);
  rclcpp::shutdown();
}
