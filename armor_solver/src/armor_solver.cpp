// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
// Copyright (C) FYT Vision Group. All rights reserved.
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

#include "armor_solver/armor_solver.hpp"
// std
#include <cmath>
#include <cstddef>
#include <stdexcept>
// project
#include "armor_solver/armor_solver_node.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace fyt::auto_aim {
Solver::Solver(std::weak_ptr<rclcpp::Node> n) : node_(n) {
  auto node = node_.lock();
  
  shooting_range_w_ = node->declare_parameter("solver.shooting_range_width", 0.135);
  shooting_range_h_ = node->declare_parameter("solver.shooting_range_height", 0.405);
  max_tracking_v_yaw_ = node->declare_parameter("solver.max_tracking_v_yaw", 6.0);
  prediction_delay_ = node->declare_parameter("solver.prediction_delay", 0.0);
  controller_delay_ = node->declare_parameter("solver.controller_delay", 0.0);
  side_angle_ = node->declare_parameter("solver.side_angle", 15.0);
  min_switching_v_yaw_ = node->declare_parameter("solver.min_switching_v_yaw", 1.0);

  std::string compenstator_type = node->declare_parameter("solver.compensator_type", "ideal");
  trajectory_compensator_ = CompensatorFactory::createCompensator(compenstator_type);
  trajectory_compensator_->iteration_times = node->declare_parameter("solver.iteration_times", 20);
  trajectory_compensator_->velocity = node->declare_parameter("solver.bullet_speed", 20.0);
  trajectory_compensator_->gravity = node->declare_parameter("solver.gravity", 9.8);
  trajectory_compensator_->resistance = node->declare_parameter("solver.resistance", 0.001);


  manual_compensator_ = std::make_unique<ManualCompensator>();
  auto angle_offset = node->declare_parameter("solver.angle_offset", std::vector<std::string>{});
  if(!manual_compensator_->updateMapFlow(angle_offset)) {
    FYT_WARN("armor_solver", "Manual compensator update failed!");
  }

  state = State::TRACKING_ARMOR;
  overflow_count_ = 0;
  transfer_thresh_ = 5;

  node.reset();
}

rm_interfaces::msg::GimbalCmd Solver::solve(const rm_interfaces::msg::Target &target,
                                         const rclcpp::Time &current_time,
                                         std::shared_ptr<tf2_ros::Buffer> tf2_buffer_) {
  // --- [第 1 步: 获取参数和云台当前状态] ---
  try {
    auto node = node_.lock();
    max_tracking_v_yaw_ = node->get_parameter("solver.max_tracking_v_yaw").as_double();
    prediction_delay_ = node->get_parameter("solver.prediction_delay").as_double();
    controller_delay_ = node->get_parameter("solver.controller_delay").as_double();
    side_angle_ = node->get_parameter("solver.side_angle").as_double();
    min_switching_v_yaw_ = node->get_parameter("solver.min_switching_v_yaw").as_double();
    node.reset();
  } catch (const std::runtime_error &e) {
    FYT_ERROR("armor_solver", "{}", e.what());
  }

  // 获取云台的 TF 变换 (从 odom 到 gimbal_link)
  geometry_msgs::msg::TransformStamped gimbal_tf;
  try {
    gimbal_tf =
      tf2_buffer_->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    FYT_ERROR("armor_solver", "{}", ex.what());
    throw ex;
  }
  
  // --- [第 2 步: 关键修正 - 提取云台在 odom 中的位置] ---
  auto gimbal_translation = gimbal_tf.transform.translation;
  Eigen::Vector3d gimbal_pos_odom(
    gimbal_translation.x,
    gimbal_translation.y,
    gimbal_translation.z
  );

  // 从 TF 中获取云台当前的 RPY (Roll, Pitch, Yaw)
  auto msg_q = gimbal_tf.transform.rotation;
  tf2::Quaternion tf_q;
  tf2::fromMsg(msg_q, tf_q);
  tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
  rpy_[1] = -rpy_[1]; // 你的自定义约定


  // --- [第 3 步: 迭代求解 1 - 主迭代循环] ---
  
  // a. 获取 EKF 状态 (在 odom 坐标系)
  Eigen::Vector3d base_center_pos(target.position.x, target.position.y, target.position.z);
  Eigen::Vector3d base_center_vel(target.velocity.x, target.velocity.y, target.velocity.z);
  double base_center_yaw = target.yaw;
  double base_center_v_yaw = target.v_yaw;

  // b. 计算基础延迟
  double base_dt =
    (current_time - rclcpp::Time(target.header.stamp)).seconds() + prediction_delay_;

  // c. 迭代求解
  int num_iterations = 3;
  double flying_time_guess = 0.0;
  Eigen::Vector3d chosen_armor_position; // 我们要求的最终目标点 (odom 系)
  Eigen::Vector3d predicted_center_odom; // 迭代后的中心点 (odom 系)
  
  for (int i = 0; i < num_iterations; ++i) {
    double total_dt = base_dt + flying_time_guess;

    predicted_center_odom = base_center_pos + total_dt * base_center_vel;
    double predicted_yaw_odom = base_center_yaw + total_dt * base_center_v_yaw;

    // `selectBestArmor` 需要车辆中心相对于【云台】的向量
    Eigen::Vector3d relative_center_vec = predicted_center_odom - gimbal_pos_odom;

    std::vector<Eigen::Vector3d> armor_positions_odom = getArmorPositions(
        predicted_center_odom, predicted_yaw_odom, target.radius_1, target.radius_2, 
        target.d_zc, target.d_za, target.armors_num);
        
    int idx = selectBestArmor(
        armor_positions_odom, relative_center_vec, predicted_yaw_odom,
        target.v_yaw, target.armors_num);
        
    chosen_armor_position = armor_positions_odom.at(idx);

    // 计算从【云台】到【未来目标】的向量
    Eigen::Vector3d target_vec = chosen_armor_position - gimbal_pos_odom;

    // 检查到【云台】的距离
    if (target_vec.norm() < 0.1) {
       throw std::runtime_error("No valid armor to shoot (in iteration)");
    }

    // [核心] 用这个向量计算新的、更准的飞行时间
    flying_time_guess = trajectory_compensator_->getFlyingTime(target_vec);
  }
  
  // --- [第 4 步: 计算 Yaw/Pitch] ---
  Eigen::Vector3d target_vec_odom = chosen_armor_position - gimbal_pos_odom;
  double yaw, pitch;
  calcYawAndPitch(target_vec_odom, rpy_, yaw, pitch);
  double distance = target_vec_odom.norm();

  // 更新调试用的预测点
  predicted_position_.x = chosen_armor_position.x();
  predicted_position_.y = chosen_armor_position.y();
  predicted_position_.z = chosen_armor_position.z();
  
  // --- [第 5 步: 填充指令和状态机] ---
  rm_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header = target.header;
  gimbal_cmd.distance = distance;
  gimbal_cmd.fire_advice = isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance);

  switch (state) {
    case TRACKING_ARMOR: {
      if (std::abs(target.v_yaw) > max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_CENTER;
      }

      // --- [第 6 步: 你的 `controller_delay_` 逻辑 (非迭代)] ---
      //
      // 遵从你的要求，这里不使用迭代，
      // 而是使用“单次预测”逻辑，但应用了坐标系修正
      //
      if (controller_delay_ != 0) {
        
        // a. 使用主迭代的 flying_time_guess 作为“最好的猜测”
        double total_dt_delayed = base_dt + controller_delay_ + flying_time_guess;

        // b. 执行【一次】预测
        Eigen::Vector3d predicted_center_delayed = base_center_pos + total_dt_delayed * base_center_vel;
        double predicted_yaw_delayed = base_center_yaw + total_dt_delayed * base_center_v_yaw;

        // c. [修正] 传递相对向量给 selectBestArmor
        Eigen::Vector3d relative_center_delayed = predicted_center_delayed - gimbal_pos_odom;

        std::vector<Eigen::Vector3d> armor_positions_delayed = getArmorPositions(
            predicted_center_delayed, predicted_yaw_delayed, target.radius_1, target.radius_2, 
            target.d_zc, target.d_za, target.armors_num);
        
        int idx_delayed = selectBestArmor(
            armor_positions_delayed, relative_center_delayed, predicted_yaw_delayed, 
            target.v_yaw, target.armors_num);
            
        Eigen::Vector3d chosen_armor_pos_delayed = armor_positions_delayed.at(idx_delayed);

        // d. [覆盖] 覆盖 yaw, pitch, distance
        Eigen::Vector3d delayed_target_vec = chosen_armor_pos_delayed - gimbal_pos_odom;
        gimbal_cmd.distance = delayed_target_vec.norm(); // 覆盖
        calcYawAndPitch(delayed_target_vec, rpy_, yaw, pitch); // 覆盖 yaw 和 pitch
        
        // e. [覆盖] 覆盖调试用的预测点
        predicted_position_.x = chosen_armor_pos_delayed.x();
        predicted_position_.y = chosen_armor_pos_delayed.y();
        predicted_position_.z = chosen_armor_pos_delayed.z();
        
        // f. [覆盖] 覆盖最终补偿要用的 `chosen_armor_position`
        chosen_armor_position = chosen_armor_pos_delayed;
      }
      break;
    }
    case TRACKING_CENTER: {
      if (std::abs(target.v_yaw) < max_tracking_v_yaw_) {
         overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_ARMOR;
        overflow_count_ = 0;
      }
      gimbal_cmd.fire_advice = true;
      
      // [修正] 瞄准中心时，也必须使用相对向量
      Eigen::Vector3d center_vec_odom = predicted_center_odom - gimbal_pos_odom;
      calcYawAndPitch(center_vec_odom, rpy_, yaw, pitch);
      break;
    }
  }

  // --- [第 7 步: 最终补偿和发送] ---
  
  // [修正] `angleHardCorrect` 也应该使用相对距离
  double final_target_dist_xy = (chosen_armor_position - gimbal_pos_odom).head(2).norm();
  double final_target_dist_z = (chosen_armor_position - gimbal_pos_odom).z();

  auto angle_offset = manual_compensator_->angleHardCorrect(final_target_dist_xy, final_target_dist_z);
  double pitch_offset = angle_offset[0] * M_PI / 180;
  double yaw_offset = angle_offset[1] * M_PI / 180;
  double cmd_pitch = pitch + pitch_offset;
  double cmd_yaw = angles::normalize_angle(yaw + yaw_offset);
  
  // 0820 调试代码 (保留你的逻辑)
  predicted_position_.y += -tan(yaw_offset)*predicted_position_.x;
  predicted_position_.z += -tan(((1/45)*M_PI))*(chosen_armor_position.head(2).norm()) - tan(pitch_offset)*(chosen_armor_position.head(2).norm());

  gimbal_cmd.yaw = cmd_yaw * 180 / M_PI;
  gimbal_cmd.pitch = cmd_pitch * 180 / M_PI - 4.0;  
  gimbal_cmd.yaw_diff = (cmd_yaw - rpy_[2]) * 180 / M_PI;
  gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180 / M_PI;

  if (gimbal_cmd.fire_advice) {
    FYT_DEBUG("armor_solver", "You Need Fire!");
  }
  return gimbal_cmd;
}

bool Solver::isOnTarget(const double cur_yaw,
                        const double cur_pitch,
                        const double target_yaw,
                        const double target_pitch,
                        const double distance) const noexcept {
  // Judge whether to shoot
  double shooting_range_yaw = std::abs(atan2(shooting_range_w_ / 2, distance));
  double shooting_range_pitch = std::abs(atan2(shooting_range_h_ / 2, distance));
  // Limit the shooting area to 1 degree to avoid not shooting when distance is
  // too large
  shooting_range_yaw = std::max(shooting_range_yaw, 1.0 * M_PI / 180);
  shooting_range_pitch = std::max(shooting_range_pitch, 1.0 * M_PI / 180);
  if (std::abs(cur_yaw - target_yaw) < shooting_range_yaw &&
      std::abs(cur_pitch - target_pitch) < shooting_range_pitch) {
    return true;
  }

  return false;
}

std::vector<Eigen::Vector3d> Solver::getArmorPositions(const Eigen::Vector3d &target_center,
                                                       const double target_yaw,
                                                       const double r1,
                                                       const double r2,
                                                       const double d_zc,
                                                       const double d_za,
                                                       const size_t armors_num) const noexcept {
  auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
  // Calculate the position of each armor
  bool is_current_pair = true;
  double r = 0., target_dz = 0.;
  for (size_t i = 0; i < armors_num; i++) {
    double temp_yaw = target_yaw + i * (2 * M_PI / armors_num);
    if (armors_num == 4) {
      r = is_current_pair ? r1 : r2;
      target_dz = d_zc + (is_current_pair ? 0 : d_za);
      is_current_pair = !is_current_pair;
    } else {
      r = r1;
      target_dz = d_zc;
    }
    armor_positions[i] =
      target_center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
  }
  return armor_positions;
}

int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                            const Eigen::Vector3d &target_center,
                            const double target_yaw,
                            const double target_v_yaw,
                            const size_t armors_num) const noexcept {
  // Angle between the car's center and the X-axis
  double alpha = std::atan2(target_center.y(), target_center.x());
  // Angle between the front of observed armor and the X-axis
  double beta = target_yaw;

  // clang-format off
  Eigen::Matrix2d R_odom2center;
  Eigen::Matrix2d R_odom2armor;
  R_odom2center << std::cos(alpha), std::sin(alpha), 
                  -std::sin(alpha), std::cos(alpha);
  R_odom2armor << std::cos(beta), std::sin(beta), 
                 -std::sin(beta), std::cos(beta);
  // clang-format on
  Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;

  // Equal to (alpha - beta) in most cases
  double decision_angle = -std::asin(R_center2armor(0, 1));

  // Angle thresh of the armor jump
  double theta = (target_v_yaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;

  // Avoid the frequent switch between two armor
  if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
    theta = 0;
  }

  double temp_angle = decision_angle + M_PI / armors_num - theta;

  if (temp_angle < 0) {
    temp_angle += 2 * M_PI;
  }

  int selected_id = static_cast<int>(temp_angle / (2 * M_PI / armors_num));
  return selected_id;
}

void Solver::calcYawAndPitch(const Eigen::Vector3d &p,
                             const std::array<double, 3> rpy,
                             double &yaw,
                             double &pitch) const noexcept {
  // Calculate yaw and pitch
  yaw = atan2(p.y(), p.x());
  pitch = atan2(p.z(), p.head(2).norm());

  if (double temp_pitch = pitch; trajectory_compensator_->compensate(p, temp_pitch)) {
    pitch = temp_pitch;
  }
}

std::vector<std::pair<double, double>> Solver::getTrajectory() const noexcept {
  auto trajectory = trajectory_compensator_->getTrajectory(15, rpy_[1]);
  // Rotate
  for (auto &p : trajectory) {
    double x = p.first;
    double y = p.second;
    p.first = x * cos(rpy_[1]) + y * sin(rpy_[1]);
    p.second = -x * sin(rpy_[1]) + y * cos(rpy_[1]);
  }
  return trajectory;
}

geometry_msgs::msg::Point fyt::auto_aim::Solver::getPredictedPosition() const noexcept {
  return predicted_position_;
}

}  // namespace fyt::auto_aim
