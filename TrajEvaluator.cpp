#include "TrajEvaluator.h"
#include <cmath>
#include <limits>
#include <algorithm>

namespace planning {

    TrajEvaluator::TrajEvaluator(const common::VehicleParams& vehicle_params, const common::LaneParams& lane_params)
        : vehicle_params_(vehicle_params), lane_params_(lane_params) {
        // 初始化车辆状态
        vehicle_state_.x = 0.0;
        vehicle_state_.y = 0.0;
        vehicle_state_.heading = 0.0;
        vehicle_state_.velocity = 0.0;
        vehicle_state_.current_lane = 0;
    }

    void TrajEvaluator::setVehicleState(const common::VehicleState& state) {
        vehicle_state_ = state;
    }

    void TrajEvaluator::setObstacles(const std::vector<common::Obstacle>& obstacles) {
        obstacles_ = obstacles;
    }

    void TrajEvaluator::calculateCosts(std::vector<common::Trajectory>& trajectories) {
        for (auto& trajectory : trajectories) {
            trajectory.cost = calculateTrajectoryCost(trajectory);
        }
    }

    common::Trajectory TrajEvaluator::selectOptimalTrajectory(const std::vector<common::Trajectory>& trajectories) {
        // 如果没有可行轨迹，返回第一条轨迹并标记为碰撞
        if (trajectories.empty()) {
            common::Trajectory empty_trajectory;
            empty_trajectory.has_collision = true;
            return empty_trajectory;
        }

        // 首先筛选出无碰撞的轨迹
        std::vector<common::Trajectory> feasible_trajectories;
        for (const auto& trajectory : trajectories) {
            if (!trajectory.has_collision) {
                feasible_trajectories.push_back(trajectory);
            }
        }

        // 如果没有无碰撞的轨迹，选择y方向上最远离障碍物的轨迹
        if (feasible_trajectories.empty()) {
            // 找出所有障碍物的y坐标
            std::vector<double> obstacle_y_positions;
            for (const auto& obstacle : obstacles_) {
                obstacle_y_positions.push_back(obstacle.position.y);
            }

            // 如果没有障碍物，则选择代价最小的轨迹
            if (obstacle_y_positions.empty()) {
                auto min_cost_it = std::min_element(trajectories.begin(), trajectories.end(),
                    [](const common::Trajectory& a, const common::Trajectory& b) {
                        return a.cost < b.cost;
                    });
                return *min_cost_it;
            }

            // 计算每条轨迹与障碍物在y方向上的最小距离
            std::vector<std::pair<double, size_t>> trajectory_distance_indices;
            for (size_t i = 0; i < trajectories.size(); i++) {
                double min_distance = std::numeric_limits<double>::max();
                // 对每个轨迹点，计算与所有障碍物的y方向距离，取最小值
                for (const auto& point : trajectories[i].points) {
                    for (double obs_y : obstacle_y_positions) {
                        double distance = std::abs(point.y - obs_y);
                        min_distance = std::min(min_distance, distance);
                    }
                }
                trajectory_distance_indices.push_back(std::make_pair(min_distance, i));
            }

            // 选择y方向上距离障碍物最远的轨迹
            auto max_distance_it = std::max_element(trajectory_distance_indices.begin(), trajectory_distance_indices.end(),
                [](const std::pair<double, size_t>& a, const std::pair<double, size_t>& b) {
                    return a.first < b.first;
                });

            return trajectories[max_distance_it->second];
        }

        // 如果有无碰撞轨迹，则在无碰撞轨迹中选择代价最小的
        auto min_cost_it = std::min_element(feasible_trajectories.begin(), feasible_trajectories.end(),
            [](const common::Trajectory& a, const common::Trajectory& b) {
                return a.cost < b.cost;
            });

        return *min_cost_it;
    }

    bool TrajEvaluator::needLaneChange(const std::vector<common::Trajectory>& trajectories) {
        // 检查是否所有轨迹都有碰撞
        bool all_collisions = true;
        for (const auto& trajectory : trajectories) {
            if (!trajectory.has_collision) {
                all_collisions = false;
                break;
            }
        }

        return all_collisions;
    }

    int TrajEvaluator::determineTargetLane() {
        // 检查是否可以向左换道
        bool can_change_left = (vehicle_state_.current_lane > 0);

        // 检查是否可以向右换道
        bool can_change_right = (vehicle_state_.current_lane < lane_params_.num_lanes - 1);

        // 确定换道方向
        if (can_change_left && can_change_right) {
            // 如果两侧都可以换道，选择障碍物较少的一侧
            // 这里简化为随机选择，实际应该基于障碍物分布决定
            return (rand() % 2 == 0) ? vehicle_state_.current_lane - 1 : vehicle_state_.current_lane + 1;
        }
        else if (can_change_left) {
            return vehicle_state_.current_lane - 1;
        }
        else if (can_change_right) {
            return vehicle_state_.current_lane + 1;
        }
        else {
            // 无法换道，返回当前车道
            return vehicle_state_.current_lane;
        }
    }

    double TrajEvaluator::calculateTrajectoryCost(const common::Trajectory& trajectory) {
        if (trajectory.has_collision) {
            return std::numeric_limits<double>::max();
        }

        double cost = 0.0;

        // 计算距离中心线的偏离代价
        double lane_center_y = vehicle_state_.current_lane == 0 ?
            lane_params_.width / 2 :     // 下车道中心 
            lane_params_.width * 3 / 2;  // 上车道中心
        double deviation_from_center = std::abs(trajectory.points.back().y - lane_center_y);
        cost += 0.5 * deviation_from_center; // 降低偏离中心线的惩罚权重

        // 计算曲率代价（惩罚过大的曲率变化）
        double curvature_cost = 0.0;
        for (const auto& point : trajectory.points) {
            curvature_cost += std::abs(point.curvature);
        }
        cost += 2.0 * curvature_cost; // 降低曲率惩罚权重，允许更弯曲的轨迹

        // 计算与障碍物的接近度代价
        double obstacle_cost = 0.0;
        for (const auto& point : trajectory.points) {
            for (const auto& obstacle : obstacles_) {
                double dx = point.x - obstacle.position.x;
                double dy = point.y - obstacle.position.y;
                double distance = std::sqrt(dx * dx + dy * dy);

                // 距离太近时增加代价
                double min_safe_distance = 2.0; // 安全距离
                if (distance < min_safe_distance) {
                    obstacle_cost += (min_safe_distance - distance) * 10.0;
                }
            }
        }
        cost += obstacle_cost;

        return cost;
    }

} // namespace planning