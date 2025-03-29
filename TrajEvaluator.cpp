#include "TrajEvaluator.h"
#include <cmath>
#include <limits>
#include <algorithm>

namespace planning {

    TrajEvaluator::TrajEvaluator(const common::VehicleParams& vehicle_params, const common::LaneParams& lane_params)
        : vehicle_params_(vehicle_params), lane_params_(lane_params) {
        // ��ʼ������״̬
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
        // ���û�п��й켣�����ص�һ���켣�����Ϊ��ײ
        if (trajectories.empty()) {
            common::Trajectory empty_trajectory;
            empty_trajectory.has_collision = true;
            return empty_trajectory;
        }

        // ����ɸѡ������ײ�Ĺ켣
        std::vector<common::Trajectory> feasible_trajectories;
        for (const auto& trajectory : trajectories) {
            if (!trajectory.has_collision) {
                feasible_trajectories.push_back(trajectory);
            }
        }

        // ���û������ײ�Ĺ켣��ѡ��y��������Զ���ϰ���Ĺ켣
        if (feasible_trajectories.empty()) {
            // �ҳ������ϰ����y����
            std::vector<double> obstacle_y_positions;
            for (const auto& obstacle : obstacles_) {
                obstacle_y_positions.push_back(obstacle.position.y);
            }

            // ���û���ϰ����ѡ�������С�Ĺ켣
            if (obstacle_y_positions.empty()) {
                auto min_cost_it = std::min_element(trajectories.begin(), trajectories.end(),
                    [](const common::Trajectory& a, const common::Trajectory& b) {
                        return a.cost < b.cost;
                    });
                return *min_cost_it;
            }

            // ����ÿ���켣���ϰ�����y�����ϵ���С����
            std::vector<std::pair<double, size_t>> trajectory_distance_indices;
            for (size_t i = 0; i < trajectories.size(); i++) {
                double min_distance = std::numeric_limits<double>::max();
                // ��ÿ���켣�㣬�����������ϰ����y������룬ȡ��Сֵ
                for (const auto& point : trajectories[i].points) {
                    for (double obs_y : obstacle_y_positions) {
                        double distance = std::abs(point.y - obs_y);
                        min_distance = std::min(min_distance, distance);
                    }
                }
                trajectory_distance_indices.push_back(std::make_pair(min_distance, i));
            }

            // ѡ��y�����Ͼ����ϰ�����Զ�Ĺ켣
            auto max_distance_it = std::max_element(trajectory_distance_indices.begin(), trajectory_distance_indices.end(),
                [](const std::pair<double, size_t>& a, const std::pair<double, size_t>& b) {
                    return a.first < b.first;
                });

            return trajectories[max_distance_it->second];
        }

        // ���������ײ�켣����������ײ�켣��ѡ�������С��
        auto min_cost_it = std::min_element(feasible_trajectories.begin(), feasible_trajectories.end(),
            [](const common::Trajectory& a, const common::Trajectory& b) {
                return a.cost < b.cost;
            });

        return *min_cost_it;
    }

    bool TrajEvaluator::needLaneChange(const std::vector<common::Trajectory>& trajectories) {
        // ����Ƿ����й켣������ײ
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
        // ����Ƿ�������󻻵�
        bool can_change_left = (vehicle_state_.current_lane > 0);

        // ����Ƿ�������һ���
        bool can_change_right = (vehicle_state_.current_lane < lane_params_.num_lanes - 1);

        // ȷ����������
        if (can_change_left && can_change_right) {
            // ������඼���Ի�����ѡ���ϰ�����ٵ�һ��
            // �����Ϊ���ѡ��ʵ��Ӧ�û����ϰ���ֲ�����
            return (rand() % 2 == 0) ? vehicle_state_.current_lane - 1 : vehicle_state_.current_lane + 1;
        }
        else if (can_change_left) {
            return vehicle_state_.current_lane - 1;
        }
        else if (can_change_right) {
            return vehicle_state_.current_lane + 1;
        }
        else {
            // �޷����������ص�ǰ����
            return vehicle_state_.current_lane;
        }
    }

    double TrajEvaluator::calculateTrajectoryCost(const common::Trajectory& trajectory) {
        if (trajectory.has_collision) {
            return std::numeric_limits<double>::max();
        }

        double cost = 0.0;

        // ������������ߵ�ƫ�����
        double lane_center_y = vehicle_state_.current_lane == 0 ?
            lane_params_.width / 2 :     // �³������� 
            lane_params_.width * 3 / 2;  // �ϳ�������
        double deviation_from_center = std::abs(trajectory.points.back().y - lane_center_y);
        cost += 0.5 * deviation_from_center; // ����ƫ�������ߵĳͷ�Ȩ��

        // �������ʴ��ۣ��ͷ���������ʱ仯��
        double curvature_cost = 0.0;
        for (const auto& point : trajectory.points) {
            curvature_cost += std::abs(point.curvature);
        }
        cost += 2.0 * curvature_cost; // �������ʳͷ�Ȩ�أ�����������Ĺ켣

        // �������ϰ���Ľӽ��ȴ���
        double obstacle_cost = 0.0;
        for (const auto& point : trajectory.points) {
            for (const auto& obstacle : obstacles_) {
                double dx = point.x - obstacle.position.x;
                double dy = point.y - obstacle.position.y;
                double distance = std::sqrt(dx * dx + dy * dy);

                // ����̫��ʱ���Ӵ���
                double min_safe_distance = 2.0; // ��ȫ����
                if (distance < min_safe_distance) {
                    obstacle_cost += (min_safe_distance - distance) * 10.0;
                }
            }
        }
        cost += obstacle_cost;

        return cost;
    }

} // namespace planning