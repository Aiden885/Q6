#include "ObstacleAvoidanceController.h"
#include <cmath> // 添加这行，提供M_PI常量

// 如果编译器不提供M_PI常量（比如MSVC）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace planning {

    ObstacleAvoidanceController::ObstacleAvoidanceController(
        const common::VehicleParams& vehicle_params,
        const common::LaneParams& lane_params)
        : vehicle_params_(vehicle_params),
        lane_params_(lane_params),
        trajectory_generator_(vehicle_params, lane_params),
        collision_detector_(vehicle_params),
        trajectory_evaluator_(vehicle_params, lane_params),
        visualizer_(vehicle_params, lane_params),
        lane_change_needed_(false),
        current_trajectory_index_(0) {
    }

    void ObstacleAvoidanceController::setVehicleState(const common::VehicleState& state) {
        vehicle_state_ = state;
        trajectory_generator_.setVehicleState(state);
        trajectory_evaluator_.setVehicleState(state);
    }

    void ObstacleAvoidanceController::setObstacles(const std::vector<common::Obstacle>& obstacles) {
        obstacles_ = obstacles;
        collision_detector_.setObstacles(obstacles);
        trajectory_evaluator_.setObstacles(obstacles);
    }

    std::vector<common::Obstacle> ObstacleAvoidanceController::initializeObstaclesForScene(common::SceneType scene_type) {
        std::vector<common::Obstacle> obstacles;

        if (scene_type == common::IN_LANE_AVOIDANCE) {
            // 场景1：车道内避障 - 放置一个小障碍物
            common::Obstacle obstacle;
            obstacle.position = cv::Point2d(30.0, 3.5); // 前方30米，当前车道
            obstacle.width = 2.0;  // 小障碍物，可在车道内绕行
            obstacle.length = 2.0;
            obstacle.heading = 0.0;
            obstacle.is_static = true;
            obstacles.push_back(obstacle);
        }
        // 初始化场景2：换道避障 - 放置一个大障碍物
        else if (scene_type == common::LANE_CHANGE) {
            // 场景2：换道避障 - 放置一个大障碍物
            common::Obstacle obstacle;
            obstacle.position = cv::Point2d(30.0, 3.5); // 前方30米，当前车道
            obstacle.width = 6.0;  // 大障碍物，占据整个车道
            obstacle.length = 10.0;
            obstacle.heading = 0.0;
            obstacle.is_static = true;
            obstacles.push_back(obstacle);

            //// 添加第二个障碍物，用于测试极端情况
            //common::Obstacle obstacle2;
            //obstacle2.position = cv::Point2d(30.0, 10.5); // 前方30米，上车道
            //obstacle2.width = 6.0;  // 大障碍物
            //obstacle2.length = 5.0;
            //obstacle2.heading = 0.0;
            //obstacle2.is_static = true;
            //obstacles.push_back(obstacle2);
        }

        return obstacles;
    }

    common::Trajectory ObstacleAvoidanceController::planTrajectory(
        double planning_distance,
        int num_trajectories,
        common::SceneType scene_type) {

        // 根据场景类型决定是否强制换道
        bool force_lane_change = (scene_type == common::LANE_CHANGE);

        // 首先生成车道内避障轨迹
        if (!force_lane_change) {
            trajectories_ = trajectory_generator_.generateInLaneTrajectories(planning_distance, num_trajectories);
        }
        else {
            // 如果强制换道，直接生成换道轨迹
            int target_lane = (vehicle_state_.current_lane == 0) ? 1 : 0; // 切换到另一车道
            trajectories_ = trajectory_generator_.generateLaneChangeTrajectories(planning_distance, num_trajectories, target_lane);
            lane_change_needed_ = true;
        }

        // 检查碰撞
        collision_detector_.checkCollisions(trajectories_);

        // 计算轨迹代价
        trajectory_evaluator_.calculateCosts(trajectories_);

        // 如果不是强制换道，判断是否需要换道
        if (!force_lane_change) {
            lane_change_needed_ = trajectory_evaluator_.needLaneChange(trajectories_);

            // 如果需要换道，重新生成轨迹
            if (lane_change_needed_) {
                // 确定目标车道
                int target_lane = trajectory_evaluator_.determineTargetLane();

                // 生成换道轨迹
                trajectories_ = trajectory_generator_.generateLaneChangeTrajectories(planning_distance, num_trajectories, target_lane);

                // 重新检查碰撞
                collision_detector_.checkCollisions(trajectories_);

                // 重新计算轨迹代价
                trajectory_evaluator_.calculateCosts(trajectories_);
            }
        }

        // 选择最优轨迹
        common::Trajectory selected_trajectory = trajectory_evaluator_.selectOptimalTrajectory(trajectories_);
        current_trajectory_index_ = 0; // 重置轨迹跟踪索引

        return selected_trajectory;
    }

    void ObstacleAvoidanceController::updateVehicleState(double dt, const common::Trajectory& trajectory) {
        // 确保轨迹有效且索引在范围内
        if (trajectory.points.empty() || current_trajectory_index_ >= trajectory.points.size()) {
            return;
        }

        // 获取当前目标点
        const math::BezierPoint& target_point = trajectory.points[current_trajectory_index_];

        // 计算当前位置到目标点的距离
        double dx = target_point.x - vehicle_state_.x;
        double dy = target_point.y - vehicle_state_.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // 如果距离小于阈值，移动到下一个点
        double distance_threshold = vehicle_state_.velocity * dt * 0.5; // 使用车辆当前速度而不是车辆参数
        if (distance < distance_threshold && current_trajectory_index_ < trajectory.points.size() - 1) {
            current_trajectory_index_++;
            return;
        }

        // 更新车辆位置和朝向
        double target_heading = target_point.heading;

        // 平滑转向（限制转向速率）
        double max_heading_change = vehicle_params_.max_steering_angle * dt;
        double heading_diff = target_heading - vehicle_state_.heading;

        // 规范化角度差到[-pi, pi]
        while (heading_diff > M_PI) heading_diff -= 2 * M_PI;
        while (heading_diff < -M_PI) heading_diff += 2 * M_PI;

        // 限制转向速率
        if (heading_diff > max_heading_change) {
            heading_diff = max_heading_change;
        }
        else if (heading_diff < -max_heading_change) {
            heading_diff = -max_heading_change;
        }

        vehicle_state_.heading += heading_diff;

        // 更新位置（沿着当前朝向移动）
        double step_distance = vehicle_state_.velocity * dt;
        vehicle_state_.x += step_distance * std::cos(vehicle_state_.heading);
        vehicle_state_.y += step_distance * std::sin(vehicle_state_.heading);

        // 更新车道信息
        if (vehicle_state_.y < lane_params_.width * 0.5) {
            vehicle_state_.current_lane = 0; // 下车道
        }
        else {
            vehicle_state_.current_lane = 1; // 上车道
        }

        // 更新车速（简化处理，保持恒定速度）
        // 真实场景中应该根据轨迹曲率和安全距离调整速度

        // 将更新后的车辆状态同步到其他模块
        trajectory_generator_.setVehicleState(vehicle_state_);
        trajectory_evaluator_.setVehicleState(vehicle_state_);
    }

    cv::Mat ObstacleAvoidanceController::visualize(
        const std::vector<common::Trajectory>& trajectories,
        const common::Trajectory& selected_trajectory) {
        return visualizer_.visualize(vehicle_state_, obstacles_, trajectories, selected_trajectory, lane_change_needed_);
    }

} // namespace planning