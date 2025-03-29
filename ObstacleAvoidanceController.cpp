#include "ObstacleAvoidanceController.h"
#include <cmath> // ������У��ṩM_PI����

// ������������ṩM_PI����������MSVC��
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
            // ����1�������ڱ��� - ����һ��С�ϰ���
            common::Obstacle obstacle;
            obstacle.position = cv::Point2d(30.0, 3.5); // ǰ��30�ף���ǰ����
            obstacle.width = 2.0;  // С�ϰ�����ڳ���������
            obstacle.length = 2.0;
            obstacle.heading = 0.0;
            obstacle.is_static = true;
            obstacles.push_back(obstacle);
        }
        // ��ʼ������2���������� - ����һ�����ϰ���
        else if (scene_type == common::LANE_CHANGE) {
            // ����2���������� - ����һ�����ϰ���
            common::Obstacle obstacle;
            obstacle.position = cv::Point2d(30.0, 3.5); // ǰ��30�ף���ǰ����
            obstacle.width = 6.0;  // ���ϰ��ռ����������
            obstacle.length = 10.0;
            obstacle.heading = 0.0;
            obstacle.is_static = true;
            obstacles.push_back(obstacle);

            //// ��ӵڶ����ϰ�����ڲ��Լ������
            //common::Obstacle obstacle2;
            //obstacle2.position = cv::Point2d(30.0, 10.5); // ǰ��30�ף��ϳ���
            //obstacle2.width = 6.0;  // ���ϰ���
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

        // ���ݳ������;����Ƿ�ǿ�ƻ���
        bool force_lane_change = (scene_type == common::LANE_CHANGE);

        // �������ɳ����ڱ��Ϲ켣
        if (!force_lane_change) {
            trajectories_ = trajectory_generator_.generateInLaneTrajectories(planning_distance, num_trajectories);
        }
        else {
            // ���ǿ�ƻ�����ֱ�����ɻ����켣
            int target_lane = (vehicle_state_.current_lane == 0) ? 1 : 0; // �л�����һ����
            trajectories_ = trajectory_generator_.generateLaneChangeTrajectories(planning_distance, num_trajectories, target_lane);
            lane_change_needed_ = true;
        }

        // �����ײ
        collision_detector_.checkCollisions(trajectories_);

        // ����켣����
        trajectory_evaluator_.calculateCosts(trajectories_);

        // �������ǿ�ƻ������ж��Ƿ���Ҫ����
        if (!force_lane_change) {
            lane_change_needed_ = trajectory_evaluator_.needLaneChange(trajectories_);

            // �����Ҫ�������������ɹ켣
            if (lane_change_needed_) {
                // ȷ��Ŀ�공��
                int target_lane = trajectory_evaluator_.determineTargetLane();

                // ���ɻ����켣
                trajectories_ = trajectory_generator_.generateLaneChangeTrajectories(planning_distance, num_trajectories, target_lane);

                // ���¼����ײ
                collision_detector_.checkCollisions(trajectories_);

                // ���¼���켣����
                trajectory_evaluator_.calculateCosts(trajectories_);
            }
        }

        // ѡ�����Ź켣
        common::Trajectory selected_trajectory = trajectory_evaluator_.selectOptimalTrajectory(trajectories_);
        current_trajectory_index_ = 0; // ���ù켣��������

        return selected_trajectory;
    }

    void ObstacleAvoidanceController::updateVehicleState(double dt, const common::Trajectory& trajectory) {
        // ȷ���켣��Ч�������ڷ�Χ��
        if (trajectory.points.empty() || current_trajectory_index_ >= trajectory.points.size()) {
            return;
        }

        // ��ȡ��ǰĿ���
        const math::BezierPoint& target_point = trajectory.points[current_trajectory_index_];

        // ���㵱ǰλ�õ�Ŀ���ľ���
        double dx = target_point.x - vehicle_state_.x;
        double dy = target_point.y - vehicle_state_.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // �������С����ֵ���ƶ�����һ����
        double distance_threshold = vehicle_state_.velocity * dt * 0.5; // ʹ�ó�����ǰ�ٶȶ����ǳ�������
        if (distance < distance_threshold && current_trajectory_index_ < trajectory.points.size() - 1) {
            current_trajectory_index_++;
            return;
        }

        // ���³���λ�úͳ���
        double target_heading = target_point.heading;

        // ƽ��ת������ת�����ʣ�
        double max_heading_change = vehicle_params_.max_steering_angle * dt;
        double heading_diff = target_heading - vehicle_state_.heading;

        // �淶���ǶȲ[-pi, pi]
        while (heading_diff > M_PI) heading_diff -= 2 * M_PI;
        while (heading_diff < -M_PI) heading_diff += 2 * M_PI;

        // ����ת������
        if (heading_diff > max_heading_change) {
            heading_diff = max_heading_change;
        }
        else if (heading_diff < -max_heading_change) {
            heading_diff = -max_heading_change;
        }

        vehicle_state_.heading += heading_diff;

        // ����λ�ã����ŵ�ǰ�����ƶ���
        double step_distance = vehicle_state_.velocity * dt;
        vehicle_state_.x += step_distance * std::cos(vehicle_state_.heading);
        vehicle_state_.y += step_distance * std::sin(vehicle_state_.heading);

        // ���³�����Ϣ
        if (vehicle_state_.y < lane_params_.width * 0.5) {
            vehicle_state_.current_lane = 0; // �³���
        }
        else {
            vehicle_state_.current_lane = 1; // �ϳ���
        }

        // ���³��٣��򻯴������ֺ㶨�ٶȣ�
        // ��ʵ������Ӧ�ø��ݹ켣���ʺͰ�ȫ��������ٶ�

        // �����º�ĳ���״̬ͬ��������ģ��
        trajectory_generator_.setVehicleState(vehicle_state_);
        trajectory_evaluator_.setVehicleState(vehicle_state_);
    }

    cv::Mat ObstacleAvoidanceController::visualize(
        const std::vector<common::Trajectory>& trajectories,
        const common::Trajectory& selected_trajectory) {
        return visualizer_.visualize(vehicle_state_, obstacles_, trajectories, selected_trajectory, lane_change_needed_);
    }

} // namespace planning