#ifndef OBSTACLE_AVOIDANCE_CONTROLLER_H
#define OBSTACLE_AVOIDANCE_CONTROLLER_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "Types.h"
#include "TrajGenerator.h"
#include "CollisionDetector.h"
#include "TrajEvaluator.h"
#include "Visualizer.h"

namespace planning {

    class ObstacleAvoidanceController {
    public:
        ObstacleAvoidanceController(const common::VehicleParams& vehicle_params, const common::LaneParams& lane_params);

        // ���ó�����ǰ״̬
        void setVehicleState(const common::VehicleState& state);

        // �����ϰ���
        void setObstacles(const std::vector<common::Obstacle>& obstacles);

        // ִ�й켣�滮
        common::Trajectory planTrajectory(double planning_distance, int num_trajectories, common::SceneType scene_type);

        // ���³���״̬�����Ź켣�˶���
        void updateVehicleState(double dt, const common::Trajectory& trajectory);

        // ���ӻ��켣�滮���
        cv::Mat visualize(const std::vector<common::Trajectory>& trajectories, const common::Trajectory& selected_trajectory);

        // ��ȡ�������ɵĹ켣����
        const std::vector<common::Trajectory>& getLatestTrajectories() const { return trajectories_; }

        // ��ȡ��ǰ�Ƿ��ڻ���ģʽ
        bool isLaneChangeMode() const { return lane_change_needed_; }

        // ��ȡ��ǰ����״̬
        const common::VehicleState& getVehicleState() const { return vehicle_state_; }

        // ���ݳ������ͳ�ʼ���ϰ���
        std::vector<common::Obstacle> initializeObstaclesForScene(common::SceneType scene_type);

    private:
        common::VehicleParams vehicle_params_;
        common::LaneParams lane_params_;
        common::VehicleState vehicle_state_;
        std::vector<common::Obstacle> obstacles_;

        TrajGenerator trajectory_generator_;
        CollisionDetector collision_detector_;
        TrajEvaluator trajectory_evaluator_;
        visualization::Visualizer visualizer_;

        bool lane_change_needed_;
        std::vector<common::Trajectory> trajectories_; // �洢�������ɵĹ켣����

        // ��ǰ�켣��������
        int current_trajectory_index_;
    };

} // namespace planning

#endif // OBSTACLE_AVOIDANCE_CONTROLLER_H