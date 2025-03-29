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

        // 设置车辆当前状态
        void setVehicleState(const common::VehicleState& state);

        // 设置障碍物
        void setObstacles(const std::vector<common::Obstacle>& obstacles);

        // 执行轨迹规划
        common::Trajectory planTrajectory(double planning_distance, int num_trajectories, common::SceneType scene_type);

        // 更新车辆状态（沿着轨迹运动）
        void updateVehicleState(double dt, const common::Trajectory& trajectory);

        // 可视化轨迹规划结果
        cv::Mat visualize(const std::vector<common::Trajectory>& trajectories, const common::Trajectory& selected_trajectory);

        // 获取最新生成的轨迹集合
        const std::vector<common::Trajectory>& getLatestTrajectories() const { return trajectories_; }

        // 获取当前是否处于换道模式
        bool isLaneChangeMode() const { return lane_change_needed_; }

        // 获取当前车辆状态
        const common::VehicleState& getVehicleState() const { return vehicle_state_; }

        // 根据场景类型初始化障碍物
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
        std::vector<common::Trajectory> trajectories_; // 存储最新生成的轨迹集合

        // 当前轨迹跟踪索引
        int current_trajectory_index_;
    };

} // namespace planning

#endif // OBSTACLE_AVOIDANCE_CONTROLLER_H