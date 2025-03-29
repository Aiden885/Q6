#ifndef TRAJ_EVALUATOR_H
#define TRAJ_EVALUATOR_H

#include <vector>
#include "Types.h"

namespace planning {

    class TrajEvaluator {
    public:
        TrajEvaluator(const common::VehicleParams& vehicle_params, const common::LaneParams& lane_params);

        // 设置车辆状态
        void setVehicleState(const common::VehicleState& state);

        // 设置障碍物
        void setObstacles(const std::vector<common::Obstacle>& obstacles);

        // 计算轨迹代价
        void calculateCosts(std::vector<common::Trajectory>& trajectories);

        // 选择最优轨迹
        common::Trajectory selectOptimalTrajectory(const std::vector<common::Trajectory>& trajectories);

        // 判断是否需要换道
        bool needLaneChange(const std::vector<common::Trajectory>& trajectories);

        // 确定目标换道车道
        int determineTargetLane();

    private:
        // 计算单条轨迹代价
        double calculateTrajectoryCost(const common::Trajectory& trajectory);

        common::VehicleParams vehicle_params_;
        common::LaneParams lane_params_;
        common::VehicleState vehicle_state_;
        std::vector<common::Obstacle> obstacles_;
    };

} // namespace planning

#endif // TRAJ_EVALUATOR_H