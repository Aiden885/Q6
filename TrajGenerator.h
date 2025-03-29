#ifndef TRAJ_GENERATOR_H
#define TRAJ_GENERATOR_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "Types.h"

namespace planning {

    class TrajGenerator {
    public:
        TrajGenerator(const common::VehicleParams& vehicle_params, const common::LaneParams& lane_params);

        // 设置车辆当前状态
        void setVehicleState(const common::VehicleState& state);

        // 生成车道内避障轨迹
        std::vector<common::Trajectory> generateInLaneTrajectories(double planning_distance, int num_trajectories);

        // 生成换道轨迹
        std::vector<common::Trajectory> generateLaneChangeTrajectories(double planning_distance, int num_trajectories, int target_lane);

    private:
        common::VehicleParams vehicle_params_;
        common::LaneParams lane_params_;
        common::VehicleState vehicle_state_;
    };

} // namespace planning

#endif // TRAJ_GENERATOR_H