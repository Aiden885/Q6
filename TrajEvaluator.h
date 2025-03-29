#ifndef TRAJ_EVALUATOR_H
#define TRAJ_EVALUATOR_H

#include <vector>
#include "Types.h"

namespace planning {

    class TrajEvaluator {
    public:
        TrajEvaluator(const common::VehicleParams& vehicle_params, const common::LaneParams& lane_params);

        // ���ó���״̬
        void setVehicleState(const common::VehicleState& state);

        // �����ϰ���
        void setObstacles(const std::vector<common::Obstacle>& obstacles);

        // ����켣����
        void calculateCosts(std::vector<common::Trajectory>& trajectories);

        // ѡ�����Ź켣
        common::Trajectory selectOptimalTrajectory(const std::vector<common::Trajectory>& trajectories);

        // �ж��Ƿ���Ҫ����
        bool needLaneChange(const std::vector<common::Trajectory>& trajectories);

        // ȷ��Ŀ�껻������
        int determineTargetLane();

    private:
        // ���㵥���켣����
        double calculateTrajectoryCost(const common::Trajectory& trajectory);

        common::VehicleParams vehicle_params_;
        common::LaneParams lane_params_;
        common::VehicleState vehicle_state_;
        std::vector<common::Obstacle> obstacles_;
    };

} // namespace planning

#endif // TRAJ_EVALUATOR_H