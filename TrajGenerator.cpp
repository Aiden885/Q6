#include "TrajGenerator.h"
#include "BezierCurve.h"

namespace planning {

    TrajGenerator::TrajGenerator(const common::VehicleParams& vehicle_params, const common::LaneParams& lane_params)
        : vehicle_params_(vehicle_params), lane_params_(lane_params) {
        // ��ʼ������״̬
        vehicle_state_.x = 0.0;
        vehicle_state_.y = 0.0;
        vehicle_state_.heading = 0.0;
        vehicle_state_.velocity = 0.0;
        vehicle_state_.current_lane = 0;
    }

    void TrajGenerator::setVehicleState(const common::VehicleState& state) {
        vehicle_state_ = state;
    }

    std::vector<common::Trajectory> TrajGenerator::generateInLaneTrajectories(double planning_distance, int num_trajectories) {
        std::vector<common::Trajectory> trajectories;

        // ȷ���켣����Ϊ�������Ա���һ�����Ĺ켣
        if (num_trajectories % 2 == 0) {
            num_trajectories++;
        }

        // �������Ĺ켣���յ�λ�ã���ǰ�����������ϣ�
        double lane_center_y = vehicle_state_.current_lane == 0 ?
            lane_params_.width * 0.5 :     // �³������� 
            lane_params_.width * 3 / 2;  // �ϳ�������

        // ����켣��ĺ���ƫ���� - ���ӹ켣�ֲ���Χ
        double lateral_range = lane_params_.width * 1; // ʹ�ó�����ȵ�100%��Ϊ�ֲ���Χ
        double lateral_step = lateral_range / num_trajectories;

        // �������й켣 - �޸�Ϊ���·ֲ�
        for (int i = 0; i < num_trajectories; i++) {
            common::Trajectory trajectory;
            trajectory.has_collision = false;
            trajectory.cost = 0.0;

            // ���㵱ǰ�켣�յ�ĺ���ƫ�� - �޸�Ϊ���·ֲ�
            // ���켣�ֲ��ڳ����������࣬���������Ǿ��ȷֲ�
            double lateral_offset;
            if (i < num_trajectories / 2) {
                // �°벿�ֹ켣
                lateral_offset = -lateral_range / 2 + i * lateral_step;
            }
            else {
                // �ϰ벿�ֹ켣
                lateral_offset = i * lateral_step - lateral_range / 2;
            }
            double end_y = lane_center_y + lateral_offset;

            // �������������߿��Ƶ� - ��������
            std::vector<cv::Point2d> control_points = {
                cv::Point2d(vehicle_state_.x, vehicle_state_.y),  // ���
                cv::Point2d(vehicle_state_.x + planning_distance * 0.25, vehicle_state_.y),  // ���Ƶ�1-����
                cv::Point2d(vehicle_state_.x + planning_distance * 0.6, end_y),  // ���Ƶ�2-��Զ�����
                cv::Point2d(vehicle_state_.x + planning_distance, end_y)  // �յ�
            };

            // ����켣��
            const int num_points = 50;  // �켣������
            trajectory.points = math::BezierCurve::calculateCubicBezier(control_points, num_points);

            trajectories.push_back(trajectory);
        }

        return trajectories;
    }

    std::vector<common::Trajectory> TrajGenerator::generateLaneChangeTrajectories(double planning_distance, int num_trajectories, int target_lane) {
        std::vector<common::Trajectory> trajectories;

        // ȷ���켣����Ϊ����
        if (num_trajectories % 2 == 0) {
            num_trajectories++;
        }

        // ����Ŀ�공������
        double target_lane_center_y = target_lane == 0 ?
            lane_params_.width / 2 :     // �³������� 
            lane_params_.width * 3 / 2;  // �ϳ�������

        // ����켣��ĺ���ƫ���� - ���ӹ켣�ֲ���Χ
        double lateral_range = lane_params_.width * 1.2; // ʹ�ó�����ȵ�80%��Ϊ�ֲ���Χ


        // ����켣��ĺ���ƫ����
        double lateral_step = lane_params_.width / (num_trajectories + 1);

        // �������й켣 - �޸�Ϊ���·ֲ�
        for (int i = 0; i < num_trajectories; i++) {
            common::Trajectory trajectory;
            trajectory.has_collision = false;
            trajectory.cost = 0.0;

            // ���㵱ǰ�켣�յ�ĺ���ƫ�� - �޸�Ϊ���·ֲ�
            // ���켣�ֲ���Ŀ�공����������
            double lateral_offset;
            if (i < num_trajectories / 2) {
                // �°벿�ֹ켣
                lateral_offset = -lateral_range / 2 + i * lateral_step;
            }
            else {
                // �ϰ벿�ֹ켣
                lateral_offset = i * lateral_step - lateral_range / 2;
            }
            double end_y = target_lane_center_y + lateral_offset;

            // �������������߿��Ƶ� - �������ʣ�ʹ���߸�����
            std::vector<cv::Point2d> control_points = {
                cv::Point2d(vehicle_state_.x, vehicle_state_.y),  // ���
                cv::Point2d(vehicle_state_.x + planning_distance * 0.3, vehicle_state_.y),  // ���Ƶ�1-����
                cv::Point2d(vehicle_state_.x + planning_distance * 0.4, end_y),  // ���Ƶ�2-��Զ�����
                cv::Point2d(vehicle_state_.x + planning_distance, end_y)  // �յ�
            };

            // ����켣��
            const int num_points = 50;  // �켣������
            trajectory.points = math::BezierCurve::calculateCubicBezier(control_points, num_points);

            trajectories.push_back(trajectory);
        }

        return trajectories;
    }

} // namespace planning