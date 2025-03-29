#include "TrajGenerator.h"
#include "BezierCurve.h"

namespace planning {

    TrajGenerator::TrajGenerator(const common::VehicleParams& vehicle_params, const common::LaneParams& lane_params)
        : vehicle_params_(vehicle_params), lane_params_(lane_params) {
        // 初始化车辆状态
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

        // 确保轨迹数量为奇数，以便有一条中心轨迹
        if (num_trajectories % 2 == 0) {
            num_trajectories++;
        }

        // 计算中心轨迹的终点位置（当前车道中心线上）
        double lane_center_y = vehicle_state_.current_lane == 0 ?
            lane_params_.width * 0.5 :     // 下车道中心 
            lane_params_.width * 3 / 2;  // 上车道中心

        // 计算轨迹间的横向偏移量 - 增加轨迹分布范围
        double lateral_range = lane_params_.width * 1; // 使用车道宽度的100%作为分布范围
        double lateral_step = lateral_range / num_trajectories;

        // 生成所有轨迹 - 修改为上下分布
        for (int i = 0; i < num_trajectories; i++) {
            common::Trajectory trajectory;
            trajectory.has_collision = false;
            trajectory.cost = 0.0;

            // 计算当前轨迹终点的横向偏移 - 修改为上下分布
            // 将轨迹分布在车道上下两侧，而不仅仅是均匀分布
            double lateral_offset;
            if (i < num_trajectories / 2) {
                // 下半部分轨迹
                lateral_offset = -lateral_range / 2 + i * lateral_step;
            }
            else {
                // 上半部分轨迹
                lateral_offset = i * lateral_step - lateral_range / 2;
            }
            double end_y = lane_center_y + lateral_offset;

            // 创建贝塞尔曲线控制点 - 增加曲率
            std::vector<cv::Point2d> control_points = {
                cv::Point2d(vehicle_state_.x, vehicle_state_.y),  // 起点
                cv::Point2d(vehicle_state_.x + planning_distance * 0.25, vehicle_state_.y),  // 控制点1-更近
                cv::Point2d(vehicle_state_.x + planning_distance * 0.6, end_y),  // 控制点2-更远离起点
                cv::Point2d(vehicle_state_.x + planning_distance, end_y)  // 终点
            };

            // 计算轨迹点
            const int num_points = 50;  // 轨迹点数量
            trajectory.points = math::BezierCurve::calculateCubicBezier(control_points, num_points);

            trajectories.push_back(trajectory);
        }

        return trajectories;
    }

    std::vector<common::Trajectory> TrajGenerator::generateLaneChangeTrajectories(double planning_distance, int num_trajectories, int target_lane) {
        std::vector<common::Trajectory> trajectories;

        // 确保轨迹数量为奇数
        if (num_trajectories % 2 == 0) {
            num_trajectories++;
        }

        // 计算目标车道中心
        double target_lane_center_y = target_lane == 0 ?
            lane_params_.width / 2 :     // 下车道中心 
            lane_params_.width * 3 / 2;  // 上车道中心

        // 计算轨迹间的横向偏移量 - 增加轨迹分布范围
        double lateral_range = lane_params_.width * 1.2; // 使用车道宽度的80%作为分布范围


        // 计算轨迹间的横向偏移量
        double lateral_step = lane_params_.width / (num_trajectories + 1);

        // 生成所有轨迹 - 修改为上下分布
        for (int i = 0; i < num_trajectories; i++) {
            common::Trajectory trajectory;
            trajectory.has_collision = false;
            trajectory.cost = 0.0;

            // 计算当前轨迹终点的横向偏移 - 修改为上下分布
            // 将轨迹分布在目标车道上下两侧
            double lateral_offset;
            if (i < num_trajectories / 2) {
                // 下半部分轨迹
                lateral_offset = -lateral_range / 2 + i * lateral_step;
            }
            else {
                // 上半部分轨迹
                lateral_offset = i * lateral_step - lateral_range / 2;
            }
            double end_y = target_lane_center_y + lateral_offset;

            // 创建贝塞尔曲线控制点 - 增加曲率，使曲线更弯曲
            std::vector<cv::Point2d> control_points = {
                cv::Point2d(vehicle_state_.x, vehicle_state_.y),  // 起点
                cv::Point2d(vehicle_state_.x + planning_distance * 0.3, vehicle_state_.y),  // 控制点1-更近
                cv::Point2d(vehicle_state_.x + planning_distance * 0.4, end_y),  // 控制点2-更远离起点
                cv::Point2d(vehicle_state_.x + planning_distance, end_y)  // 终点
            };

            // 计算轨迹点
            const int num_points = 50;  // 轨迹点数量
            trajectory.points = math::BezierCurve::calculateCubicBezier(control_points, num_points);

            trajectories.push_back(trajectory);
        }

        return trajectories;
    }

} // namespace planning