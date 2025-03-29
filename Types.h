#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "BezierCurve.h"

namespace common {

    // 车辆参数结构体
    struct VehicleParams {
        double width;  // 车宽
        double length; // 车长
        double wheelbase; // 轴距
        double max_steering_angle; // 最大转向角
    };

    // 车道参数结构体
    struct LaneParams {
        double width;  // 车道宽度
        int num_lanes; // 车道数量
    };

    // 车辆状态结构体
    struct VehicleState {
        double x;
        double y;
        double heading; // 车辆朝向（弧度）
        double velocity; // 车速
        int current_lane; // 当前车道
    };

    // 障碍物结构体
    struct Obstacle {
        cv::Point2d position; // 障碍物中心位置
        double width;  // 障碍物宽度
        double length; // 障碍物长度
        double heading; // 障碍物朝向（弧度）
        bool is_static; // 是否为静态障碍物
        cv::Point2d velocity; // 障碍物速度向量（如果非静态）
    };

    // 轨迹结构体
    struct Trajectory {
        std::vector<math::BezierPoint> points;
        bool has_collision;
        double cost; // 轨迹评估的代价
    };

    // 场景类型
    enum SceneType {
        IN_LANE_AVOIDANCE = 1,  // 车道内避障
        LANE_CHANGE = 2         // 换道避障
    };

} // namespace common

#endif // TYPES_H