#include "CollisionDetector.h"
#include <cmath>
#include <limits>

namespace planning {

    CollisionDetector::CollisionDetector(const common::VehicleParams& vehicle_params)
        : vehicle_params_(vehicle_params) {
    }

    void CollisionDetector::setObstacles(const std::vector<common::Obstacle>& obstacles) {
        obstacles_ = obstacles;
    }

    void CollisionDetector::checkCollisions(std::vector<common::Trajectory>& trajectories) {
        for (auto& trajectory : trajectories) {
            trajectory.has_collision = false;

            // 检查轨迹的每个点是否与任一障碍物碰撞
            for (const auto& point : trajectory.points) {
                for (const auto& obstacle : obstacles_) {
                    if (checkPointCollision(point, obstacle)) {
                        trajectory.has_collision = true;
                        break;
                    }
                }
                if (trajectory.has_collision) {
                    break;
                }
            }
        }
    }

    bool CollisionDetector::checkPointCollision(const math::BezierPoint& point, const common::Obstacle& obstacle) {
        // 计算车辆包围盒的四个角点
        double half_length = vehicle_params_.length / 2.0;
        double half_width = vehicle_params_.width / 2.0;

        // 车辆当前位置角点坐标
        double cos_heading = std::cos(point.heading);
        double sin_heading = std::sin(point.heading);

        std::vector<cv::Point2d> vehicle_corners = {
            cv::Point2d(point.x + cos_heading * half_length - sin_heading * half_width,
                       point.y + sin_heading * half_length + cos_heading * half_width),
            cv::Point2d(point.x + cos_heading * half_length + sin_heading * half_width,
                       point.y + sin_heading * half_length - cos_heading * half_width),
            cv::Point2d(point.x - cos_heading * half_length + sin_heading * half_width,
                       point.y - sin_heading * half_length - cos_heading * half_width),
            cv::Point2d(point.x - cos_heading * half_length - sin_heading * half_width,
                       point.y - sin_heading * half_length + cos_heading * half_width)
        };

        // 障碍物角点坐标
        double obs_cos_heading = std::cos(obstacle.heading);
        double obs_sin_heading = std::sin(obstacle.heading);
        double obs_half_length = obstacle.length / 2.0;
        double obs_half_width = obstacle.width / 2.0;

        std::vector<cv::Point2d> obstacle_corners = {
            cv::Point2d(obstacle.position.x + obs_cos_heading * obs_half_length - obs_sin_heading * obs_half_width,
                       obstacle.position.y + obs_sin_heading * obs_half_length + obs_cos_heading * obs_half_width),
            cv::Point2d(obstacle.position.x + obs_cos_heading * obs_half_length + obs_sin_heading * obs_half_width,
                       obstacle.position.y + obs_sin_heading * obs_half_length - obs_cos_heading * obs_half_width),
            cv::Point2d(obstacle.position.x - obs_cos_heading * obs_half_length + obs_sin_heading * obs_half_width,
                       obstacle.position.y - obs_sin_heading * obs_half_length - obs_cos_heading * obs_half_width),
            cv::Point2d(obstacle.position.x - obs_cos_heading * obs_half_length - obs_sin_heading * obs_half_width,
                       obstacle.position.y - obs_sin_heading * obs_half_length + obs_cos_heading * obs_half_width)
        };

        // 使用分离轴定理(SAT)进行碰撞检测
        return checkSATCollision(vehicle_corners, obstacle_corners);
    }

    bool CollisionDetector::checkSATCollision(const std::vector<cv::Point2d>& polygon1,
        const std::vector<cv::Point2d>& polygon2) {
        // 分别检查两个多边形的所有边的法向量作为投影轴
        if (!checkSeparationOnAxes(polygon1, polygon2)) {
            return false;
        }

        if (!checkSeparationOnAxes(polygon2, polygon1)) {
            return false;
        }

        // 没有找到分离轴，两个多边形相交
        return true;
    }

    bool CollisionDetector::checkSeparationOnAxes(const std::vector<cv::Point2d>& polygon1,
        const std::vector<cv::Point2d>& polygon2) {
        int size = polygon1.size();
        for (int i = 0; i < size; i++) {
            // 计算边的法向量作为投影轴
            int next_i = (i + 1) % size;
            double edge_x = polygon1[next_i].x - polygon1[i].x;
            double edge_y = polygon1[next_i].y - polygon1[i].y;

            // 法向量（垂直于边）
            double axis_x = -edge_y;
            double axis_y = edge_x;

            // 归一化法向量
            double length = std::sqrt(axis_x * axis_x + axis_y * axis_y);
            if (length > 0) {
                axis_x /= length;
                axis_y /= length;
            }

            // 计算两个多边形在该轴上的投影范围
            double min1, max1, min2, max2;
            projectPolygon(polygon1, axis_x, axis_y, min1, max1);
            projectPolygon(polygon2, axis_x, axis_y, min2, max2);

            // 检查投影是否重叠，如果不重叠则找到了分离轴
            if (max1 < min2 || max2 < min1) {
                return false;
            }
        }

        // 在所有轴上都有重叠
        return true;
    }

    void CollisionDetector::projectPolygon(const std::vector<cv::Point2d>& polygon,
        double axis_x, double axis_y,
        double& min_proj, double& max_proj) {
        min_proj = std::numeric_limits<double>::max();
        max_proj = std::numeric_limits<double>::lowest();

        for (const auto& point : polygon) {
            // 点在轴上的投影
            double projection = point.x * axis_x + point.y * axis_y;

            if (projection < min_proj) min_proj = projection;
            if (projection > max_proj) max_proj = projection;
        }
    }

} // namespace planning