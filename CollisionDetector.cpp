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

            // ���켣��ÿ�����Ƿ�����һ�ϰ�����ײ
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
        // ���㳵����Χ�е��ĸ��ǵ�
        double half_length = vehicle_params_.length / 2.0;
        double half_width = vehicle_params_.width / 2.0;

        // ������ǰλ�ýǵ�����
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

        // �ϰ���ǵ�����
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

        // ʹ�÷����ᶨ��(SAT)������ײ���
        return checkSATCollision(vehicle_corners, obstacle_corners);
    }

    bool CollisionDetector::checkSATCollision(const std::vector<cv::Point2d>& polygon1,
        const std::vector<cv::Point2d>& polygon2) {
        // �ֱ�����������ε����бߵķ�������ΪͶӰ��
        if (!checkSeparationOnAxes(polygon1, polygon2)) {
            return false;
        }

        if (!checkSeparationOnAxes(polygon2, polygon1)) {
            return false;
        }

        // û���ҵ������ᣬ����������ཻ
        return true;
    }

    bool CollisionDetector::checkSeparationOnAxes(const std::vector<cv::Point2d>& polygon1,
        const std::vector<cv::Point2d>& polygon2) {
        int size = polygon1.size();
        for (int i = 0; i < size; i++) {
            // ����ߵķ�������ΪͶӰ��
            int next_i = (i + 1) % size;
            double edge_x = polygon1[next_i].x - polygon1[i].x;
            double edge_y = polygon1[next_i].y - polygon1[i].y;

            // ����������ֱ�ڱߣ�
            double axis_x = -edge_y;
            double axis_y = edge_x;

            // ��һ��������
            double length = std::sqrt(axis_x * axis_x + axis_y * axis_y);
            if (length > 0) {
                axis_x /= length;
                axis_y /= length;
            }

            // ��������������ڸ����ϵ�ͶӰ��Χ
            double min1, max1, min2, max2;
            projectPolygon(polygon1, axis_x, axis_y, min1, max1);
            projectPolygon(polygon2, axis_x, axis_y, min2, max2);

            // ���ͶӰ�Ƿ��ص���������ص����ҵ��˷�����
            if (max1 < min2 || max2 < min1) {
                return false;
            }
        }

        // ���������϶����ص�
        return true;
    }

    void CollisionDetector::projectPolygon(const std::vector<cv::Point2d>& polygon,
        double axis_x, double axis_y,
        double& min_proj, double& max_proj) {
        min_proj = std::numeric_limits<double>::max();
        max_proj = std::numeric_limits<double>::lowest();

        for (const auto& point : polygon) {
            // �������ϵ�ͶӰ
            double projection = point.x * axis_x + point.y * axis_y;

            if (projection < min_proj) min_proj = projection;
            if (projection > max_proj) max_proj = projection;
        }
    }

} // namespace planning