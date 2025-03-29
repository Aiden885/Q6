#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include <vector>
#include "Types.h"

namespace planning {

    class CollisionDetector {
    public:
        explicit CollisionDetector(const common::VehicleParams& vehicle_params);

        void setObstacles(const std::vector<common::Obstacle>& obstacles);

        void checkCollisions(std::vector<common::Trajectory>& trajectories);

    private:
        bool checkPointCollision(const math::BezierPoint& point, const common::Obstacle& obstacle);

        // SAT算法相关函数
        bool checkSATCollision(const std::vector<cv::Point2d>& polygon1,
            const std::vector<cv::Point2d>& polygon2);

        bool checkSeparationOnAxes(const std::vector<cv::Point2d>& polygon1,
            const std::vector<cv::Point2d>& polygon2);

        void projectPolygon(const std::vector<cv::Point2d>& polygon,
            double axis_x, double axis_y,
            double& min_proj, double& max_proj);

        common::VehicleParams vehicle_params_;
        std::vector<common::Obstacle> obstacles_;
    };

} // namespace planning

#endif // COLLISION_DETECTOR_H