#include "BezierCurve.h"
#include <cmath>

namespace math {

    std::vector<BezierPoint> BezierCurve::calculateCubicBezier(
        const std::vector<cv::Point2d>& control_points,
        int num_points) {

        std::vector<BezierPoint> points;

        for (int i = 0; i < num_points; i++) {
            double t = static_cast<double>(i) / (num_points - 1);
            BezierPoint point = evaluateCubicBezier(control_points, t);
            points.push_back(point);
        }

        return points;
    }

    BezierPoint BezierCurve::evaluateCubicBezier(
        const std::vector<cv::Point2d>& control_points,
        double t) {

        // 确保有4个控制点（三阶贝塞尔曲线）
        if (control_points.size() != 4) {
            throw std::invalid_argument("三阶贝塞尔曲线需要4个控制点");
        }

        // 贝塞尔曲线公式: B(t) = (1-t)^3 * P0 + 3(1-t)^2 * t * P1 + 3(1-t) * t^2 * P2 + t^3 * P3
        double t1 = 1.0 - t;
        double t2 = t1 * t1;
        double t3 = t2 * t1;

        double b0 = t3;
        double b1 = 3.0 * t2 * t;
        double b2 = 3.0 * t1 * t * t;
        double b3 = t * t * t;

        BezierPoint point;
        point.x = b0 * control_points[0].x + b1 * control_points[1].x + b2 * control_points[2].x + b3 * control_points[3].x;
        point.y = b0 * control_points[0].y + b1 * control_points[1].y + b2 * control_points[2].y + b3 * control_points[3].y;

        // 计算一阶导数用于航向角
        double dx_dt = 3.0 * t1 * t1 * (control_points[1].x - control_points[0].x) +
            6.0 * t1 * t * (control_points[2].x - control_points[1].x) +
            3.0 * t * t * (control_points[3].x - control_points[2].x);

        double dy_dt = 3.0 * t1 * t1 * (control_points[1].y - control_points[0].y) +
            6.0 * t1 * t * (control_points[2].y - control_points[1].y) +
            3.0 * t * t * (control_points[3].y - control_points[2].y);

        // 计算航向角（弧度）
        point.heading = std::atan2(dy_dt, dx_dt);

        // 计算二阶导数用于曲率
        double d2x_dt2 = 6.0 * t1 * (control_points[2].x - 2.0 * control_points[1].x + control_points[0].x) +
            6.0 * t * (control_points[3].x - 2.0 * control_points[2].x + control_points[1].x);

        double d2y_dt2 = 6.0 * t1 * (control_points[2].y - 2.0 * control_points[1].y + control_points[0].y) +
            6.0 * t * (control_points[3].y - 2.0 * control_points[2].y + control_points[1].y);

        // 计算曲率：k = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
        double denom = std::pow(dx_dt * dx_dt + dy_dt * dy_dt, 1.5);
        if (std::abs(denom) > 1e-6) {
            point.curvature = (dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / denom;
        }
        else {
            point.curvature = 0.0;
        }

        return point;
    }

} // namespace math