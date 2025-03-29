#ifndef BEZIER_CURVE_H
#define BEZIER_CURVE_H

#include <vector>
#include <opencv2/opencv.hpp>

namespace math {

    // 贝塞尔曲线点结构体
    struct BezierPoint {
        double x;
        double y;
        double heading; // 航向角（弧度）
        double curvature; // 曲率
    };

    class BezierCurve {
    public:
        // 计算三阶贝塞尔曲线
        static std::vector<BezierPoint> calculateCubicBezier(
            const std::vector<cv::Point2d>& control_points,
            int num_points);

        // 计算贝塞尔曲线的一个点
        static BezierPoint evaluateCubicBezier(
            const std::vector<cv::Point2d>& control_points,
            double t);
    };

} // namespace math

#endif // BEZIER_CURVE_H