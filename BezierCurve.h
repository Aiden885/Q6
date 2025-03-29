#ifndef BEZIER_CURVE_H
#define BEZIER_CURVE_H

#include <vector>
#include <opencv2/opencv.hpp>

namespace math {

    // ���������ߵ�ṹ��
    struct BezierPoint {
        double x;
        double y;
        double heading; // ����ǣ����ȣ�
        double curvature; // ����
    };

    class BezierCurve {
    public:
        // �������ױ���������
        static std::vector<BezierPoint> calculateCubicBezier(
            const std::vector<cv::Point2d>& control_points,
            int num_points);

        // ���㱴�������ߵ�һ����
        static BezierPoint evaluateCubicBezier(
            const std::vector<cv::Point2d>& control_points,
            double t);
    };

} // namespace math

#endif // BEZIER_CURVE_H