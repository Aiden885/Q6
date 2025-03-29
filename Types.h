#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "BezierCurve.h"

namespace common {

    // ���������ṹ��
    struct VehicleParams {
        double width;  // ����
        double length; // ����
        double wheelbase; // ���
        double max_steering_angle; // ���ת���
    };

    // ���������ṹ��
    struct LaneParams {
        double width;  // �������
        int num_lanes; // ��������
    };

    // ����״̬�ṹ��
    struct VehicleState {
        double x;
        double y;
        double heading; // �������򣨻��ȣ�
        double velocity; // ����
        int current_lane; // ��ǰ����
    };

    // �ϰ���ṹ��
    struct Obstacle {
        cv::Point2d position; // �ϰ�������λ��
        double width;  // �ϰ�����
        double length; // �ϰ��ﳤ��
        double heading; // �ϰ��ﳯ�򣨻��ȣ�
        bool is_static; // �Ƿ�Ϊ��̬�ϰ���
        cv::Point2d velocity; // �ϰ����ٶ�����������Ǿ�̬��
    };

    // �켣�ṹ��
    struct Trajectory {
        std::vector<math::BezierPoint> points;
        bool has_collision;
        double cost; // �켣�����Ĵ���
    };

    // ��������
    enum SceneType {
        IN_LANE_AVOIDANCE = 1,  // �����ڱ���
        LANE_CHANGE = 2         // ��������
    };

} // namespace common

#endif // TYPES_H