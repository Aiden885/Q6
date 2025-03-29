#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <opencv2/opencv.hpp>
#include "Types.h"

namespace visualization {

    class Visualizer {
    public:
        Visualizer(const common::VehicleParams& vehicle_params, const common::LaneParams& lane_params);

        // ���ӻ��켣�滮���
        cv::Mat visualize(
            const common::VehicleState& vehicle_state,
            const std::vector<common::Obstacle>& obstacles,
            const std::vector<common::Trajectory>& trajectories,
            const common::Trajectory& selected_trajectory,
            bool lane_change_mode
        );

        // �����û������������
        void createControlPanel();

        // ���¿������
        void updateControlPanel(int scene_type, const std::vector<common::Obstacle>& obstacles);

        // ��ȡ������崰��
        cv::Mat getControlPanel() const { return control_panel_; }

        // �����û�����
        bool handleUserInput(int key, int& scene_type, std::vector<common::Obstacle>& obstacles);

    private:
        // ���Ƴ���
        void drawVehicle(cv::Mat& canvas, const common::VehicleState& state, const cv::Point2d& origin, double scale);

        // �����ϰ���
        void drawObstacle(cv::Mat& canvas, const common::Obstacle& obstacle, const cv::Point2d& origin, double scale);

        // ���ƹ켣
        void drawTrajectory(cv::Mat& canvas, const common::Trajectory& trajectory, const cv::Scalar& color,
            int thickness, const cv::Point2d& origin, double scale);

        // ���Ƴ�����
        void drawLanes(cv::Mat& canvas, const cv::Point2d& origin, double scale);

        common::VehicleParams vehicle_params_;
        common::LaneParams lane_params_;

        // �������
        cv::Mat control_panel_;

        // ����������
        int selected_obstacle_; // ��ǰѡ�е��ϰ�������
        int obstacle_x_slider_; // �ϰ���Xλ�û���ֵ
        int obstacle_y_slider_; // �ϰ���Yλ�û���ֵ
        int obstacle_width_slider_; // �ϰ����Ȼ���ֵ
        int obstacle_length_slider_; // �ϰ��ﳤ�Ȼ���ֵ
    };

} // namespace visualization

#endif // VISUALIZER_H