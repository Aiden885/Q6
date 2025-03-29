#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <opencv2/opencv.hpp>
#include "Types.h"

namespace visualization {

    class Visualizer {
    public:
        Visualizer(const common::VehicleParams& vehicle_params, const common::LaneParams& lane_params);

        // 可视化轨迹规划结果
        cv::Mat visualize(
            const common::VehicleState& vehicle_state,
            const std::vector<common::Obstacle>& obstacles,
            const std::vector<common::Trajectory>& trajectories,
            const common::Trajectory& selected_trajectory,
            bool lane_change_mode
        );

        // 创建用户交互控制面板
        void createControlPanel();

        // 更新控制面板
        void updateControlPanel(int scene_type, const std::vector<common::Obstacle>& obstacles);

        // 获取控制面板窗口
        cv::Mat getControlPanel() const { return control_panel_; }

        // 处理用户输入
        bool handleUserInput(int key, int& scene_type, std::vector<common::Obstacle>& obstacles);

    private:
        // 绘制车辆
        void drawVehicle(cv::Mat& canvas, const common::VehicleState& state, const cv::Point2d& origin, double scale);

        // 绘制障碍物
        void drawObstacle(cv::Mat& canvas, const common::Obstacle& obstacle, const cv::Point2d& origin, double scale);

        // 绘制轨迹
        void drawTrajectory(cv::Mat& canvas, const common::Trajectory& trajectory, const cv::Scalar& color,
            int thickness, const cv::Point2d& origin, double scale);

        // 绘制车道线
        void drawLanes(cv::Mat& canvas, const cv::Point2d& origin, double scale);

        common::VehicleParams vehicle_params_;
        common::LaneParams lane_params_;

        // 控制面板
        cv::Mat control_panel_;

        // 控制面板参数
        int selected_obstacle_; // 当前选中的障碍物索引
        int obstacle_x_slider_; // 障碍物X位置滑块值
        int obstacle_y_slider_; // 障碍物Y位置滑块值
        int obstacle_width_slider_; // 障碍物宽度滑块值
        int obstacle_length_slider_; // 障碍物长度滑块值
    };

} // namespace visualization

#endif // VISUALIZER_H