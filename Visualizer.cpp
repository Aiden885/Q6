#include "Visualizer.h"

namespace visualization {

    Visualizer::Visualizer(const common::VehicleParams& vehicle_params, const common::LaneParams& lane_params)
        : vehicle_params_(vehicle_params), lane_params_(lane_params),
        selected_obstacle_(0), obstacle_x_slider_(0), obstacle_y_slider_(0),
        obstacle_width_slider_(0), obstacle_length_slider_(0) {

        // 创建控制面板
        createControlPanel();
    }

    cv::Mat Visualizer::visualize(
        const common::VehicleState& vehicle_state,
        const std::vector<common::Obstacle>& obstacles,
        const std::vector<common::Trajectory>& trajectories,
        const common::Trajectory& selected_trajectory,
        bool lane_change_mode
    ) {
        // 创建画布，大小根据规划距离调整
        int canvas_width = 800;
        int canvas_height = 600;
        cv::Mat canvas(canvas_height, canvas_width, CV_8UC3, cv::Scalar(255, 255, 255));

        // 设置可视化参数
        double scale = 10.0; // 缩放因子
        cv::Point2d origin(100, canvas_height / 2); // 原点位置

        // 绘制车道线
        drawLanes(canvas, origin, scale);

        // 绘制所有候选轨迹
        for (const auto& trajectory : trajectories) {
            cv::Scalar color = trajectory.has_collision ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
            drawTrajectory(canvas, trajectory, color, 1, origin, scale);
        }

        // 绘制选中的轨迹
        if (!selected_trajectory.points.empty()) {
            drawTrajectory(canvas, selected_trajectory, cv::Scalar(255, 0, 0), 3, origin, scale);
        }

        // 绘制车辆
        drawVehicle(canvas, vehicle_state, origin, scale);

        // 绘制障碍物
        for (const auto& obstacle : obstacles) {
            drawObstacle(canvas, obstacle, origin, scale);
        }

        // 添加信息文本
        std::string info = lane_change_mode ? "Mode: Lane Change" : "Mode: In-Lane Avoidance";
        cv::putText(canvas, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);

        // 检查是否所有轨迹都有碰撞
        bool all_collisions = true;
        for (const auto& traj : trajectories) {
            if (!traj.has_collision) {
                all_collisions = false;
                break;
            }
        }

        // 如果所有轨迹都有碰撞，添加警告提示
        if (all_collisions && !trajectories.empty()) {
            cv::putText(canvas, "WARNING: All trajectories have collision risk!",
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            cv::putText(canvas, "Selected trajectory is farthest from obstacles in Y direction",
                cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        }

        return canvas;
    }

    void Visualizer::drawVehicle(cv::Mat& canvas, const common::VehicleState& state, const cv::Point2d& origin, double scale) {
        double half_length = vehicle_params_.length / 2.0 * scale;
        double half_width = vehicle_params_.width / 2.0 * scale;

        double cos_heading = std::cos(state.heading);
        double sin_heading = std::sin(state.heading);

        cv::Point2d vehicle_pos(origin.x + state.x * scale,
            origin.y - state.y * scale);

        std::vector<cv::Point> vehicle_corners = {
            cv::Point(vehicle_pos.x + cos_heading * half_length - sin_heading * half_width,
                     vehicle_pos.y - (sin_heading * half_length + cos_heading * half_width)),
            cv::Point(vehicle_pos.x + cos_heading * half_length + sin_heading * half_width,
                     vehicle_pos.y - (sin_heading * half_length - cos_heading * half_width)),
            cv::Point(vehicle_pos.x - cos_heading * half_length + sin_heading * half_width,
                     vehicle_pos.y - (-sin_heading * half_length - cos_heading * half_width)),
            cv::Point(vehicle_pos.x - cos_heading * half_length - sin_heading * half_width,
                     vehicle_pos.y - (-sin_heading * half_length + cos_heading * half_width))
        };

        std::vector<std::vector<cv::Point>> contours = { vehicle_corners };
        cv::drawContours(canvas, contours, 0, cv::Scalar(0, 0, 255), -1);

        // 添加车辆前进方向指示
        cv::Point front_center(vehicle_pos.x + cos_heading * half_length,
            vehicle_pos.y - sin_heading * half_length);
        cv::arrowedLine(canvas, vehicle_pos, front_center, cv::Scalar(255, 255, 255), 2);
    }

    void Visualizer::drawObstacle(cv::Mat& canvas, const common::Obstacle& obstacle, const cv::Point2d& origin, double scale) {
        cv::Point2d obs_pos(origin.x + obstacle.position.x * scale,
            origin.y - obstacle.position.y * scale);

        double obs_half_length = obstacle.length / 2.0 * scale;
        double obs_half_width = obstacle.width / 2.0 * scale;

        double obs_cos_heading = std::cos(obstacle.heading);
        double obs_sin_heading = std::sin(obstacle.heading);

        std::vector<cv::Point> obs_corners = {
            cv::Point(obs_pos.x + obs_cos_heading * obs_half_length - obs_sin_heading * obs_half_width,
                     obs_pos.y - (obs_sin_heading * obs_half_length + obs_cos_heading * obs_half_width)),
            cv::Point(obs_pos.x + obs_cos_heading * obs_half_length + obs_sin_heading * obs_half_width,
                     obs_pos.y - (obs_sin_heading * obs_half_length - obs_cos_heading * obs_half_width)),
            cv::Point(obs_pos.x - obs_cos_heading * obs_half_length + obs_sin_heading * obs_half_width,
                     obs_pos.y - (-obs_sin_heading * obs_half_length - obs_cos_heading * obs_half_width)),
            cv::Point(obs_pos.x - obs_cos_heading * obs_half_length - obs_sin_heading * obs_half_width,
                     obs_pos.y - (-obs_sin_heading * obs_half_length + obs_cos_heading * obs_half_width))
        };

        std::vector<std::vector<cv::Point>> obs_contours = { obs_corners };
        cv::drawContours(canvas, obs_contours, 0, cv::Scalar(0, 0, 0), -1);
    }

    void Visualizer::drawTrajectory(cv::Mat& canvas, const common::Trajectory& trajectory, const cv::Scalar& color,
        int thickness, const cv::Point2d& origin, double scale) {
        for (size_t i = 0; i < trajectory.points.size() - 1; i++) {
            cv::Point pt1(origin.x + trajectory.points[i].x * scale,
                origin.y - trajectory.points[i].y * scale);
            cv::Point pt2(origin.x + trajectory.points[i + 1].x * scale,
                origin.y - trajectory.points[i + 1].y * scale);
            cv::line(canvas, pt1, pt2, color, thickness);
        }
    }

    void Visualizer::drawLanes(cv::Mat& canvas, const cv::Point2d& origin, double scale) {
        int canvas_width = canvas.cols;

        // 只绘制两条车道
        int num_lanes = 2;

        // 绘制两条车道的下边界线
        int y = origin.y;
        cv::line(canvas, cv::Point(0, y), cv::Point(canvas_width, y), cv::Scalar(0, 0, 0), 2);

        // 绘制两条车道的上边界线
        y = origin.y - num_lanes * lane_params_.width * scale;
        cv::line(canvas, cv::Point(0, y), cv::Point(canvas_width, y), cv::Scalar(0, 0, 0), 2);

        // 绘制中间的虚线分隔线
        y = origin.y - lane_params_.width * scale;
        for (int x = 0; x < canvas_width; x += 20) {
            cv::line(canvas, cv::Point(x, y), cv::Point(x + 10, y), cv::Scalar(0, 0, 0), 2);
        }

        // 绘制车道中心线
        // 下车道中心线
        y = origin.y - lane_params_.width * scale / 2;
        cv::Scalar centerline_color(100, 100, 100); // 灰色中心线
        for (int x = 0; x < canvas_width; x += 30) {
            cv::line(canvas, cv::Point(x, y), cv::Point(x + 15, y), centerline_color, 1);
        }

        // 上车道中心线
        y = origin.y - lane_params_.width * scale * 3 / 2;
        for (int x = 0; x < canvas_width; x += 30) {
            cv::line(canvas, cv::Point(x, y), cv::Point(x + 15, y), centerline_color, 1);
        }
    }

    void Visualizer::createControlPanel() {
        // 创建控制面板窗口
        control_panel_ = cv::Mat(300, 400, CV_8UC3, cv::Scalar(240, 240, 240));

        // 默认参数
        selected_obstacle_ = 0;
        obstacle_x_slider_ = 30;
        obstacle_y_slider_ = 50;
        obstacle_width_slider_ = 20;
        obstacle_length_slider_ = 50;
    }

    void Visualizer::updateControlPanel(int scene_type, const std::vector<common::Obstacle>& obstacles) {
        // 清空控制面板
        control_panel_ = cv::Scalar(240, 240, 240);

        // 添加标题
        cv::putText(control_panel_, "Obstacle Avoidance Control", cv::Point(10, 20),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);

        // 场景选择
        std::string scene_str = (scene_type == 1) ? "Scene 1: In-Lane Avoidance" : "Scene 2: Lane Change";
        cv::putText(control_panel_, scene_str, cv::Point(10, 45),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        cv::putText(control_panel_, "Press 1/2 to switch scene", cv::Point(10, 65),
            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

        // 障碍物选择
        cv::putText(control_panel_, "Obstacle Parameters:", cv::Point(10, 95),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        for (size_t i = 0; i < obstacles.size(); i++) {
            std::string obs_str = "Obstacle " + std::to_string(i + 1);
            cv::Scalar color = cv::Scalar(0, 0, 0);
            cv::putText(control_panel_, obs_str, cv::Point(30, 115 + i * 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
        }
        cv::putText(control_panel_, "Use keys to adjust obstacle parameters", cv::Point(10, 175),
            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

        // 障碍物参数控制
        if (!obstacles.empty()) {
            const common::Obstacle& obs = obstacles[0]; // 使用第一个障碍物

            // 显示当前障碍物参数
            cv::putText(control_panel_, "Position X: " + std::to_string(int(obs.position.x)),
                cv::Point(10, 200), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
            cv::putText(control_panel_, "Position Y: " + std::to_string(int(obs.position.y)),
                cv::Point(10, 220), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
            cv::putText(control_panel_, "Width: " + std::to_string(obs.width),
                cv::Point(10, 240), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
            cv::putText(control_panel_, "Length: " + std::to_string(obs.length),
                cv::Point(10, 260), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

            // 按键说明
            cv::putText(control_panel_, "Use a/d: X position, w/s: Y position",
                cv::Point(10, 280), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
            cv::putText(control_panel_, "Use q/e: Width, r/f: Length, ESC: Exit",
                cv::Point(10, 300), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
        }
    }

    bool Visualizer::handleUserInput(int key, int& scene_type, std::vector<common::Obstacle>& obstacles) {
        // 场景切换
        if (key == '1') {
            scene_type = 1; // 车道内避障
            return true;
        }
        else if (key == '2') {
            scene_type = 2; // 换道避障
            return true;
        }

        // 修改障碍物参数
        if (!obstacles.empty()) {
            common::Obstacle& obs = obstacles[0]; // 始终使用第一个障碍物

            // X位置
            if (key == 'a') {
                obs.position.x -= 1.0;
                return true;
            }
            else if (key == 'd') {
                obs.position.x += 1.0;
                return true;
            }

            // Y位置
            if (key == 'w') {
                obs.position.y += 0.5;
                return true;
            }
            else if (key == 's') {
                obs.position.y -= 0.5;
                return true;
            }

            // 宽度
            if (key == 'q') {
                obs.width = std::max(0.5, obs.width - 0.5);
                return true;
            }
            else if (key == 'e') {
                obs.width += 0.5;
                return true;
            }

            // 长度
            if (key == 'r') {
                obs.length += 0.5;
                return true;
            }
            else if (key == 'f') {
                obs.length = std::max(0.5, obs.length - 0.5);
                return true;
            }
        }

        return false;
    }

} // namespace visualization