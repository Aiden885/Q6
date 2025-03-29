#include <iostream>
#include <opencv2/highgui.hpp>
#include <thread>
#include <chrono>
#include "Types.h"
#include "ObstacleAvoidanceController.h"

// 全局变量
bool g_running = true;
int g_scene_type = 1; // 默认场景1：车道内避障
std::vector<common::Obstacle> g_obstacles;
bool g_params_changed = false;
bool g_auto_run = false;
bool g_reset_vehicle = false;

int main() {
    // 初始化车辆参数
    common::VehicleParams vehicle_params;
    vehicle_params.width = 2.0;  // 车宽2米
    vehicle_params.length = 5.0; // 车长5米
    vehicle_params.wheelbase = 2.8; // 轴距2.8米
    vehicle_params.max_steering_angle = 0.8; // 约34+度

    // 初始化车道参数 - 修改为两车道
    common::LaneParams lane_params;
    lane_params.width = 7.0; // 车道宽7米
    lane_params.num_lanes = 2; // 2车道

    // 创建避障规划器
    planning::ObstacleAvoidanceController controller(vehicle_params, lane_params);
    visualization::Visualizer visualizer(vehicle_params, lane_params);

    // 初始化车辆状态 - 放在下车道
    common::VehicleState initial_state;
    initial_state.x = 0.0;
    initial_state.y = 3.5; // 下车道中央 (7.0/2)
    initial_state.heading = 0.0;
    initial_state.velocity = 8.0; // 初始速度8m/s
    initial_state.current_lane = 0; // 在下车道
    controller.setVehicleState(initial_state);

    // 初始化障碍物 - 根据默认场景
    g_obstacles = controller.initializeObstaclesForScene(static_cast<common::SceneType>(g_scene_type));
    controller.setObstacles(g_obstacles);

    // 规划参数
    double planning_distance = 70.0; // 规划50米
    int num_trajectories = 7; // 生成7条轨迹

    // 创建窗口
    cv::namedWindow("自动驾驶避障轨迹规划", cv::WINDOW_NORMAL);
    cv::namedWindow("控制面板", cv::WINDOW_NORMAL);
    cv::resizeWindow("控制面板", 400, 300);

    // 执行初始轨迹规划
    common::Trajectory selected_trajectory = controller.planTrajectory(
        planning_distance, num_trajectories, static_cast<common::SceneType>(g_scene_type));

    // 主循环
    const double dt = 0.1; // 时间步长0.1秒
    std::chrono::time_point<std::chrono::steady_clock> last_time = std::chrono::steady_clock::now();

    while (g_running) {
        // 计算时间增量
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - last_time;
        last_time = current_time;
        double frame_dt = elapsed.count();
        if (frame_dt > 0.5) frame_dt = 0.1; // 防止时间步长过大

        // 检查是否需要重置车辆
        if (g_reset_vehicle) {
            controller.setVehicleState(initial_state);
            g_reset_vehicle = false;
            g_params_changed = true;
        }

        // 检查参数是否改变，需要重新规划
        if (g_params_changed) {
            controller.setObstacles(g_obstacles);
            selected_trajectory = controller.planTrajectory(
                planning_distance, num_trajectories, static_cast<common::SceneType>(g_scene_type));
            g_params_changed = false;
        }

        // 自动运行模式下更新车辆状态
        if (g_auto_run) {
            controller.updateVehicleState(dt, selected_trajectory);

            // 检查是否需要重新规划（车辆移动后）
            if (controller.getVehicleState().x > planning_distance * 1) {
                g_reset_vehicle = true; // 车辆行驶超过规划距离的60%时重置
                continue;
            }
        }

        // 可视化结果
        cv::Mat visualization = controller.visualize(
            controller.getLatestTrajectories(), selected_trajectory);

        // 更新控制面板
        visualizer.updateControlPanel(g_scene_type, g_obstacles);
        cv::Mat control_panel = visualizer.getControlPanel();

        // 显示结果
        cv::imshow("自动驾驶避障轨迹规划", visualization);
        cv::imshow("控制面板", control_panel);

        // 处理按键
        int key = cv::waitKey(30); // 30ms刷新率，约33fps
        if (key != -1) {
            // 处理基本按键操作
            // 重置车辆位置
            if (key == 'z' || key == 'Z') {
                g_reset_vehicle = true;
            }
            // 切换自动/手动运行
            else if (key == ' ') { // 空格键
                g_auto_run = !g_auto_run;
                std::cout << (g_auto_run ? "自动运行模式" : "手动控制模式") << std::endl;
            }

            // 处理场景切换
            else if (key == '1' || key == '2') {
                g_scene_type = (key == '1') ? 1 : 2;
                g_obstacles = controller.initializeObstaclesForScene(static_cast<common::SceneType>(g_scene_type));
                g_params_changed = true;
                std::cout << "切换到场景 " << g_scene_type << std::endl;
            }

            // 处理障碍物参数调整
            else {
                bool handled = false;

                // 如果有障碍物，调整选中障碍物的参数
                if (!g_obstacles.empty()) {
                    static int selected_obstacle = 0;
                    if (selected_obstacle < g_obstacles.size()) {
                        common::Obstacle& obs = g_obstacles[selected_obstacle];

                        // X位置
                        if (key == 'a') {
                            obs.position.x -= 1.0;
                            handled = true;
                        }
                        else if (key == 'd') {
                            obs.position.x += 1.0;
                            handled = true;
                        }

                        // Y位置
                        if (key == 'w') {
                            obs.position.y += 0.5;
                            handled = true;
                        }
                        else if (key == 's') {
                            obs.position.y -= 0.5;
                            handled = true;
                        }

                        // 宽度
                        if (key == 'q') {
                            obs.width = std::max(0.5, obs.width - 0.5);
                            handled = true;
                        }
                        else if (key == 'e') {
                            obs.width += 0.5;
                            handled = true;
                        }

                        // 长度
                        if (key == 'r') {
                            obs.length += 0.5;
                            handled = true;
                        }
                        else if (key == 'f') {
                            obs.length = std::max(0.5, obs.length - 0.5);
                            handled = true;
                        }
                    }
                }

                if (handled) {
                    g_params_changed = true;
                }
            }
        }

        // 稍微延迟，减少CPU占用
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    cv::destroyAllWindows();
    return 0;
}