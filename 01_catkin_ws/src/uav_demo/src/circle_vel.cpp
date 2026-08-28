/*
程序功能：使用ORCA接口进行圆形轨迹飞行 - 闭环反馈控制版本
*/
#include <ros/ros.h>
#include <printf_format.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"
#include <csignal>
#include <cmath>

using namespace sunray_logger;
using namespace std;

swarm_msgs::UAVState uav_state;
swarm_msgs::UAVControlCMD uav_cmd;
swarm_msgs::UAVSetup uav_setup;
string node_name;

// 发布者
ros::Publisher control_cmd_pub;
ros::Publisher uav_setup_pub;

// 圆形轨迹参数
double center_x = 0;
double center_y = 0;
double radius = 1.5;
double linear_speed = 0.25;
bool circle_start = false;
bool circle_finished = false;
ros::Time circle_start_time;
ros::Timer circle_timer;

// 用于闭环控制的物理角度与状态变量
double last_physical_angle = 0;
double physical_accumulated_angle = 0;
bool move_to_circle = false;  // 是否正在飞到圆周
double target_circle_x = 0;  // 圆周上目标点的x坐标
double target_circle_y = 0;  // 圆周上目标点的y坐标

void mySigintHandler(int sig)
{
    std::cout << "[circle_vel] exit..." << std::endl;
    ros::shutdown();
    exit(EXIT_SUCCESS);
}

void uav_state_cb(const swarm_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

// 计算角度差（考虑角度环绕）
double normalizeAngle(double diff)
{
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}

// 定时器回调：计算并发布圆形轨迹速度指令
void timer_circle_goal(const ros::TimerEvent& e)
{
    if (!move_to_circle && !circle_start)
        return;

    // 第一阶段：飞到圆周上的起始点
    if (move_to_circle)
    {
        double dx = target_circle_x - uav_state.position[0];
        double dy = target_circle_y - uav_state.position[1];
        double dist = sqrt(dx * dx + dy * dy);

        double k_p = 1.5;
        double max_vel = 0.5;
        double vx = k_p * dx;
        double vy = k_p * dy;
        vx = min(max(vx, -max_vel), max_vel);
        vy = min(max(vy, -max_vel), max_vel);

        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = swarm_msgs::UAVControlCMD::XyVelZPosYaw;
        uav_cmd.desired_vel[0] = vx;
        uav_cmd.desired_vel[1] = vy;
        uav_cmd.desired_pos[2] = 0.8;
        control_cmd_pub.publish(uav_cmd);

        if (dist < 0.1)
        {
            move_to_circle = false;
            circle_start = true;
            circle_start_time = ros::Time::now();
            
            // 初始化物理角度
            last_physical_angle = atan2(uav_state.position[1] - center_y, uav_state.position[0] - center_x);
            physical_accumulated_angle = 0; 
            
            Logger::print_color(int(LogColor::green), node_name, ": Arrived at circle! Start circling...");
        }
        return;
    }

    // 第二阶段：绕圆飞行 (闭环控制)
    if (circle_start)
    {
        // 1. 获取当前无人机相对于圆心的坐标
        double dx = uav_state.position[0] - center_x;
        double dy = uav_state.position[1] - center_y;
        
        // 2. 计算当前实际半径和实际物理角度
        double current_radius = sqrt(dx * dx + dy * dy);
        double current_angle = atan2(dy, dx);

        // 3. 计算角度增量并累加 (使用normalizeAngle处理越界/跳变问题)
        double angle_diff = normalizeAngle(current_angle - last_physical_angle);
        // 逆时针飞行，angle_diff为正，直接累加即可记录真实转过的角度
        physical_accumulated_angle += angle_diff;
        last_physical_angle = current_angle;

        // 4. 速度控制：径向反馈修正 + 切向前馈
        // 4.1 径向修正速度（如果半径偏大，向圆心压；如果偏小，向外扩）
        double radial_error = radius - current_radius;
        // 径向纠偏系数（P参数）。如果发现轨迹修正在抖动，可以稍微调小；如果回弹太慢，可以稍微调大。
        double k_r = 1.0; 
        double v_radial = k_r * radial_error;

        // 4.2 切向速度（沿着圆周前进）
        double v_tangential = linear_speed;

        // 4.3 速度合成转换到全局XY坐标系
        // 径向方向就是 current_angle
        // 切向方向是 current_angle + M_PI/2 (逆时针)
        double vx = v_radial * cos(current_angle) + v_tangential * cos(current_angle + M_PI / 2.0);
        double vy = v_radial * sin(current_angle) + v_tangential * sin(current_angle + M_PI / 2.0);

        // 发布控制指令
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = swarm_msgs::UAVControlCMD::XyVelZPosYaw;
        uav_cmd.desired_vel[0] = vx;
        uav_cmd.desired_vel[1] = vy;
        uav_cmd.desired_pos[2] = 0.8;
        control_cmd_pub.publish(uav_cmd);

        // 5. 根据物理累积角度判断是否完成一圈
        if (fabs(physical_accumulated_angle) >= 2.0 * M_PI)
        {
            circle_start = false;
            circle_finished = true;
            circle_timer.stop();
            Logger::print_color(int(LogColor::yellow), node_name, ": Circle finished! Total angle:", physical_accumulated_angle, "rad");
        }
    }
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "circle_vel");
    // 创建一个节点句柄，允许访问参数服务器。
    ros::NodeHandle nh("~");

    ros::Rate rate(20.0);
    node_name = ros::this_node::getName();

    int uav_id;

    signal(SIGINT, mySigintHandler);

    string uav_name, target_topic_name;
    bool sim_mode, flag_printf;

    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");

    // 无人机名和话题前缀
    uav_name = "/" + uav_name + std::to_string(uav_id);
    // 【订阅】无人机状态 -- from vision_pose
    ros::Subscriber uav_state_sub = nh.subscribe<swarm_msgs::UAVState>(uav_name + "/swarm/uav_state", 1, uav_state_cb);
    // 【发布】无人机控制指令
    control_cmd_pub = nh.advertise<swarm_msgs::UAVControlCMD>(uav_name + "/uav_control_cmd", 1);
    // 【发布】无人机设置指令
    uav_setup_pub = nh.advertise<swarm_msgs::UAVSetup>(uav_name + "/setup", 1);

    // 变量初始化
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = 102;
    uav_cmd.desired_pos[0] = 0.0;
    uav_cmd.desired_pos[1] = 0.0;
    uav_cmd.desired_pos[2] = 0.0;
    uav_cmd.desired_vel[0] = 0.0;
    uav_cmd.desired_vel[1] = 0.0;
    uav_cmd.desired_vel[2] = 0.0;
    uav_cmd.desired_acc[0] = 0.0;
    uav_cmd.desired_acc[1] = 0.0;
    uav_cmd.desired_acc[2] = 0.0;
    uav_cmd.desired_att[0] = 0.0;
    uav_cmd.desired_att[1] = 0.0;
    uav_cmd.desired_att[2] = 0.0;
    uav_cmd.desired_yaw = 0.0;
    uav_cmd.desired_yaw_rate = 0.0;

    ros::Duration(0.5).sleep();
    
    // 初始化检查：等待PX4连接
    int times = 0;
    while (ros::ok() && !uav_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            Logger::print_color(int(LogColor::red), node_name, ": Wait for UAV connect...");
    }
    Logger::print_color(int(LogColor::green), node_name, ": UAV connected!");

    // 切换到指令控制模式
    while (ros::ok() && uav_state.control_mode != swarm_msgs::UAVSetup::CMD_CONTROL)
    {
        uav_setup.cmd = swarm_msgs::UAVSetup::SET_CONTROL_MODE;
        uav_setup.control_mode = "CMD_CONTROL";
        uav_setup_pub.publish(uav_setup);
        Logger::print_color(int(LogColor::green), node_name, ": SET_CONTROL_MODE - [CMD_CONTROL]. ");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    Logger::print_color(int(LogColor::green), node_name, ": UAV control_mode set to [CMD_CONTROL] successfully!");

    // 解锁无人机
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 5 sec...");
    ros::Duration(1.0).sleep();
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 4 sec...");
    ros::Duration(1.0).sleep();
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 3 sec...");
    ros::Duration(1.0).sleep();
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 2 sec...");
    ros::Duration(1.0).sleep();
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 1 sec...");
    ros::Duration(1.0).sleep();
    while (ros::ok() && !uav_state.armed)
    {
        // 【异常判断】如果在解锁期间被遥控器切走模式，退出
        if (uav_state.control_mode != swarm_msgs::UAVSetup::CMD_CONTROL) {
            Logger::print_color(int(LogColor::red), node_name, ": Manual takeover! Aborting arming...");
            return 0;
        }

        uav_setup.cmd = swarm_msgs::UAVSetup::ARM;
        uav_setup_pub.publish(uav_setup);
        Logger::print_color(int(LogColor::green), node_name, ": Arm UAV now.");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV successfully!");

    // 起飞无人机
    while (ros::ok() && fabs(uav_state.position[2] - uav_state.home_pos[2] - uav_state.takeoff_height) > 0.2)
    {
        // 【异常判断】被切模式或异常上锁（降落/保护），退出
        if (!uav_state.armed || uav_state.control_mode != swarm_msgs::UAVSetup::CMD_CONTROL) {
            Logger::print_color(int(LogColor::red), node_name, ": Manual takeover or disarmed! Aborting takeoff...");
            return 0;
        }

        uav_cmd.cmd = swarm_msgs::UAVControlCMD::Takeoff;
        control_cmd_pub.publish(uav_cmd);
        Logger::print_color(int(LogColor::green), node_name, ": Takeoff UAV now.");
        ros::Duration(4.0).sleep();
        ros::spinOnce();
    }
    Logger::print_color(int(LogColor::green), node_name, ": Takeoff UAV successfully!");

    ros::Duration(2).sleep();
    ros::spinOnce();
    // 【异常判断】起飞后长休眠期间可能被接管
    if (!uav_state.armed || uav_state.control_mode != swarm_msgs::UAVSetup::CMD_CONTROL) return 0;

    // 悬停
    Logger::print_color(int(LogColor::green), node_name, ": Send UAV Hover cmd.");
    uav_cmd.cmd = swarm_msgs::UAVControlCMD::Hover;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(2).sleep();
    ros::spinOnce();
    // 【异常判断】悬停长休眠期间可能被接管
    if (!uav_state.armed || uav_state.control_mode != swarm_msgs::UAVSetup::CMD_CONTROL) return 0;

    // 设置圆形轨迹参数
    center_x = 0;
    center_y = 0;
    radius = 1.5;
    linear_speed = 0.25;  // m/s

    // 计算圆周上的起始点（0度位置）
    target_circle_x = center_x + radius;  // (1, 0)
    target_circle_y = center_y;

    Logger::print_color(int(LogColor::green), node_name, ": Move to circle point: (", target_circle_x, ", ", target_circle_y, ")");
    Logger::print_color(int(LogColor::green), node_name, ": Circle radius:", radius, "Speed:", linear_speed);

    // 创建定时器：每100ms计算并发布速度指令
    circle_timer = nh.createTimer(ros::Duration(0.1), timer_circle_goal);
    move_to_circle = true;
    circle_start = false;

    // 主循环：等待走完一圈
    while (ros::ok() && !circle_finished)
    {
        // 【异常判断】绕圆期间被遥控器接管或意外降落，退出节点
        if (!uav_state.armed || uav_state.control_mode != swarm_msgs::UAVSetup::CMD_CONTROL) {
            Logger::print_color(int(LogColor::red), node_name, ": Manual takeover or disarmed! Aborting circle...");
            circle_timer.stop();
            return 0;
        }

        ros::spinOnce();
        rate.sleep();
    }

    // 回到原点
    Logger::print_color(int(LogColor::yellow), node_name, ": Returning to origin...");

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.8;

    double k_p = 1.0;
    double max_vel = 0.5;

    while (ros::ok())
    {
        // 【异常判断】返回原点途中被接管
        if (!uav_state.armed || uav_state.control_mode != swarm_msgs::UAVSetup::CMD_CONTROL) {
            Logger::print_color(int(LogColor::red), node_name, ": Manual takeover or disarmed! Aborting return...");
            return 0;
        }

        double dx = pose.pose.position.x - uav_state.position[0];
        double dy = pose.pose.position.y - uav_state.position[1];

        double vx = k_p * dx;
        double vy = k_p * dy;

        vx = min(max(vx, -max_vel), max_vel);
        vy = min(max(vy, -max_vel), max_vel);

        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = swarm_msgs::UAVControlCMD::XyzVel;
        uav_cmd.desired_vel[0] = vx;
        uav_cmd.desired_vel[1] = vy;
        uav_cmd.desired_vel[2] = 0;
        control_cmd_pub.publish(uav_cmd);

        if (fabs(uav_state.position[0] - pose.pose.position.x) < 0.2 &&
            fabs(uav_state.position[1] - pose.pose.position.y) < 0.2)
        {
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    // 降落
    while (ros::ok() && uav_state.control_mode != swarm_msgs::UAVSetup::LAND_CONTROL && uav_state.landed_state != 1)
    {
        // 【异常判断】如果在发出降落指令前就已经上锁（强制落地/停机），直接退出
        if (!uav_state.armed) {
            Logger::print_color(int(LogColor::yellow), node_name, ": UAV is already disarmed. Exiting.");
            return 0;
        }

        uav_cmd.cmd = swarm_msgs::UAVControlCMD::Land;
        control_cmd_pub.publish(uav_cmd);
        Logger::print_color(int(LogColor::green), node_name, ": Land UAV now.");
        ros::Duration(4.0).sleep();
        ros::spinOnce();
    }

    while (ros::ok() && uav_state.landed_state != 1)
    {
        // 【异常判断】等降落完成时发现上锁了，直接退出（上锁即意味着降落完成或停机）
        if (!uav_state.armed) {
            Logger::print_color(int(LogColor::yellow), node_name, ": UAV disarmed. Exiting.");
            return 0;
        }

        Logger::print_color(int(LogColor::green), node_name, ": Landing");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }

    Logger::print_color(int(LogColor::green), node_name, ": Demo finished, quit!");
    return 0;
}
