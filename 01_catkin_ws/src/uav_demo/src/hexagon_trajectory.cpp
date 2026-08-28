/*
程序功能：使用XyzPos接口进行六边形轨迹飞行
*/
#include <ros/ros.h>
#include <printf_format.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"
#include <csignal>

using namespace sunray_logger;
using namespace std;

swarm_msgs::UAVState uav_state;
swarm_msgs::UAVControlCMD uav_cmd;
swarm_msgs::UAVSetup uav_setup;
string node_name;

void mySigintHandler(int sig)
{
    std::cout << "[hexagon_trajectory] exit..." << std::endl;
    ros::shutdown();
    exit(EXIT_SUCCESS);
}

void uav_state_cb(const swarm_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

int main(int argc, char **argv)
{
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "hexagon_trajectory");
    ros::NodeHandle nh("~");

    ros::Rate rate(20.0);
    node_name = ros::this_node::getName();

    int uav_id;
    signal(SIGINT, mySigintHandler);

    string uav_name, target_topic_name;
    bool sim_mode, flag_printf;

    nh.param<int>("uav_id", uav_id, 1);
    nh.param<string>("uav_name", uav_name, "uav");

    uav_name = "/" + uav_name + std::to_string(uav_id);
    ros::Subscriber uav_state_sub = nh.subscribe<swarm_msgs::UAVState>(uav_name + "/swarm/uav_state", 1, uav_state_cb);
    ros::Publisher control_cmd_pub = nh.advertise<swarm_msgs::UAVControlCMD>(uav_name + "/uav_control_cmd", 1);
    ros::Publisher uav_setup_pub = nh.advertise<swarm_msgs::UAVSetup>(uav_name + "/setup", 1);

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
    int times = 0;
    while (ros::ok() && !uav_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            Logger::print_color(int(LogColor::red), node_name, ": Wait for UAV connect...");
    }
    Logger::print_color(int(LogColor::green), node_name, ": UAV connected!");

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
        // 【异常检测】如果解锁阶段被切出 CMD_CONTROL，直接退出
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

    while (ros::ok() && abs(uav_state.position[2] - uav_state.home_pos[2] - uav_state.takeoff_height) > 0.2)
    {
        // 【异常检测】起飞阶段被接管或意外降落上锁，直接退出
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
    // 【异常检测】休眠后同步一下状态，判断期间是否被接管
    if (!uav_state.armed || uav_state.control_mode != swarm_msgs::UAVSetup::CMD_CONTROL) return 0;

    Logger::print_color(int(LogColor::green), node_name, ": Send UAV Hover cmd.");
    uav_cmd.cmd = swarm_msgs::UAVControlCMD::Hover;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(2).sleep();
    ros::spinOnce();
    // 【异常检测】悬停休眠后同步状态判断
    if (!uav_state.armed || uav_state.control_mode != swarm_msgs::UAVSetup::CMD_CONTROL) return 0;

    // Define hexagon parameters
    double center_x = 0;
    double center_y = 0;
    double radius = 1.2;  // circumscribed circle radius
    double height = 0.8;   // fixed altitude
    int num_sides = 6;    // number of hexagon vertices

    // Generate the 6 vertices of the hexagon (counterclockwise)
    vector<geometry_msgs::Point> hexagon_points;
    for (int i = 0; i < num_sides; i++)
    {
        double theta = i * 2 * M_PI / num_sides - M_PI / 2.0; // start from top vertex
        geometry_msgs::Point p;
        p.x = center_x + radius * cos(theta);
        p.y = center_y + radius * sin(theta);
        p.z = height;
        hexagon_points.push_back(p);
    }

    // Add first point at the end to close the hexagon
    hexagon_points.push_back(hexagon_points[0]);

    // Define square parameters
    double max_vel_xy = 1.0;  // 最大速度限制（用于判断是否到达）

    geometry_msgs::PoseStamped pose;
    Logger::print_color(int(LogColor::green), node_name, ": Start hexagon trajectory.");

    for (size_t i = 0; i < hexagon_points.size(); i++)
    {
        pose.pose.position.x = hexagon_points[i].x;
        pose.pose.position.y = hexagon_points[i].y;
        pose.pose.position.z = hexagon_points[i].z;

        Logger::print_color(int(LogColor::green), node_name,
            (string(": Heading to point ") + to_string(i + 1) + string("/") + to_string(hexagon_points.size())).c_str());

        while (ros::ok())
        {
            // 【异常检测】飞向目标点途中，若被接管或上锁则退出
            if (!uav_state.armed || uav_state.control_mode != swarm_msgs::UAVSetup::CMD_CONTROL) {
                Logger::print_color(int(LogColor::red), node_name, ": Manual takeover or disarmed! Aborting hexagon task...");
                return 0;
            }

            // 直接发送目标位置，由底层控制器规划速度
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = swarm_msgs::UAVControlCMD::XyzPos;
            uav_cmd.desired_pos[0] = pose.pose.position.x;
            uav_cmd.desired_pos[1] = pose.pose.position.y;
            uav_cmd.desired_pos[2] = pose.pose.position.z;
            control_cmd_pub.publish(uav_cmd);

            // 判断是否到达目标点
            double dist_xy = sqrt(
                pow(uav_state.position[0] - pose.pose.position.x, 2) +
                pow(uav_state.position[1] - pose.pose.position.y, 2));

            if (dist_xy < 0.15)
            {
                break;
            }

            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }

        // 到达目标点后停留1秒
        ros::Duration(1.0).sleep();
        ros::spinOnce();
        // 【异常检测】停留期间若被接管则退出
        if (!uav_state.armed || uav_state.control_mode != swarm_msgs::UAVSetup::CMD_CONTROL) return 0;
    }

    // Return to origin
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = height;
    while (ros::ok())
    {
        // 【异常检测】返回原点途中若被接管或上锁则退出
        if (!uav_state.armed || uav_state.control_mode != swarm_msgs::UAVSetup::CMD_CONTROL) {
            Logger::print_color(int(LogColor::red), node_name, ": Manual takeover or disarmed! Aborting return to origin...");
            return 0;
        }

        // 直接发送目标位置
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = swarm_msgs::UAVControlCMD::XyzPos;
        uav_cmd.desired_pos[0] = pose.pose.position.x;
        uav_cmd.desired_pos[1] = pose.pose.position.y;
        uav_cmd.desired_pos[2] = pose.pose.position.z;
        control_cmd_pub.publish(uav_cmd);

        double dist_xy = sqrt(
            pow(uav_state.position[0] - pose.pose.position.x, 2) +
            pow(uav_state.position[1] - pose.pose.position.y, 2));

        if (dist_xy < 0.2)
        {
            ros::Duration(1.0).sleep();
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    // Land UAV
    while (ros::ok() && uav_state.control_mode != swarm_msgs::UAVSetup::LAND_CONTROL && uav_state.landed_state != 1)
    {
        // 【异常检测】如果在发降落指令前就已经上锁（强制落地/停机），退出
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
        // 【异常检测】如果等待降落期间上锁了，退出
        if (!uav_state.armed) {
            Logger::print_color(int(LogColor::yellow), node_name, ": UAV disarmed. Exiting.");
            return 0;
        }

        Logger::print_color(int(LogColor::green), node_name, ": Landing");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    Logger::print_color(int(LogColor::green), node_name, ": Land UAV successfully!");
    Logger::print_color(int(LogColor::green), node_name, ": Demo finished, quit!");
    return 0;
}
