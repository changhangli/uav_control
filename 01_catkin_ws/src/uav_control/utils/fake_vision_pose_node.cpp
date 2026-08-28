/*
 * fake_vision_pose_node
 *
 * 功能：向 /mavros/vision_pose/pose 发布假位置数据
 * 用途：在没有动捕/SLAM的环境下，模拟外部定位数据输入PX4
 *       用于测试 Mavros <-> PX4 的通信链路
 *
 * 使用方式：
 *   1. 先确保 PX4 飞控参数 EKF2_AID_MASK 启用了 Vision Position Fusion
 *   2. 启动 Mavros 连接飞控
 *   3. 启动本节点
 *   4. 解锁 + OFFBOARD 模式后即可发控制指令
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>

bool px4_connected = false;
bool px4_armed = false;
std::string px4_mode = "";

// PX4 连接状态回调
void px4_state_callback(const mavros_msgs::State::ConstPtr &msg)
{
    px4_connected = msg->connected;
    px4_armed = msg->armed;
    px4_mode = msg->mode;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_vision_pose_node");
    ros::NodeHandle nh("~");

    // 【参数】假数据的目标位置
    double fake_pos_x = 0.0, fake_pos_y = 0.0, fake_pos_z = 0.0;
    // 【参数】假数据的目标姿态（欧拉角 rad）
    double fake_yaw_deg = 0.0;
    // 【参数】发布频率
    double publish_rate = 50.0;
    // 【参数】是否打印状态
    bool print_state = true;

    nh.param<double>("fake_pos_x", fake_pos_x, 0.0);
    nh.param<double>("fake_pos_y", fake_pos_y, 0.0);
    nh.param<double>("fake_pos_z", fake_pos_z, 0.0);
    nh.param<double>("fake_yaw", fake_yaw_deg, 0.0);
    nh.param<double>("publish_rate", publish_rate, 50.0);
    nh.param<bool>("print_state", print_state, true);

    // 【订阅】监听 PX4 连接状态（可选，用于判断飞控是否连接）
    ros::Subscriber px4_state_sub = nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, px4_state_callback);

    // 【发布】假定位数据 -> mavros/vision_pose/pose
    // PX4 EKF 会融合这个数据（需要配置 EKF2_AID_MASK）
    ros::Publisher fake_vision_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/vision_pose/pose", 10);

    ros::Rate rate(publish_rate);
    geometry_msgs::PoseStamped fake_pose;

    // 初始化四元数（从 yaw 转换）
    // yaw=0 -> quaternion = (0, 0, 0, 1)
    fake_pose.pose.orientation.w = cos(fake_yaw_deg * M_PI / 180.0 / 2.0);
    fake_pose.pose.orientation.x = 0.0;
    fake_pose.pose.orientation.y = 0.0;
    fake_pose.pose.orientation.z = sin(fake_yaw_deg * M_PI / 180.0 / 2.0);

    // 等待 PX4 连接
    ROS_INFO("[fake_vision_pose_node] Waiting for PX4 connection...");
    while (ros::ok() && !px4_connected)
    {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("[fake_vision_pose_node] PX4 connected!");

    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();

        // 更新假数据
        fake_pose.header.stamp = ros::Time::now();
        fake_pose.header.frame_id = "map";
        fake_pose.pose.position.x = fake_pos_x;
        fake_pose.pose.position.y = fake_pos_y;
        fake_pose.pose.position.z = fake_pos_z;

        // 发布假数据
        fake_vision_pub.publish(fake_pose);

        // 定时打印状态
        if (print_state && ++count % 50 == 0)
        {
            ROS_INFO("------------------------------");
            ROS_INFO("[fake_vision_pose] PX4: [%s] ARMED=[%s] MODE=[%s]",
                     px4_connected ? "CONNECTED" : "DISCONNECTED",
                     px4_armed ? "true" : "false",
                     px4_mode.c_str());
            ROS_INFO("[fake_vision_pose] Publishing: pos=(%.3f, %.3f, %.3f) yaw=%.1f deg",
                     fake_pos_x, fake_pos_y, fake_pos_z, fake_yaw_deg);
            ROS_INFO("[fake_vision_pose] Topic: /mavros/vision_pose/pose @ %.0f Hz",
                     publish_rate);
        }

        rate.sleep();
    }

    return 0;
}
