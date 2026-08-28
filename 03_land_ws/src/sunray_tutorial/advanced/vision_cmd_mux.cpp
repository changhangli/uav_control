/*
 * 指令复用：不改 catkin 源码，通过 launch remap 把 demo 航线与视觉节点分开，
 * 由本节点按 pause_demo 选择转发哪一路到 uav_control_cmd（同 catkin 里 Land 让航线停发的思路）。
 *
 * demo 节点应 remap：uav_control_cmd -> demo_control_cmd
 * 视觉 listen 节点发布到：vision_control_cmd
 * pause_demo=true 时只转发 vision；false 时只转发 demo
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <swarm_msgs/UAVControlCMD.h>

static swarm_msgs::UAVControlCMD last_demo;
static swarm_msgs::UAVControlCMD last_vision;
static bool have_demo{false};
static bool have_vision{false};
static bool vision_active{false};

static void demo_cb(const swarm_msgs::UAVControlCMD::ConstPtr &msg)
{
    last_demo = *msg;
    have_demo = true;
}

static void vision_cb(const swarm_msgs::UAVControlCMD::ConstPtr &msg)
{
    last_vision = *msg;
    have_vision = true;
}

static void pause_cb(const std_msgs::Bool::ConstPtr &msg)
{
    vision_active = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_cmd_mux");
    ros::NodeHandle nh("~");

    int uav_id = 1;
    std::string uav_name = "uav";
    nh.param<int>("uav_id", uav_id, 1);
    nh.param<std::string>("uav_name", uav_name, "uav");

    const std::string prefix = "/" + uav_name + std::to_string(uav_id);
    const std::string demo_topic = prefix + "/demo_control_cmd";
    const std::string vision_topic = prefix + "/vision_control_cmd";
    const std::string out_topic = prefix + "/uav_control_cmd";
    const std::string pause_topic = prefix + "/pause_demo";

    ros::Subscriber sub_demo = nh.subscribe(demo_topic, 10, demo_cb);
    ros::Subscriber sub_vision = nh.subscribe(vision_topic, 10, vision_cb);
    ros::Subscriber sub_pause = nh.subscribe(pause_topic, 1, pause_cb);

    ros::Publisher pub = nh.advertise<swarm_msgs::UAVControlCMD>(out_topic, 1);

    ROS_WARN("[MUX] demo=%s vision=%s out=%s pause=%s",
             demo_topic.c_str(), vision_topic.c_str(), out_topic.c_str(), pause_topic.c_str());
    printf("[MUX] 启动：pause=true 转发视觉，pause=false 转发 demo 航线\n");

    ros::Rate rate(20.0);
    while (ros::ok())
    {
        ros::spinOnce();
        swarm_msgs::UAVControlCMD out;
        if (vision_active && have_vision)
            out = last_vision;
        else if (!vision_active && have_demo)
            out = last_demo;
        else if (have_vision || have_demo)
        {
            // Offboard 需要连续 setpoint：首选源暂时没有时发 Hover，禁止静默停发
            out = swarm_msgs::UAVControlCMD();
            out.cmd = swarm_msgs::UAVControlCMD::Hover;
            ROS_WARN_THROTTLE(2.0,
                              "[MUX] preferred source missing (vision_active=%d), publishing Hover",
                              (int)vision_active);
        }
        else
        {
            // 启动后尚未收到任何指令，无法安全伪造
            rate.sleep();
            continue;
        }
        out.header.stamp = ros::Time::now();
        pub.publish(out);
        rate.sleep();
    }
    return 0;
}
