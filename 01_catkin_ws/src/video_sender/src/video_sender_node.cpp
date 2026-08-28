#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <string>
#include <sstream>

class VideoSender
{
public:
    VideoSender()
        : nh_(),
          pnh_("~"),
          writer_initialized_(false)
    {
        pnh_.param<std::string>(
            "image_topic",
            image_topic_,
            "/usb_cam/image_raw"
        );

        pnh_.param<std::string>(
            "target_ip",
            target_ip_,
            "192.168.1.100"
        );

        pnh_.param<int>(
            "target_port",
            target_port_,
            9696
        );

        pnh_.param<int>(
            "width",
            width_,
            1280
        );

        pnh_.param<int>(
            "height",
            height_,
            720
        );

        pnh_.param<int>(
            "fps",
            fps_,
            30
        );

        pnh_.param<int>(
            "bitrate",
            bitrate_,
            3000
        );

        image_sub_ = nh_.subscribe(
            image_topic_,
            1,
            &VideoSender::imageCallback,
            this
        );

        ROS_INFO("================================");
        ROS_INFO("Video sender initialized");
        ROS_INFO("Image topic : %s", image_topic_.c_str());
        ROS_INFO("Target IP   : %s", target_ip_.c_str());
        ROS_INFO("Target Port : %d", target_port_);
        ROS_INFO("Resolution  : %dx%d", width_, height_);
        ROS_INFO("FPS         : %d", fps_);
        ROS_INFO("Bitrate     : %d kbps", bitrate_);
        ROS_INFO("================================");
    }

    ~VideoSender()
    {
        if (writer_.isOpened())
        {
            writer_.release();
        }
    }

private:
    bool initWriter()
    {
        std::stringstream pipeline;

        pipeline
            << "appsrc "
            << "is-live=true "
            << "format=time "
            << "do-timestamp=true "
            << "! videoconvert "
            << "! video/x-raw,format=I420 "
            << "! x264enc "
            << "tune=zerolatency "
            << "speed-preset=ultrafast "
            << "bitrate=" << bitrate_ << " "
            << "key-int-max=" << fps_ << " "
            << "bframes=0 "
            << "! h264parse "
            << "! rtph264pay "
            << "config-interval=1 "
            << "pt=96 "
            << "! udpsink "
            << "host=" << target_ip_ << " "
            << "port=" << target_port_ << " "
            << "sync=false "
            << "async=false";

        ROS_INFO("Opening GStreamer pipeline:");
        ROS_INFO("%s", pipeline.str().c_str());

        bool success = writer_.open(
            pipeline.str(),
            cv::CAP_GSTREAMER,
            0,
            static_cast<double>(fps_),
            cv::Size(width_, height_),
            true
        );

        if (!success)
        {
            ROS_ERROR("Failed to open GStreamer pipeline.");
            ROS_ERROR("Please check whether OpenCV has GStreamer support.");
            return false;
        }

        writer_initialized_ = true;

        ROS_INFO("GStreamer pipeline opened successfully.");

        return true;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvShare(
                msg,
                sensor_msgs::image_encodings::BGR8
            );
        }
        catch (const cv_bridge::Exception& e)
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "cv_bridge exception: %s",
                e.what()
            );
            return;
        }

        if (cv_ptr->image.empty())
        {
            ROS_WARN_THROTTLE(
                1.0,
                "Received empty frame."
            );
            return;
        }

        cv::Mat frame;

        if (cv_ptr->image.cols != width_ ||
            cv_ptr->image.rows != height_)
        {
            cv::resize(
                cv_ptr->image,
                frame,
                cv::Size(width_, height_)
            );
        }
        else
        {
            frame = cv_ptr->image;
        }

        if (!writer_initialized_)
        {
            if (!initWriter())
            {
                ROS_ERROR_THROTTLE(
                    2.0,
                    "Video writer initialization failed."
                );
                return;
            }
        }

        if (!writer_.isOpened())
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "Video writer is not opened."
            );
            return;
        }

        writer_.write(frame);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber image_sub_;

    cv::VideoWriter writer_;

    bool writer_initialized_;

    std::string image_topic_;
    std::string target_ip_;

    int target_port_;
    int width_;
    int height_;
    int fps_;
    int bitrate_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_sender_node");

    VideoSender sender;

    ros::spin();

    return 0;
}
