/*************************************************************************************************************************
 * Copyright 2024 Grifcc&Kylin
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *************************************************************************************************************************/
#include <memory>
#include <mutex>

#include <nodelet/nodelet.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "common/common.h"
#include "common/targets_in_frame.h"
#include "common/ellipse.h"
#include "utils/gason.h"

#include "inference_backend/opencv/ellipse/ellipse_detector.h"

#include <detection_msgs/TargetMsg.h>
#include <detection_msgs/TargetsInFrameMsg.h>

namespace sunray_detection
{
    namespace ellipse_detection_ros
    {
        class EllipseDetector : public nodelet::Nodelet
        {
        public:
            EllipseDetector() = default;
            ~EllipseDetector() = default;
            void onInit() override;

            void imageCallback(const sensor_msgs::ImageConstPtr &image_rect,
                               const sensor_msgs::CameraInfoConstPtr &camera_info);
            detection_msgs::TargetsInFrameMsg detectEllipses(cv::Mat &image,
                                                             const sensor_msgs::CameraInfoConstPtr &camera_info);
            void refreshEllipsesParameters();

        private:
            std::shared_ptr<sunray_detection::EllipseDetector> ed_;
            std::mutex detection_mutex_;
            bool draw_ellipse_detections_image_ = false;
            std::string local_saving_path_;
            cv::Mat cv_image_;
            int frame_id_;
            std::string algorithm_params_json_;
            std::string camera_params_yaml_;
            std::vector<int> ids_;
            bool ids_loaded_ = false;

            std::shared_ptr<image_transport::ImageTransport> it_;
            image_transport::CameraSubscriber camera_image_subscriber_;
            ros::Publisher ellipse_detections_publisher_;
            image_transport::Publisher ellipse_detections_image_publisher_;

            ros::ServiceServer refresh_params_service_;
            bool refreshParamsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
            int max_candidates_ = 20;

            std::chrono::system_clock::time_point _t0;
            void detect(cv::Mat image, TargetsInFrame &tgts_);

        protected:
            void _loadLabels();
            bool _params_loaded  = false;
            float _max_center_distance_ratio = 0.05f;
            double _radius_in_meter;
            std::vector<std::string> _labels_need;
        };
    }
}
