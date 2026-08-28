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
#include "ellipse_detector.h"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(sunray_detection::ellipse_detection_ros::EllipseDetector, nodelet::Nodelet);

namespace sunray_detection
{

    namespace ellipse_detection_ros
    {
        void EllipseDetector::onInit()
        {
            // Initialize the node handle and image transport
            ros::NodeHandle &nh = getNodeHandle();
            ros::NodeHandle &pnh = getPrivateNodeHandle();
            ed_ = std::shared_ptr<sunray_detection::EllipseDetector>(new sunray_detection::EllipseDetector());
            ed_->_params_loaded = false;
            it_ = std::shared_ptr<image_transport::ImageTransport>(
                new image_transport::ImageTransport(nh));

            std::string transport_hint;
            pnh.param<std::string>("transport_hint", transport_hint, "raw");

            // Load parameters from the parameter server
            pnh.getParam("draw_ellipse_detections_image", draw_ellipse_detections_image_);
            pnh.getParam("local_saving_path", local_saving_path_);
            pnh.getParam("algorithm_params_json", algorithm_params_json_);
            pnh.getParam("camera_params_yaml", camera_params_yaml_);
            pnh.getParam("max_center_distance_ratio", this->_max_center_distance_ratio);
            pnh.getParam("max_candidates", max_candidates_);
            ROS_INFO_STREAM("Draw ellipse detections image: " << draw_ellipse_detections_image_);
            ROS_INFO_STREAM("Loaded algorithm parameters from: " << algorithm_params_json_);
            ROS_INFO_STREAM("Loaded camera parameters from: " << camera_params_yaml_);
            ROS_INFO_STREAM("Local saving path: " << local_saving_path_);

            // Subscribe to image and camera info topics
            camera_image_subscriber_ =
                it_->subscribeCamera("image_rect", 1,
                                     &EllipseDetector::imageCallback, this,
                                     image_transport::TransportHints(transport_hint));
            refresh_params_service_ =
                pnh.advertiseService("refresh_tag_params",
                                     &EllipseDetector::refreshParamsCallback, this);
            // Advertise for detection results
            ellipse_detections_publisher_ =
                nh.advertise<detection_msgs::TargetsInFrameMsg>("/sunray_detect/ellipse_detection_ros", 1);
            // If desired, advertise the detection image
            if (draw_ellipse_detections_image_)
            {
                ellipse_detections_image_publisher_ = it_->advertise("/sunray_detect/ellipse_detection_ros/image_rect", 1);
            }
            ed_->loadAlgorithmParams(algorithm_params_json_);
            ed_->loadCameraParams(camera_params_yaml_);

            NODELET_INFO("EllipseDetector initialized.");
        }
        void EllipseDetector::imageCallback(const sensor_msgs::ImageConstPtr &image_rect,
                                            const sensor_msgs::CameraInfoConstPtr &camera_info)
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            cv::Mat cp_image; // 拷贝图像 绘制框用

            try
            {

                cp_image = cv_bridge::toCvCopy(image_rect, image_rect->encoding)->image;
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            auto targets = detectEllipses(cp_image, camera_info);
            targets.header.stamp = camera_info->header.stamp;
            targets.header.seq = camera_info->header.seq;
            ellipse_detections_publisher_.publish(targets);

            // If drawing detection results to the image is enabled, publish the debug image
            if (draw_ellipse_detections_image_)
            {
                ellipse_detections_image_publisher_.publish(cv_bridge::CvImage(image_rect->header, "rgb8", cp_image).toImageMsg());
            }
        }

        detection_msgs::TargetsInFrameMsg EllipseDetector::detectEllipses(cv::Mat &image,
                                                                          const sensor_msgs::CameraInfoConstPtr &camera_info)
        {
            if(!ed_->_params_loaded)
            {
                ed_->_load();
                ed_->_params_loaded = true;
            }
            TargetsInFrame tgts(frame_id_++);
            detect(image, tgts);
            detection_msgs::TargetsInFrameMsg ellipses_msg;
            ellipses_msg.frame_id = tgts.frame_id;
            ellipses_msg.height = tgts.height;
            ellipses_msg.width = tgts.width;
            ellipses_msg.fps = tgts.fps;
            ellipses_msg.fov_x = tgts.fov_x;
            ellipses_msg.fov_y = tgts.fov_y;
            if (draw_ellipse_detections_image_)
                drawTargetsInFrame(image, tgts);

            for (size_t i = 0; i < tgts.targets.size(); i++)
            {
                detection_msgs::TargetMsg target_msg;
                target_msg.cx = tgts.targets[i].cx;
                target_msg.cy = tgts.targets[i].cy;
                target_msg.w = tgts.targets[i].w;
                target_msg.h = tgts.targets[i].h;

                target_msg.score = 1.0f;
                target_msg.category = tgts.targets[i].category;
                target_msg.category_id = tgts.targets[i].category_id;

                target_msg.los_ax = tgts.targets[i].los_ax;
                target_msg.los_ay = tgts.targets[i].los_ay;

                target_msg.px = static_cast<float>(tgts.targets[i].px);
                target_msg.py = static_cast<float>(tgts.targets[i].py);
                target_msg.pz = static_cast<float>(tgts.targets[i].pz);
                ellipses_msg.targets.push_back(target_msg);
            }
            return ellipses_msg;
        }

        void EllipseDetector::refreshEllipsesParameters()
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            ed_.reset(new sunray_detection::EllipseDetector());
            ed_->loadAlgorithmParams(algorithm_params_json_);
            ed_->loadCameraParams(camera_params_yaml_);
        }

        bool EllipseDetector::refreshParamsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
        {
            refreshEllipsesParameters();
            NODELET_INFO("Ellipse detector parameters refreshed.");
            return true;
        }

        void EllipseDetector::_loadLabels()
        {
            JsonValue all_value;
            JsonAllocator allocator;
            _load_all_json(ed_->alg_params_fn, all_value, allocator);

            JsonValue landing_params_value;
            _parser_algorithm_params("LandingMarkerDetector", all_value, landing_params_value);

            for (auto i : landing_params_value)
            {
                if ("labels" == std::string(i->key) && i->value.getTag() == JSON_ARRAY)
                {
                    for (auto j : i->value)
                    {
                        this->_labels_need.push_back(j->value.toString());
                    }
                }
                else if ("maxCandidates" == std::string(i->key))
                {
                    this->max_candidates_ = i->value.toNumber();
                    // std::cout << "maxCandidates: " << this->_max_candidates << std::endl;
                }
            }
        }
        void EllipseDetector::detect(cv::Mat image, TargetsInFrame &tgts_)
        {

            // 计算最大中心距离并设置
            float fMaxCenterDistance = sqrt(static_cast<float>(image.cols * image.cols + image.rows * image.rows)) * _max_center_distance_ratio;
            ed_->SetMCD(fMaxCenterDistance);

            // 检测椭圆
            std::vector<Ellipse> ellsCned;
            ed_->Detect(image, ellsCned);
            // 初始化目标帧
            tgts_.setSize(image.cols, image.rows);
            tgts_.setFOV(ed_->fov_x, ed_->fov_y);
            // 计算帧率并设置
            auto t1 = std::chrono::system_clock::now();
            tgts_.setFPS(1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(t1 - _t0).count());
            _t0 = t1; // 更新时间戳
            tgts_.setTimeNow();

            static std::vector<std::string> s_label2str = {"neg", "h", "x", "1", "2", "3", "4", "5", "6", "7", "8"};
            size_t cand_cnt = 0;
            std::vector<cv::Mat> input_rois;
            while (cand_cnt < this->max_candidates_ && ellsCned.size() > cand_cnt)
            {
                Ellipse e = ellsCned[cand_cnt++];
                // 直接跳过分类，赋予默认标签
                int label = 1;

                // 创建目标对象并设置属性
                Target tgt;
                tgt.setEllipse(e.xc_, e.yc_, e.a_, e.b_, e.rad_, e.score_, tgts_.width, tgts_.height, ed_->camera_matrix, ed_->_radius_in_meter);
                tgt.setCategory("H", label); // 使用默认标签 "default" 和默认编号
                tgts_.targets.push_back(tgt);
            }
            // 设置任务类型为地标检测
            tgts_.type = MissionType::ELLIPSE_DET;
        }
    } // namespace ellipse_detection_ros
} // namespace sunray_detection
