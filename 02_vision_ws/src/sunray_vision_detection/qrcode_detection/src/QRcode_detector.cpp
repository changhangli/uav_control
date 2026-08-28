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
#include "QRcode_detector.h"
#include "id_value.h"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(sunray_detection::qrcode_detection_ros::QRcodeDetector, nodelet::Nodelet);

namespace sunray_detection
{

    namespace qrcode_detection_ros
    {
        void QRcodeDetector::onInit()
        {
            ros::NodeHandle &nh = getNodeHandle();
            ros::NodeHandle &pnh = getPrivateNodeHandle();

            tag_detector_ = std::shared_ptr<ArucoDetector>(new ArucoDetector());
            it_ = std::shared_ptr<image_transport::ImageTransport>(
                new image_transport::ImageTransport(nh));

            std::string transport_hint;
            pnh.param<std::string>("transport_hint", transport_hint, "raw");

            int queue_size;
            int uav_id;
            std::string uav_name; 
            pnh.param<int>("queue_size", queue_size, 1);
            pnh.getParam("uav_id", uav_id);
            pnh.param<std::string>("uav_name", uav_name, "uav");
            pnh.getParam("x_bias", x_bias_);
            pnh.getParam("y_bias", y_bias_);
            pnh.getParam("draw_tag_detections_image", draw_tag_detections_image_);
            pnh.getParam("algorithm_parameters", algorithm_params_json_);
            pnh.getParam("camera_parameters", camera_params_yaml_);
            pnh.getParam("local_saving_path", local_saving_path_);

            pnh.getParam("rotate", rotate_);
            
            uav_name = "/"+ uav_name  + std::to_string(uav_id);

            ROS_INFO_STREAM("Loaded uav_id: " << uav_id);
            ROS_INFO_STREAM("Loaded x_bias: " << x_bias_);
            ROS_INFO_STREAM("Loaded y_bias: " << y_bias_);
            ROS_INFO_STREAM("Draw tag detections image: " << draw_tag_detections_image_);
            ROS_INFO_STREAM("Loaded algorithm parameters from: " << algorithm_params_json_);
            ROS_INFO_STREAM("Loaded camera parameters from: " << camera_params_yaml_);
            ROS_INFO_STREAM("Local saving path: " << local_saving_path_);

            camera_image_subscriber_ =
                it_->subscribeCamera("image_rect", queue_size,
                                     &QRcodeDetector::imageCallback, this,
                                     image_transport::TransportHints(transport_hint));
            tag_detections_publisher_ =
                nh.advertise<detection_msgs::TargetsInFrameMsg>(uav_name + "/sunray_detect/qrcode_detection_ros", 1);

            tag_detections_image_publisher_ = it_->advertise(uav_name + "/sunray_detect/qrcode_detection_ros/image_rect", 1); 

            refresh_params_service_ =
                pnh.advertiseService("refresh_tag_params",
                                     &QRcodeDetector::refreshParamsCallback, this);

            tag_detector_->loadAlgorithmParams(algorithm_params_json_);
            tag_detector_->loadCameraParams(camera_params_yaml_);
        }

        void QRcodeDetector::refreshTagParameters()
        {
            // Resetting the tag detector will cause a new param server lookup
            // So if the parameters have changed (by someone/something),
            // they will be updated dynamically
            std::lock_guard<std::mutex> lock(detection_mutex_);
            tag_detector_.reset(new ArucoDetector());
            tag_detector_->loadAlgorithmParams(algorithm_params_json_);
            tag_detector_->loadCameraParams(camera_params_yaml_);
        }

        bool QRcodeDetector::refreshParamsCallback(std_srvs::Empty::Request &req,
                                                   std_srvs::Empty::Response &res)
        {
            refreshTagParameters();
            return true;
        }
        void QRcodeDetector::imageCallback(
            const sensor_msgs::ImageConstPtr &image_rect,
            const sensor_msgs::CameraInfoConstPtr &camera_info)
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            cv::Mat cp_image;   // 拷贝图像 绘制框用
            try
            {
                if (cv_bridge::toCvCopy(image_rect, image_rect->encoding)->image.channels() == 1)
                {
                    cp_image = cv_bridge::toCvCopy(image_rect, image_rect->encoding)->image;
                    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding)->image;
                    if(rotate_)
                    {
                        switch(rotate_){
                        case 1://顺时针180°
                            cv::rotate(cv_image_, cv_image_, cv::ROTATE_180);
                            cv::rotate(cp_image, cp_image, cv::ROTATE_180);
                            break;
                        case 2://顺时针90°
                            cv::rotate(cv_image_, cv_image_, cv::ROTATE_90_CLOCKWISE);
                            cv::rotate(cp_image, cp_image, cv::ROTATE_90_CLOCKWISE);
                            break;
                        case 3://逆时针90°
                            cv::rotate(cv_image_, cv_image_, cv::ROTATE_90_COUNTERCLOCKWISE);
                            cv::rotate(cp_image, cp_image, cv::ROTATE_90_COUNTERCLOCKWISE);
                            break;
                        }
                    }
                }
                else
                {
                    cp_image = cv_bridge::toCvCopy(image_rect, image_rect->encoding)->image;
                    cv::cvtColor(cv_bridge::toCvCopy(image_rect, image_rect->encoding)->image, cv_image_, CV_BGR2GRAY);
                    if(rotate_)
                    {
                        switch(rotate_){
                        case 1://顺时针180°
                            cv::rotate(cv_image_, cv_image_, cv::ROTATE_180);
                            cv::rotate(cp_image, cp_image, cv::ROTATE_180);
                            break;
                        case 2://顺时针90°
                            cv::rotate(cv_image_, cv_image_, cv::ROTATE_90_CLOCKWISE);
                            cv::rotate(cp_image, cp_image, cv::ROTATE_90_CLOCKWISE);
                            break;
                        case 3://逆时针90°
                            cv::rotate(cv_image_, cv_image_, cv::ROTATE_90_COUNTERCLOCKWISE);
                            cv::rotate(cp_image, cp_image, cv::ROTATE_90_COUNTERCLOCKWISE);
                            break;
                        }
                    }
                }
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            auto targets = detectTags(cp_image, camera_info);
            targets.header.stamp = camera_info->header.stamp;
            targets.header.seq = camera_info->header.seq;
            tag_detections_publisher_.publish(targets);
            if (draw_tag_detections_image_)
            {
                if(cp_image.channels() == 1){
                    tag_detections_image_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", cp_image).toImageMsg());
                }
                else
                    tag_detections_image_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "rgb8", cp_image).toImageMsg());
                // cv::imwrite(local_saving_path_ + "/" + std::to_string(frame_id_) + ".jpg", cv_image_);
            }
        }
        detection_msgs::TargetsInFrameMsg QRcodeDetector::detectTags(cv::Mat &img,
                                                                     const sensor_msgs::CameraInfoConstPtr &camera_info)
        {
            TargetsInFrame tgts(frame_id_++);
            // 执行Aruco二维码检测
            tag_detector_->detect(cv_image_, tgts);
            if (!ids_loaded_)
            {
                ids_loaded_ = true;
                tag_detector_->getIdsWithLengths(ids_, lengths_);
                ROS_INFO_STREAM("ids: " << ids_.size() << ", lengths: " << lengths_.size());
            }

            detection_msgs::TargetsInFrameMsg ros_tgts;
            ros_tgts.frame_id = tgts.frame_id;
            ros_tgts.height = tgts.height;
            ros_tgts.width = tgts.width;
            ros_tgts.fps = tgts.fps;
            ros_tgts.fov_x = tgts.fov_x;
            ros_tgts.fov_y = tgts.fov_y;

            // 控制台打印Aruco检测结果
            ROS_INFO_STREAM("Frame-[" << frame_id_ << "]\n");
            // 打印当前检测的FPS
            ROS_INFO_STREAM("  FPS =" << tgts.fps << "\n");
            // 打印当前相机的视场角（degree）
            ROS_INFO_STREAM("  FOV (fx, fy) = (" << tgts.fov_x << ", " << tgts.fov_y << ")\n");

            // 可视化检测结果，叠加到img上
            if (draw_tag_detections_image_)
            {
                drawTargetsInFrame(img, tgts);
            }
            std::vector<bool> is_bigids;

            for (int i = 0; i < static_cast<int>(tgts.targets.size()); i++)
            {
                int id;
                std::vector<cv::Point2f> corners;
                cv::Vec3d rvecs;
                cv::Vec3d tvecs;
                tgts.targets[i].getAruco(id, corners, rvecs, tvecs);
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvecs, rotation_matrix);

                float id2c_t[3];
                if (fill_value_from_id(id2c_t, tgts.targets[i].category_id, lengths_[0] / 4., x_bias_, y_bias_))
                {
                    cv::Mat id2c_t_mat = cv::Mat(3, 1, CV_32FC1, id2c_t);
                    rotation_matrix.convertTo(rotation_matrix, CV_32FC1);
                    // cv::invert(rotation_matrix, rotation_matrix);

                    std::vector<double> vec_t{tvecs[0], tvecs[1], tvecs[2]};
                    cv::Mat vec_t_mat{vec_t};
                    vec_t_mat.convertTo(vec_t_mat, CV_32FC1);
                    // 计算标定板子原点，在当前二维码下到坐标: rotation_matrix * id2c_t_mat
                    // 机体坐标系下，到标定版中点的坐标: cent_t
                    cv::Mat cent_t = rotation_matrix * id2c_t_mat + vec_t_mat;

                    detection_msgs::TargetMsg ros_target;
                    ros_target.cx = tgts.targets[i].cx;
                    ros_target.cy = tgts.targets[i].cy;
                    ros_target.w = tgts.targets[i].w;
                    ros_target.h = tgts.targets[i].h;

                    ros_target.score = 1.0f;
                    ros_target.category = tgts.targets[i].category;
                    ros_target.category_id = tgts.targets[i].category_id;

                    ros_target.yaw = tgts.targets[i].yaw;
                    ros_target.pitch = tgts.targets[i].pitch;
                    ros_target.roll = tgts.targets[i].roll;
                    ros_target.tracked_id = tgts.targets[i].tracked_id;

                    ros_target.los_ax = tgts.targets[i].los_ax;
                    ros_target.los_ay = tgts.targets[i].los_ay;

                    ros_target.px = tgts.targets[i].px;
                    ros_target.py = tgts.targets[i].py;
                    ros_target.pz = tgts.targets[i].pz;
                    ros_tgts.targets.push_back(ros_target);

                    ROS_INFO_STREAM("Frame-[" << frame_id_ << "], Aruco-[" << i << "]");
                    //  打印每个二维码的中心位置，cx，cy的值域为[0, 1]
                    ROS_INFO_STREAM("  Aruco Center (cx, cy) = (" << tgts.targets[i].cx << "," << tgts.targets[i].cy << ")");
                    //  打印每个二维码的外接矩形框的宽度、高度，w，h的值域为(0, 1]
                    ROS_INFO_STREAM("  Aruco Size (w, h) = (" << tgts.targets[i].w << "," << tgts.targets[i].h << ")");
                    //  打印每个二维码的偏航角，值域为[-180, 180]
                    ROS_INFO_STREAM("  Aruco Yaw-angle = " << tgts.targets[i].yaw);
                     //  打印每个二维码的俯仰角，值域为[-90, 90]
                    ROS_INFO_STREAM("  Aruco Pitch-angle = " << tgts.targets[i].pitch);
                     //  打印每个二维码的横滚角，值域为[-180, 180]
                    ROS_INFO_STREAM("  Aruco Roll-angle = " << tgts.targets[i].roll);
                    //  打印每个二维码的类别，字符串类型，"aruco-?"
                    ROS_INFO_STREAM("  Aruco Category = " << tgts.targets[i].category.c_str());
                    //  打印每个二维码的ID号
                    ROS_INFO_STREAM("  Aruco Tracked-ID = " << tgts.targets[i].tracked_id);
                    //  打印每个二维码的视线角，跟相机视场相关
                    ROS_INFO_STREAM("  Aruco Line-of-sight (ax, ay) = (" << tgts.targets[i].los_ax << "," << tgts.targets[i].los_ay << ")");
                    //  打印每个二维码的3D位置（在相机坐标系下），跟二维码实际边长、相机参数相关
                    ROS_INFO_STREAM("  -- Aruco Position = (x, y, z) = (" << tgts.targets[i].px << "," << tgts.targets[i].py << "," << tgts.targets[i].pz << ")");
                    ROS_INFO_STREAM("  Aruco Position = (x, y, z) = (" << cent_t.at<float>(0) << "," << cent_t.at<float>(2) << "," << tgts.targets[i].pz << ")");
                    if (tgts.targets[i].category_id > 90)
                        is_bigids.push_back(true);
                    else
                        is_bigids.push_back(false);
                }
            }
            return ros_tgts;
        }
    }
}
