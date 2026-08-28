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
#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/tracking.hpp>
#include <string>
#include <chrono>

#include "common/common.h"
#include "utils/gason.h"
#include "common/box.h"
#include "common/target.h"
#include "common/targets_in_frame.h"

#include "framework/detector.h"

#if CHECK_OPENCV_VERSION(4, 7, 0)
#include <opencv2/objdetect/aruco_detector.hpp>
#endif

namespace sunray_detection
{
  struct CameraInfo
  {
    cv::Mat camera_matrix;
    int height;
    int width;
    std::string distortion_model;
    cv::Mat distCoeffs;
  };

  class ArucoDetector: public BaseDetector
  {
  public:
    ArucoDetector();
    ~ArucoDetector(){};
    void detect(const cv::Mat &img_, TargetsInFrame &tgts_);
    void getIdsWithLengths(std::vector<int> &ids_, std::vector<double> &lengths_);
    void loadCameraParams(std::string yaml_fn_);
    void loadAlgorithmParams(std::string json_fn_);
    void loadCameraParams(const CameraInfo &camera_info);
    cv::Mat camera_matrix;
    cv::Mat distortion;
    int image_width_;
    int image_height_;
    double fov_x;
    double fov_y;
    std::string alg_params_fn;

  private:
    void _load();
    bool _params_loaded;
    cv::Ptr<cv::aruco::DetectorParameters> _detector_params;
    cv::Ptr<cv::aruco::Dictionary> _dictionary;
    int _dictionary_id;
    std::vector<int> _ids_need;
    std::vector<double> _lengths_need;

  protected:
    std::chrono::system_clock::time_point _t0;
  };
}
