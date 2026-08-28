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
#include "inference_backend/opencv/aruco/aruco_detector.h"

namespace sunray_detection
{
  // 构造检测器并初始化延迟加载状态。
  ArucoDetector::ArucoDetector()
  {
    // 参数延迟加载：首次 detect() 时再真正解析 json 并构造 OpenCV 检测器。
    _params_loaded = false;
    _dictionary = nullptr;
    this->_t0 = std::chrono::system_clock::now();
  }

  // 向外返回当前配置中缓存的 marker ID 与物理边长。
  void ArucoDetector::getIdsWithLengths(std::vector<int> &ids_, std::vector<double> &lengths_)
  {
    ids_ = this->_ids_need;
    lengths_ = this->_lengths_need;
  }

  // 从 json 文件读取并解析 ArUco 检测参数。
  void ArucoDetector::_load()
  {
    // 读取算法 json，并定位到 ArucoDetector 对应的配置块。
    JsonValue all_value;
    JsonAllocator allocator;
    std::cout << "Load: [" << this->alg_params_fn << "]" << std::endl;
    _load_all_json(this->alg_params_fn, all_value, allocator);

    JsonValue aruco_params_value;
    _parser_algorithm_params("ArucoDetector", all_value, aruco_params_value);

    _dictionary_id = 10;
    // _detector_params = aruco::DetectorParameters::create();
    // 将 json 中的各项参数映射到 OpenCV ArUco 检测参数对象。
    _detector_params = new cv::aruco::DetectorParameters;
    for (auto i : aruco_params_value)
    {
      if ("dictionaryId" == std::string(i->key))
      {
        // std::cout << "dictionary_id (old, new): " << _dictionary_id << ", " << i->value.toNumber() << std::endl;
        _dictionary_id = i->value.toNumber();
      }
      else if ("adaptiveThreshConstant" == std::string(i->key))
      {
        // std::cout << "adaptiveThreshConstant (old, new): " << _detector_params->adaptiveThreshConstant << ", " << i->value.toNumber() << std::endl;
        _detector_params->adaptiveThreshConstant = i->value.toNumber();
      }
      else if ("adaptiveThreshWinSizeMax" == std::string(i->key))
      {
        // std::cout << "adaptiveThreshWinSizeMax (old, new): " << _detector_params->adaptiveThreshWinSizeMax << ", " << i->value.toNumber() << std::endl;
        _detector_params->adaptiveThreshWinSizeMax = i->value.toNumber();
      }
      else if ("adaptiveThreshWinSizeMin" == std::string(i->key))
      {
        // std::cout << "adaptiveThreshWinSizeMin (old, new): " << _detector_params->adaptiveThreshWinSizeMin << ", " << i->value.toNumber() << std::endl;
        _detector_params->adaptiveThreshWinSizeMin = i->value.toNumber();
      }
      else if ("adaptiveThreshWinSizeStep" == std::string(i->key))
      {
        // std::cout << "adaptiveThreshWinSizeStep (old, new): " << _detector_params->adaptiveThreshWinSizeStep << ", " << i->value.toNumber() << std::endl;
        _detector_params->adaptiveThreshWinSizeStep = i->value.toNumber();
      }
      else if ("aprilTagCriticalRad" == std::string(i->key))
      {
        // std::cout << "aprilTagCriticalRad (old, new): " << _detector_params->aprilTagCriticalRad << ", " << i->value.toNumber() << std::endl;
        _detector_params->aprilTagCriticalRad = i->value.toNumber();
      }
      else if ("aprilTagDeglitch" == std::string(i->key))
      {
        // std::cout << "aprilTagDeglitch (old, new): " << _detector_params->aprilTagDeglitch << ", " << i->value.toNumber() << std::endl;
        _detector_params->aprilTagDeglitch = i->value.toNumber();
      }
      else if ("aprilTagMaxLineFitMse" == std::string(i->key))
      {
        // std::cout << "aprilTagMaxLineFitMse (old, new): " << _detector_params->aprilTagMaxLineFitMse << ", " << i->value.toNumber() << std::endl;
        _detector_params->aprilTagMaxLineFitMse = i->value.toNumber();
      }
      else if ("aprilTagMaxNmaxima" == std::string(i->key))
      {
        // std::cout << "aprilTagMaxNmaxima (old, new): " << _detector_params->aprilTagMaxNmaxima << ", " << i->value.toNumber() << std::endl;
        _detector_params->aprilTagMaxNmaxima = i->value.toNumber();
      }
      else if ("aprilTagMinClusterPixels" == std::string(i->key))
      {
        // std::cout << "aprilTagMinClusterPixels (old, new): " << _detector_params->aprilTagMinClusterPixels << ", " << i->value.toNumber() << std::endl;
        _detector_params->aprilTagMinClusterPixels = i->value.toNumber();
      }
      else if ("aprilTagMinWhiteBlackDiff" == std::string(i->key))
      {
        // std::cout << "aprilTagMinWhiteBlackDiff (old, new): " << _detector_params->aprilTagMinWhiteBlackDiff << ", " << i->value.toNumber() << std::endl;
        _detector_params->aprilTagMinWhiteBlackDiff = i->value.toNumber();
      }
      else if ("aprilTagQuadDecimate" == std::string(i->key))
      {
        // std::cout << "aprilTagQuadDecimate (old, new): " << _detector_params->aprilTagQuadDecimate << ", " << i->value.toNumber() << std::endl;
        _detector_params->aprilTagQuadDecimate = i->value.toNumber();
      }
      else if ("aprilTagQuadSigma" == std::string(i->key))
      {
        // std::cout << "aprilTagQuadSigma (old, new): " << _detector_params->aprilTagQuadSigma << ", " << i->value.toNumber() << std::endl;
        _detector_params->aprilTagQuadSigma = i->value.toNumber();
      }
      else if ("cornerRefinementMaxIterations" == std::string(i->key))
      {
        // std::cout << "cornerRefinementMaxIterations (old, new): " << _detector_params->cornerRefinementMaxIterations << ", " << i->value.toNumber() << std::endl;
        _detector_params->cornerRefinementMaxIterations = i->value.toNumber();
      }
      else if ("cornerRefinementMethod" == std::string(i->key))
      {
        // std::cout << "cornerRefinementMethod (old, new): " << _detector_params->cornerRefinementMethod << ", " << i->value.toNumber() << std::endl;
        // _detector_params->cornerRefinementMethod = i->value.toNumber();
        int method = (int)i->value.toNumber();
        if (method == 1)
        {
          _detector_params->cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX;
        }
        else if (method == 2)
        {
          _detector_params->cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_CONTOUR;
        }
        else if (method == 3)
        {
          _detector_params->cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_APRILTAG;
        }
        else
        {
          _detector_params->cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_NONE;
        }
      }
      else if ("cornerRefinementMinAccuracy" == std::string(i->key))
      {
        // std::cout << "cornerRefinementMinAccuracy (old, new): " << _detector_params->cornerRefinementMinAccuracy << ", " << i->value.toNumber() << std::endl;
        _detector_params->cornerRefinementMinAccuracy = i->value.toNumber();
      }
      else if ("cornerRefinementWinSize" == std::string(i->key))
      {
        // std::cout << "cornerRefinementWinSize (old, new): " << _detector_params->cornerRefinementWinSize << ", " << i->value.toNumber() << std::endl;
        _detector_params->cornerRefinementWinSize = i->value.toNumber();
      }
      else if ("detectInvertedMarker" == std::string(i->key))
      {
        bool json_tf = false;
        if (i->value.getTag() == JSON_TRUE)
          json_tf = true;
        // std::cout << "detectInvertedMarker (old, new): " << _detector_params->detectInvertedMarker << ", " << json_tf << std::endl;
        _detector_params->detectInvertedMarker = json_tf;
      }
      else if ("errorCorrectionRate" == std::string(i->key))
      {
        // std::cout << "errorCorrectionRate (old, new): " << _detector_params->errorCorrectionRate << ", " << i->value.toNumber() << std::endl;
        _detector_params->errorCorrectionRate = i->value.toNumber();
      }
      else if ("markerBorderBits" == std::string(i->key))
      {
        // std::cout << "markerBorderBits (old, new): " << _detector_params->markerBorderBits << ", " << i->value.toNumber() << std::endl;
        _detector_params->markerBorderBits = i->value.toNumber();
      }
      else if ("maxErroneousBitsInBorderRate" == std::string(i->key))
      {
        // std::cout << "maxErroneousBitsInBorderRate (old, new): " << _detector_params->maxErroneousBitsInBorderRate << ", " << i->value.toNumber() << std::endl;
        _detector_params->maxErroneousBitsInBorderRate = i->value.toNumber();
      }
      else if ("maxMarkerPerimeterRate" == std::string(i->key))
      {
        // std::cout << "maxMarkerPerimeterRate (old, new): " << _detector_params->maxMarkerPerimeterRate << ", " << i->value.toNumber() << std::endl;
        _detector_params->maxMarkerPerimeterRate = i->value.toNumber();
      }
      else if ("minCornerDistanceRate" == std::string(i->key))
      {
        // std::cout << "minCornerDistanceRate (old, new): " << _detector_params->minCornerDistanceRate << ", " << i->value.toNumber() << std::endl;
        _detector_params->minCornerDistanceRate = i->value.toNumber();
      }
      else if ("minDistanceToBorder" == std::string(i->key))
      {
        // std::cout << "minDistanceToBorder (old, new): " << _detector_params->minDistanceToBorder << ", " << i->value.toNumber() << std::endl;
        _detector_params->minDistanceToBorder = i->value.toNumber();
      }
      else if ("minMarkerDistanceRate" == std::string(i->key))
      {
        // std::cout << "minMarkerDistanceRate (old, new): " << _detector_params->minMarkerDistanceRate << ", " << i->value.toNumber() << std::endl;
        _detector_params->minMarkerDistanceRate = i->value.toNumber();
      }
      else if ("minMarkerPerimeterRate" == std::string(i->key))
      {
        // std::cout << "minMarkerPerimeterRate (old, new): " << _detector_params->minMarkerPerimeterRate << ", " << i->value.toNumber() << std::endl;
        _detector_params->minMarkerPerimeterRate = i->value.toNumber();
      }
      else if ("minOtsuStdDev" == std::string(i->key))
      {
        // std::cout << "minOtsuStdDev (old, new): " << _detector_params->minOtsuStdDev << ", " << i->value.toNumber() << std::endl;
        _detector_params->minOtsuStdDev = i->value.toNumber();
      }
      else if ("perspectiveRemoveIgnoredMarginPerCell" == std::string(i->key))
      {
        // std::cout << "perspectiveRemoveIgnoredMarginPerCell (old, new): " << _detector_params->perspectiveRemoveIgnoredMarginPerCell << ", " << i->value.toNumber() << std::endl;
        _detector_params->perspectiveRemoveIgnoredMarginPerCell = i->value.toNumber();
      }
      else if ("perspectiveRemovePixelPerCell" == std::string(i->key))
      {
        // std::cout << "perspectiveRemovePixelPerCell (old, new): " << _detector_params->perspectiveRemovePixelPerCell << ", " << i->value.toNumber() << std::endl;
        _detector_params->perspectiveRemovePixelPerCell = i->value.toNumber();
      }
      else if ("polygonalApproxAccuracyRate" == std::string(i->key))
      {
        // std::cout << "polygonalApproxAccuracyRate (old, new): " << _detector_params->polygonalApproxAccuracyRate << ", " << i->value.toNumber() << std::endl;
        _detector_params->polygonalApproxAccuracyRate = i->value.toNumber();
      }
#if CHECK_OPENCV_VERSION(4, 6, 0)
      else if ("minMarkerLengthRatioOriginalImg" == std::string(i->key) )
      {
        // std::cout << "minMarkerLengthRatioOriginalImg (old, new): " << _detector_params->minMarkerLengthRatioOriginalImg << ", " << i->value.toNumber() << std::endl;
        _detector_params->minMarkerLengthRatioOriginalImg = i->value.toNumber();
      }
      else if ("minSideLengthCanonicalImg" == std::string(i->key))
      {
        // std::cout << "minSideLengthCanonicalImg (old, new): " << _detector_params->minSideLengthCanonicalImg << ", " << i->value.toNumber() << std::endl;
        _detector_params->minSideLengthCanonicalImg = i->value.toNumber();
      }
      else if ("useAruco3Detection" == std::string(i->key))
      {
        bool json_tf = false;
        if (i->value.getTag() == JSON_TRUE)
          json_tf = true;
        // std::cout << "useAruco3Detection (old, new): " << _detector_params->useAruco3Detection << ", " << json_tf << std::endl;
        _detector_params->useAruco3Detection = json_tf;
      }
#endif
      else if ("markerIds" == std::string(i->key) && i->value.getTag() == JSON_ARRAY)
      {
        // markerIds 用于筛选需要参与位姿估计的码；-1 表示不过滤，接受所有检测结果。
        int jcnt = 0;
        for (auto j : i->value)
        {
          if (jcnt == 0 && j->value.toNumber() == -1)
          {
            _ids_need.push_back(-1);
            break;
          }
          else
          {
            _ids_need.push_back(j->value.toNumber());
          }
        }
      }
      else if ("markerLengths" == std::string(i->key) && i->value.getTag() == JSON_ARRAY)
      {
        // markerLengths 与 markerIds 一一对应，决定 estimatePoseSingleMarkers() 的尺度单位。
        for (auto j : i->value)
        {
          if (_ids_need.size() > 0 && _ids_need[0] == -1)
          {
            _lengths_need.push_back(j->value.toNumber());
            break;
          }
          else
          {
            _lengths_need.push_back(j->value.toNumber());
          }
        }
      }
    }

    if (_ids_need.size() == 0)
      _ids_need.push_back(-1);
    // 若指定了若干个 markerId，则每个 markerId 都必须有对应的实际边长。
    if (_lengths_need.size() != _ids_need.size())
    {
      throw std::runtime_error("Parameter markerIds.length != markerLengths.length!");
    }

    // for (int id : _ids_need)
    //   std::cout << "_ids_need: " << id << std::endl;
    // for (double l : _lengths_need)
    //   std::cout << "_lengths_need: " << l << std::endl;
  }

  // 执行 ArUco 检测、位姿估计，并封装为内部目标列表。
  void ArucoDetector::detect(const cv::Mat &img_, TargetsInFrame &tgts_)
  {
    if (!_params_loaded)
    {
      // 首帧时完成参数解析，避免节点启动阶段做过多初始化。
      this->_load();
      _params_loaded = true;
    }
    if (img_.cols != this->image_width_ || img_.rows != this->image_height_)
    {
      char msg[256];
      sprintf(msg, "Calib camera SIZE(%d) != Input image SIZE(%d)!", this->image_width_, img_.cols);
      throw std::runtime_error(msg);
    }
    if (this->_dictionary == nullptr)
    {
      // 根据 dictionaryId 构造 OpenCV 预定义字典。
      this->_dictionary = new cv::aruco::Dictionary;
      *(this->_dictionary) = cv::aruco::getPredefinedDictionary(_dictionary_id);
    }

    std::vector<int> ids, ids_final;
    std::vector<std::vector<cv::Point2f>> corners, corners_final, rejected;
    std::vector<cv::Vec3d> rvecs, tvecs;

    // 第一步：从图像中找出所有候选 marker，返回角点和 ID。
    cv::aruco::detectMarkers(img_, this->_dictionary, corners, ids, _detector_params, rejected);

    if (ids.size() > 0)
    {
      if (_ids_need[0] == -1)
      {
        // 不做 ID 过滤时，所有 marker 共用同一个 markerLength。
        cv::aruco::estimatePoseSingleMarkers(corners, _lengths_need[0], this->camera_matrix, this->distortion, rvecs, tvecs);
        ids_final = ids;
        corners_final = corners;
      }
      else
      {
        // 若配置了特定 ID，则按 ID 逐个匹配长度，再分别做位姿估计。
        for (int i = 0; i < _ids_need.size(); i++)
        {
          int id_need = _ids_need[i];
          double length_need = _lengths_need[i];
          std::vector<cv::Vec3d> t_rvecs, t_tvecs;
          std::vector<std::vector<cv::Point2f>> t_corners;
          for (int j = 0; j < ids.size(); j++)
          {
            if (ids[j] == id_need)
            {
              t_corners.push_back(corners[j]);
              ids_final.push_back(ids[j]);
              corners_final.push_back(corners[j]);
            }
          }
          if (t_corners.size() > 0)
          {
            cv::aruco::estimatePoseSingleMarkers(t_corners, length_need, this->camera_matrix, this->distortion, t_rvecs, t_tvecs);
            for (auto t_rvec : t_rvecs)
              rvecs.push_back(t_rvec);
            for (auto t_tvec : t_tvecs)
              tvecs.push_back(t_tvec);
          }
        }
      }
    }

    // aruco::drawDetectedMarkers(img_, corners_final, ids_final);
    tgts_.setSize(img_.cols, img_.rows);

    // tgts_.fov_x = this->fov_x;
    // tgts_.fov_y = this->fov_y;
    tgts_.setFOV(this->fov_x, this->fov_y);
    auto t1 = std::chrono::system_clock::now();
    tgts_.setFPS(1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(t1 - this->_t0).count());
    this->_t0 = std::chrono::system_clock::now();
    tgts_.setTimeNow();

    if (ids_final.size() > 0)
    {
      // 将 OpenCV 输出封装为项目内部统一的 Target 结构，供上层节点使用。
      for (int i = 0; i < ids_final.size(); i++)
      {
        Target tgt;
        tgt.setAruco(ids_final[i], corners_final[i], rvecs[i], tvecs[i], tgts_.width, tgts_.height, this->camera_matrix);
        tgts_.targets.push_back(tgt);
      }
    }

    tgts_.type = MissionType::ARUCO_DET;
  }

  // 从相机标定 yaml 文件加载内参、畸变和图像尺寸。
  void ArucoDetector::loadCameraParams(std::string yaml_fn_)
  {
    // 从标定文件读取相机内参、畸变参数和图像尺寸。
    cv::FileStorage fs(yaml_fn_, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
      throw std::runtime_error("Camera calib file NOT exist!");
    }
    fs["camera_matrix"] >> this->camera_matrix;
    fs["distortion_coefficients"] >> this->distortion;
    fs["image_width"] >> this->image_width_;
    fs["image_height"] >> this->image_height_;

    if (this->camera_matrix.rows != 3 || this->camera_matrix.cols != 3 ||
        this->distortion.rows != 1 || this->distortion.cols != 5 ||
        this->image_width_ == 0 || this->image_height_ == 0)
    {
      throw std::runtime_error("Camera parameters reading ERROR!");
    }
    // 由焦距和图像尺寸计算视场角，供上层输出诊断信息使用。
    this->fov_x = 2 * atan(this->image_width_ / 2. / this->camera_matrix.at<double>(0, 0)) * RAD2DEG;
    this->fov_y = 2 * atan(this->image_height_ / 2. / this->camera_matrix.at<double>(1, 1)) * RAD2DEG;
  }

  // 从内存中的 CameraInfo 结构加载相机模型参数。
  void ArucoDetector::loadCameraParams(const CameraInfo& camera_info)
  {
    // 也支持直接从内存中的 CameraInfo 结构加载相机模型。
    this->camera_matrix = camera_info.camera_matrix;
    this->distortion = camera_info.distCoeffs;
    this->image_width_ = camera_info.width;
    this->image_height_ = camera_info.height;

    if (this->camera_matrix.rows != 3 || this->camera_matrix.cols != 3 ||
        this->distortion.rows != 1 || this->distortion.cols != 5 ||
        this->image_width_ == 0 || this->image_height_ == 0)
    {
      throw std::runtime_error("Camera parameters reading ERROR!");
    }
    this->fov_x = 2 * atan(this->image_width_ / 2. / this->camera_matrix.at<double>(0, 0)) * RAD2DEG;
    this->fov_y = 2 * atan(this->image_height_ / 2. / this->camera_matrix.at<double>(1, 1)) * RAD2DEG;
  }
  
  // 记录算法配置文件路径，供首次检测时解析。
  void ArucoDetector::loadAlgorithmParams(std::string json_fn_)
  {
    // 这里只记录配置文件路径，真正解析在首次 detect() 时执行。
    this->alg_params_fn = json_fn_;
  }
}
