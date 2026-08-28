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

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <unordered_map>

#include "common/common.h"
#include "common/ellipse.h"
#include "utils/gason.h"
#include "common/box.h"
#include "common/target.h"
#include "common/targets_in_frame.h"

#include "inference_backend/opencv/aruco/aruco_detector.h"

namespace sunray_detection
{
    class EllipseDetector
    {
    public:
        // Constructor and Destructor
        EllipseDetector(void);
        ~EllipseDetector(void);

        // Detect the ellipses in the gray image
        void Detect(cv::Mat3b &I, std::vector<Ellipse> &ellipses);
        void Detect(cv::Mat &I, std::vector<Ellipse> &ellipses);
        void loadCameraParams(std::string yaml_fn_);
        void loadAlgorithmParams(std::string json_fn_);
        void loadCameraParams(const CameraInfo &camera_info);

        cv::Mat camera_matrix;
        cv::Mat distortion;
        int image_width;
        int image_height;
        double fov_x;
        double fov_y;
        std::string alg_params_fn;
        // Draw the first iTopN ellipses on output
        void DrawDetectedEllipses(cv::Mat &output, std::vector<Ellipse> &ellipses, int iTopN = 0, int thickness = 2);

        // Set the parameters of the detector
        void SetParameters(cv::Size szPreProcessingGaussKernelSize,
                           double dPreProcessingGaussSigma,
                           float fThPosition,
                           float fMaxCenterDistance,
                           int iMinEdgeLength,
                           float fMinOrientedRectSide,
                           float fDistanceToEllipseContour,
                           float fMinScore,
                           float fMinReliability,
                           int iNs,
                           double dPercentNe,
                           float fT_CNC,
                           float fT_TCN_L,
                           float fT_TCN_P,
                           float fThre_r);

        void SetMCD(float fMaxCenterDistance);

        // Return the execution time
        double GetExecTime()
        {
            double time_all(0);
            for (size_t i = 0; i < times_.size(); i++)
                time_all += times_[i];
            return time_all;
        }
        std::vector<double> GetTimes() { return times_; }

        float countOfFindEllipse_;
        float countOfGetFastCenter_;

        void _load();

        bool _params_loaded;
        float _max_center_distance_ratio;
        double _radius_in_meter = 0.5;

    private:
        // keys for hash table
        static const uint16_t PAIR_12 = 0x00;
        static const uint16_t PAIR_23 = 0x01;
        static const uint16_t PAIR_34 = 0x02;
        static const uint16_t PAIR_14 = 0x03;

        // generate keys from pair and indicse
        uint inline GenerateKey(uint8_t pair, uint16_t u, uint16_t v);

        void PreProcessing(cv::Mat1b &I, cv::Mat1b &arcs8);
        void RemoveStraightLine(std::vector<std::vector<cv::Point>> &segments, std::vector<std::vector<cv::Point>> &segments_update, int id = 0);
        void PreProcessing(cv::Mat1b &I, cv::Mat1b &DP, cv::Mat1b &DN);

        void ClusterEllipses(std::vector<Ellipse> &ellipses);

        // int FindMaxK(const std::vector<int>& v) const;
        // int FindMaxN(const std::vector<int>& v) const;
        // int FindMaxA(const std::vector<int>& v) const;

        int FindMaxK(const int *v) const;
        int FindMaxN(const int *v) const;
        int FindMaxA(const int *v) const;

        float GetMedianSlope(std::vector<cv::Point2f> &med, cv::Point2f &M, std::vector<float> &slopes);
        void GetFastCenter(std::vector<cv::Point> &e1, std::vector<cv::Point> &e2, EllipseData &data);
        float GetMinAnglePI(float alpha, float beta);

        void DetectEdges13(cv::Mat1b &DP, std::vector<std::vector<cv::Point>> &points_1, std::vector<std::vector<cv::Point>> &points_3);
        void DetectEdges24(cv::Mat1b &DN, std::vector<std::vector<cv::Point>> &points_2, std::vector<std::vector<cv::Point>> &points_4);

        void ArcsCheck1234(std::vector<std::vector<cv::Point>> &points_1, std::vector<std::vector<cv::Point>> &points_2, std::vector<std::vector<cv::Point>> &points_3, std::vector<std::vector<cv::Point>> &points_4);

        void FindEllipses(cv::Point2f &center,
                          std::vector<cv::Point> &edge_i,
                          std::vector<cv::Point> &edge_j,
                          std::vector<cv::Point> &edge_k,
                          EllipseData &data_ij,
                          EllipseData &data_ik,
                          Ellipse &ell);

        cv::Point2f GetCenterCoordinates(EllipseData &data_ij, EllipseData &data_ik);

        void Triplets124(std::vector<std::vector<cv::Point>> &pi,
                         std::vector<std::vector<cv::Point>> &pj,
                         std::vector<std::vector<cv::Point>> &pk,
                         std::unordered_map<uint, EllipseData> &data,
                         std::vector<Ellipse> &ellipses);

        void Triplets231(std::vector<std::vector<cv::Point>> &pi,
                         std::vector<std::vector<cv::Point>> &pj,
                         std::vector<std::vector<cv::Point>> &pk,
                         std::unordered_map<uint, EllipseData> &data,
                         std::vector<Ellipse> &ellipses);

        void Triplets342(std::vector<std::vector<cv::Point>> &pi,
                         std::vector<std::vector<cv::Point>> &pj,
                         std::vector<std::vector<cv::Point>> &pk,
                         std::unordered_map<uint, EllipseData> &data,
                         std::vector<Ellipse> &ellipses);

        void Triplets413(std::vector<std::vector<cv::Point>> &pi,
                         std::vector<std::vector<cv::Point>> &pj,
                         std::vector<std::vector<cv::Point>> &pk,
                         std::unordered_map<uint, EllipseData> &data,
                         std::vector<Ellipse> &ellipses);

        void Tic(unsigned idx = 0) // start
        {
            while (idx >= timesSign_.size())
            {
                timesSign_.push_back(0);
                times_.push_back(.0);
            }
            timesSign_[idx] = 0;
            timesSign_[idx]++;
            times_[idx] = (double)cv::getTickCount();
        }

        void Toc(unsigned idx = 0, std::string step = "") // stop
        {
            assert(timesSign_[idx] == 1);
            timesSign_[idx]++;
            times_[idx] = ((double)cv::getTickCount() - times_[idx]) * 1000. / cv::getTickFrequency();
            // #ifdef DEBUG_SPEED
            std::cout << "Cost time: " << times_[idx] << " ms [" << idx << "] - " << step << std::endl;
            if (idx == times_.size() - 1)
                std::cout << "Totally cost time: " << this->GetExecTime() << " ms" << std::endl;
            // #endif
        }

    private:
        std::vector<int> timesSign_;

    private:
        // Parameters
        // Preprocessing - Gaussian filter. See Sect [] in the paper
        cv::Size szPreProcessingGaussKernel_; // size of the Gaussian filter in preprocessing step
        double dPreProcessingGaussSigma_;     // sigma of the Gaussian filter in the preprocessing step

        // Selection strategy - Step 1 - Discard noisy or straight arcs. See Sect [] in the paper
        int iMinEdgeLength_;         // minimum edge size
        float fMinOrientedRectSide_; // minumum size of the oriented bounding box containing the arc
        float fMaxRectAxesRatio_;    // maximum aspect ratio of the oriented bounding box containing the arc

        // Selection strategy - Step 2 - Remove according to mutual convexities. See Sect [] in the paper
        float fThrArcPosition_;

        // Selection Strategy - Step 3 - Number of points considered for slope estimation when estimating the center. See Sect [] in the paper
        unsigned uNs_; // Find at most Ns parallel chords.

        // Selection strategy - Step 3 - Discard pairs of arcs if their estimated center is not close enough. See Sect [] in the paper
        float fMaxCenterDistance_;  // maximum distance in pixel between 2 center points
        float fMaxCenterDistance2_; // _fMaxCenterDistance * _fMaxCenterDistance

        // Validation - Points within a this threshold are considered to lie on the ellipse contour. See Sect [] in the paper
        float fDistanceToEllipseContour_; // maximum distance between a point and the contour. See equation [] in the paper

        // Validation - Assign a score. See Sect [] in the paper
        float fMinScore_;       // minimum score to confirm a detection
        float fMinReliability_; // minimum auxiliary score to confirm a detection

        double dPercentNe_;

        float fT_CNC_;
        float fT_TCN_L_; // filter lines
        float fT_TCN_P_;
        float fThre_r_;

        // auxiliary variables
        cv::Size szIm_; // input image size

        std::vector<double> times_; // times_ is a vector containing the execution time of each step.

        int ACC_N_SIZE; // size of accumulator N = B/A
        int ACC_R_SIZE; // size of accumulator R = rho = atan(K)
        int ACC_A_SIZE; // size of accumulator A

        int *accN; // pointer to accumulator N
        int *accR; // pointer to accumulator R
        int *accA; // pointer to accumulator A

        cv::Mat1f EO_;

        std::vector<std::vector<cv::Point>> points_1, points_2, points_3, points_4; // vector of points, one for each convexity class
    };

}
