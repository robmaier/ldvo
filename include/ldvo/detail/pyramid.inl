/**
* This file is part of LDVO.
*
* Copyright 2019 Robert Maier, Technical University of Munich.
* For more information see <https://github.com/robmaier/ldvo>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* LDVO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LDVO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LDVO. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <ldvo/pyramid.hpp>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <opencv2/core/core.hpp>


namespace ldvo
{

    inline Pyramid::Pyramid(int num_levels):
        num_pyramid_levels_(num_levels)
    {
    }


    inline Pyramid::~Pyramid()
    {
    }


    inline int Pyramid::levels() const
    {
        return num_pyramid_levels_;
    }


    inline CameraModelPyramid::CameraModelPyramid(const CameraModel& cam,
                                                  int num_levels) :
        Pyramid(num_levels)
    {
        // create pyramid
        pyramid_.push_back(cam);

        // pyramid downsampling
        for (int lvl = 1; lvl < num_pyramid_levels_; ++lvl)
        {
            size_t lvlPrev = static_cast<size_t>(lvl-1);
            // downsample camera matrix
            Eigen::Matrix3f k_down = pyramid_[lvlPrev].intrinsics();
            k_down(0, 2) += 0.5f;
            k_down(1, 2) += 0.5f;
            k_down.topLeftCorner(2, 3) = k_down.topLeftCorner(2, 3) * 0.5f;
            k_down(0, 2) -= 0.5f;
            k_down(1, 2) -= 0.5f;

            // downsample pyramid level size
            int w_down = pyramid_[lvlPrev].width() / 2;
            int h_down = pyramid_[lvlPrev].height() / 2;

            // store downsampled camera
            CameraModel camDown(w_down, h_down, k_down);
            pyramid_.push_back(camDown);
        }
    }


    inline const CameraModel& CameraModelPyramid::at(const int level) const
    {
        return pyramid_.at(static_cast<size_t>(level));
    }


    inline FramePyramid::FramePyramid(int w, int h, int num_levels):
        Pyramid(num_levels)
    {
        // create pyramid

        // directly move created frame into pyramid
        pyramid_.push_back(std::move(Frame(w, h)));

        // pyramid downsampling
        for (int lvl = 1; lvl < num_pyramid_levels_; ++lvl)
        {
            size_t lvlPrev = static_cast<size_t>(lvl-1);
            const Frame& f = pyramid_[lvlPrev];
            // directly move created frame into pyramid
            pyramid_.push_back(std::move(Frame(f.width() / 2, f.height() / 2)));
        }
    }


    inline const Frame& FramePyramid::at(const int level) const
    {
        return pyramid_.at(static_cast<size_t>(level));
    }


    inline Frame& FramePyramid::at(const int level)
    {
        return pyramid_.at(static_cast<size_t>(level));
    }


    inline void FramePyramid::fill(const Frame &frame)
    {
        // init pyramid
        fill(frame.gray(), frame.depth(), frame.timeColor(), frame.timeDepth());
    }


    inline void FramePyramid::fill(const cv::Mat &gray, const cv::Mat &depth,
                                   double time_color, double time_depth)
    {
        if (pyramid_.size() != static_cast<size_t>(num_pyramid_levels_))
            return;

        // init pyramid
        pyramid_[0].fill(gray, depth, time_color, time_depth);

        // pyramid downsampling
        for (size_t lvl = 1; lvl < static_cast<size_t>(num_pyramid_levels_); ++lvl)
        {
            size_t lvlPrev = static_cast<size_t>(lvl-1);
            downsample(pyramid_[lvlPrev].gray(), pyramid_[lvl].gray());
            downsampleDepth(pyramid_[lvlPrev].depth(), pyramid_[lvl].depth());
            pyramid_[lvl].setTimeColor(pyramid_[lvlPrev].timeColor());
            pyramid_[lvl].setTimeDepth(pyramid_[lvlPrev].timeDepth());
            pyramid_[lvl].computeGradients();
        }
    }


    inline void FramePyramid::downsample(const cv::Mat &img, cv::Mat &img_out) const
    {
        const float* ptr_img = reinterpret_cast<const float*>(img.data);
        int w = img.cols;
        int h = img.rows;
        int w_down = w / 2;
        int h_down = h / 2;
        if (img_out.rows != h_down || img_out.cols != w_down)
            return;
        img_out.setTo(0.0);

        float* ptr_img_down = reinterpret_cast<float*>(img_out.data);
        for (int y = 0; y < h_down; ++y)
        {
            for (int x = 0; x < w_down; ++x)
            {
                float avg = 0.0f;
                avg += ptr_img[2*y * w + 2*x] * 0.25f;
                avg += ptr_img[2*y * w + 2*x+1] * 0.25f;
                avg += ptr_img[(2*y+1) * w + 2*x] * 0.25f;
                avg += ptr_img[(2*y+1) * w + 2*x+1] * 0.25f;
                ptr_img_down[y*w_down + x] = avg;
            }
        }
    }


    inline void FramePyramid::downsampleDepth(const cv::Mat &depth, cv::Mat &depth_out) const
    {
        const float* ptr_depth = reinterpret_cast<const float*>(depth.data);
        int w = depth.cols;
        int h = depth.rows;
        int w_down = w / 2;
        int h_down = h / 2;
        if (depth_out.rows != h_down || depth_out.cols != w_down)
            return;
        depth_out.setTo(0.0);

        // downscaling by averaging the inverse depth
        float* ptr_depth_down = reinterpret_cast<float*>(depth_out.data);
        for (int y = 0; y < h_down; ++y)
        {
            for (int x = 0; x < w_down; ++x)
            {
                float d0 = ptr_depth[2*y * w + 2*x];
                float d1 = ptr_depth[2*y * w + 2*x+1];
                float d2 = ptr_depth[(2*y+1) * w + 2*x];
                float d3 = ptr_depth[(2*y+1) * w + 2*x+1];

                size_t cnt = 0;
                float avg = 0.0f;
                if (d0 != 0.0f)
                {
                    avg += 1.0f / d0;
                    ++cnt;
                }
                if (d1 != 0.0f)
                {
                    avg += 1.0f / d1;
                    ++cnt;
                }
                if (d2 != 0.0f)
                {
                    avg += 1.0f / d2;
                    ++cnt;
                }
                if (d3 != 0.0f)
                {
                    avg += 1.0f / d3;
                    ++cnt;
                }

                if (cnt > 0)
                {
                    float d_inv = avg / static_cast<float>(cnt);
                    if (d_inv != 0.0f)
                        ptr_depth_down[y*w_down + x] = 1.0f / d_inv;
                }
            }
        }
    }



    inline ImagePyramid::ImagePyramid(int w, int h, int num_levels):
        Pyramid(num_levels)
    {
        // create pyramid
        pyramid_.push_back(cv::Mat::zeros(h, w, CV_32FC1));

        // pyramid downsampling
        for (int lvl = 1; lvl < num_pyramid_levels_; ++lvl)
        {
            size_t lvlPrev = static_cast<size_t>(lvl-1);
            const cv::Mat& img = pyramid_[lvlPrev];
            int w_down = img.cols / 2;
            int h_down = img.rows / 2;
            pyramid_.push_back(cv::Mat::zeros(h_down, w_down, CV_32FC1));
        }
    }


    inline const cv::Mat& ImagePyramid::at(const int level) const
    {
        return pyramid_.at(static_cast<size_t>(level));
    }


    inline cv::Mat& ImagePyramid::at(const int level)
    {
        return pyramid_.at(static_cast<size_t>(level));
    }

} // namespace ldvo
