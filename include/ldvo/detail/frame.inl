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

#include <ldvo/frame.hpp>

#include <iostream>
#include <memory>

#include <ldvo/math/util.hpp>

namespace ldvo
{

    inline Frame::Frame(int width, int height) :
        width_(width),
        height_(height),
        gray_(height, width, CV_32FC1),
        depth_(height, width, CV_32FC1),
        time_color_(0.0),
        time_depth_(0.0),
        gradient_x_(height, width, CV_32FC1),
        gradient_y_(height, width, CV_32FC1)
    {
    }


    inline Frame::Frame(const Frame&& other) :
        width_(other.width_),
        height_(other.height_),
        gray_(other.gray_),
        depth_(other.depth_),
        time_color_(other.time_color_),
        time_depth_(other.time_depth_),
        gradient_x_(other.gradient_x_),
        gradient_y_(other.gradient_y_)
    {
    }


    inline Frame::~Frame()
    {
    }


    inline int Frame::width() const
    {
        return width_;
    }

    inline int Frame::height() const
    {
        return height_;
    }


    inline const cv::Mat& Frame::gray() const
    {
        return gray_;
    }


    inline cv::Mat& Frame::gray()
    {
        return gray_;
    }


    inline const cv::Mat& Frame::depth() const
    {
        return depth_;
    }

    inline cv::Mat& Frame::depth()
    {
        return depth_;
    }


    inline double Frame::timeColor() const
    {
        return time_color_;
    }


    inline void Frame::setTimeColor(double t)
    {
        time_color_ = t;
    }


    inline double Frame::timeDepth() const
    {
        return time_depth_;
    }


    inline void Frame::setTimeDepth(double t)
    {
        time_depth_ = t;
    }


    inline const cv::Mat& Frame::gradientX() const
    {
        return gradient_x_;
    }


    inline const cv::Mat& Frame::gradientY() const
    {
        return gradient_y_;
    }


    inline void Frame::fill(const cv::Mat &gray, const cv::Mat &depth,
                            double time_color, double time_depth)
    {
        if (gray.empty() || depth.empty() ||
                gray.cols != width_ || gray.rows != height_ ||
                depth.cols != width_ || depth.rows != height_)
            return;

        // fill internal data from input
        size_t byte_size = static_cast<size_t>(width_) * static_cast<size_t>(height_) * sizeof(float);
        memcpy(gray_.data, gray.data, byte_size);
        memcpy(depth_.data, depth.data, byte_size);
        time_color_ = time_color;
        time_depth_ = time_depth;
        // compute gradient of intensity
        computeGradients();
    }


    inline void Frame::computeGradients()
    {
        // compute gradient of intensity
        math::util::computeGradient(gray_, gradient_x_, 0);
        math::util::computeGradient(gray_, gradient_y_, 1);
    }

} // namespace ldvo
