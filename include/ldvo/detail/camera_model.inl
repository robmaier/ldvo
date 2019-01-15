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

#include <ldvo/camera_model.hpp>

#include <iostream>
#include <fstream>


namespace ldvo
{

    inline CameraModel::CameraModel() :
        width_(640),
        height_(480),
        K_(Eigen::Matrix3f::Identity())
    {
        K_(0, 0) = 525.0f;
        K_(1, 1) = 525.0f;
        K_(0, 2) = 319.5f;
        K_(1, 2) = 239.5f;
    }


    inline CameraModel::CameraModel(int width, int height, const Eigen::Matrix3f &K) :
        width_(width),
        height_(height),
        K_(K)
    {
    }


    inline CameraModel::~CameraModel()
    {
    }


    inline int CameraModel::width() const
    {
        return width_;
    }

    inline int CameraModel::height() const
    {
        return height_;
    }


    inline const Eigen::Matrix3f CameraModel::intrinsics() const
    {
        return K_;
    }


    inline float CameraModel::fx() const
    {
        return K_(0, 0);
    }


    inline float CameraModel::fy() const
    {
        return K_(1, 1);
    }


    inline float CameraModel::cx() const
    {
        return K_(0, 2);
    }


    inline float CameraModel::cy() const
    {
        return K_(1, 2);
    }


    inline Eigen::Vector2f CameraModel::project(const Eigen::Vector3f &pt3) const
    {

        const Eigen::Vector3f pt2_h = K_ * pt3;
        const float pt_z_inv = 1.0f / pt2_h[2];
        Eigen::Vector2f pt2;
        pt2[0] = pt2_h[0] * pt_z_inv;
        pt2[1] = pt2_h[1] * pt_z_inv;
        return pt2;
    }


    inline Eigen::Vector3f CameraModel::backProject(const int x, const int y, const float depth) const
    {
        const Eigen::Vector2f pt2(static_cast<float>(x), static_cast<float>(y));
        return backProject(pt2, depth);
    }


    inline Eigen::Vector3f CameraModel::backProject(const Eigen::Vector2f &pt2, const float depth) const
    {
        // backproject point into 3d using its depth
        const float x0 = (pt2[0] - K_(0, 2)) / K_(0, 0);
        const float y0 = (pt2[1] - K_(1, 2)) / K_(1, 1);
        return Eigen::Vector3f(x0 * depth, y0 * depth, depth);
    }


    void CameraModel::print() const
    {
        std::cout << "Camera model:" << std::endl;
        std::cout << "   image size: " << width_ << "x" << height_ << std::endl;
        std::cout << "   intrinsics: " << std::endl << K_ << std::endl;
    }


    bool CameraModel::load(const std::string &filename)
    {
        if (filename.empty())
            return false;
        std::ifstream file(filename.c_str());
        if (!file.is_open())
            return false;

        //camera width and height
        file >> width_ >> height_;

        //camera intrinsics
        K_ = Eigen::Matrix3f::Identity();
        float fVal = 0;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                file >> fVal;
                K_(i, j) = fVal;
            }
        }
        file.close();

        return true;
    }

} // namespace ldvo
