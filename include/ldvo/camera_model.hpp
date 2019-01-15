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

#include <string>
#include <Eigen/Dense>


namespace ldvo
{

    /**
     * @brief   Pinhole camera model class for storing camera intrinsics and
     *          their respective geometric functions for (back)-projection.
     * @author  Robert Maier
     */
    class CameraModel
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief   Constructor for creating a pinhole camera model with
         *          default parameters, i.e.:
         *          width=640, height=480,
         *          fx=525.0, fy=525.0, cx=319.5f, cy=239.5f.
         * @param   cfg     Configuration struct to configure
         *                  the dataset.
         */
        CameraModel();

        /**
         * @brief   Constructor for creating a pinhole camera model with
         *          specified image dimensions and 3x3 K matrix.
         * @param   width   Camera/image width.
         * @param   height  Camera/image height.
         * @param   K       3x3 camera intrinsics matrix.
         */
        CameraModel(int width, int height, const Eigen::Matrix3f &K);

        /// Destructor.
        ~CameraModel();

        /// Get width of the camera/image.
        int width() const;

        /// Get height of the camera/image.
        int height() const;

        /// Get the 3x3 camera intrinsics matrix.
        const Eigen::Matrix3f intrinsics() const;

        /// Get focal length w.r.t. x-direction.
        float fx() const;

        /// Get focal length w.r.t. y-direction.
        float fy() const;

        /// Get center pixel offset in x-direction.
        float cx() const;

        /// Get center pixel offset in y-direction.
        float cy() const;

        /**
         * @brief   Project a 3D point to 2D image coordinates using the
         *          internal camera model parameters.
         * @param   pt3     3D point to be projected.
         * @return  Projected 2D image point coordinates.
         */
        Eigen::Vector2f project(const Eigen::Vector3f &pt3) const;

        /**
         * @brief   Backproject a 2D image to 3D coordinates using
         *          its depth with the internal camera model parameters.
         * @param   x       2D image point x-coordinate.
         * @param   y       2D image point y-coordinate.
         * @param   depth   Depth value at the pixel (x,y).
         * @return  Back-projected 3D point coordinates.
         */
        Eigen::Vector3f backProject(const int x, const int y, const float depth) const;

        /**
         * @brief   Backproject a 2D image point to 3D coordinates using
         *          its depth with the internal camera model parameters.
         * @param   pt2     2D image point coordinates.
         * @param   depth   Depth value at the image point pt2.
         * @return  Back-projected 3D point coordinates.
         */
        Eigen::Vector3f backProject(const Eigen::Vector2f &pt2, const float depth) const;

        /// Print out the camera model parameters
        void print() const;

        /**
         * @brief   Load camera intrinsics from a file on disk.
         * @param   filename    Filename of camera intrinsics file to load.
         * @return  True if loading successful, otherwise False.
         */
        bool load(const std::string &filename);

    protected:
        int width_;
        int height_;
        Eigen::Matrix3f K_;
    };

} // namespace ldvo

#include <ldvo/detail/camera_model.inl>
