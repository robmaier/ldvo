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

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <ldvo/camera_model.hpp>
#include <ldvo/frame.hpp>


namespace ldvo
{


    /**
     * @brief   Base class for various pyramids that
     *          include downsampling functionality.
     * @author  Robert Maier
     */
    class Pyramid
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief   Constructor that initializes an empty RGB-D frame with
         *          specified width and height. Intensity image and depth
         *          map must have the same dimensions and must be registered.
         * @param   num_levels    Number of pyramid levels.
         */
        Pyramid(int num_levels);

        /// Destructor
        virtual ~Pyramid();

        /// Get number of pyramid levels.
        int levels() const;

    protected:
        // disable copy constructor
        Pyramid(const Pyramid& other) = delete;
        // disable move constructor
        Pyramid(const Pyramid&& other) = delete;
        // disable copy assignment operator
        Pyramid& operator=(const Pyramid& other) = delete;
        // disable move assignment operator
        Pyramid& operator=(const Pyramid&& other) = delete;

        int num_pyramid_levels_;
    };


    /**
     * @brief   Class for a camera model (camera intrinsics) pyramid.
     * @author  Robert Maier
     */
    class CameraModelPyramid : public Pyramid
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief   Constructor that creates a CameraModel pyramid.
         * @param   cam           Camera model to downsample.
         * @param   num_levels    Number of pyramid levels.
         */
        CameraModelPyramid(const CameraModel &cam, int num_levels);

        /**
         * @brief   Get the downsampled CameraModel for a level.
         * @param   level     Pyramid level to retrieve.
         * @return  Downsampled CameraModel.
         */
        const CameraModel& at(const int level) const;

    private:
        std::vector<CameraModel> pyramid_;
    };


    /**
     * @brief   Class for an RGB-D frame pyramid.
     * @author  Robert Maier
     */
    class FramePyramid : public Pyramid
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief   Constructor that creates a frame pyramid.
         * @param   w             Frame width.
         * @param   h             Frame height.
         * @param   num_levels    Number of pyramid levels.
         */
        FramePyramid(int w, int h, int num_levels);

        /**
         * @brief   Get the downsampled RGB-D frame for a level.
         * @param   level     Pyramid level to retrieve.
         * @return  Const reference to the downsampled RGB-D frame.
         */
        const Frame& at(const int level) const;

        /**
         * @brief   Get the downsampled RGB-D frame for a level.
         * @param   level     RGB-D frame to retrieve.
         * @return  Reference to the downsampled RGB-D frame.
         */
        Frame& at(const int level);

        /**
         * @brief   Fill the RGB-D frame pyramid with a frame.
         * @param   frame     Input RGB-D frame that is downsampled.
         */
        void fill(const Frame &frame);

    private:

        /**
         * @brief   Fill the RGB-D frame pyramid with frame data.
         * @param   gray        Input intensity image.
         * @param   depth       Input depth image.
         * @param   time_color  Timestamp of input intensity.
         * @param   time_depth  Timestamp of input depth.
         */
        void fill(const cv::Mat &gray, const cv::Mat &depth,
                  double time_color = 0.0, double time_depth = 0.0);

        /**
         * @brief   Downsamples an input (intensity/grayscale) image.
         * @param   img         Input intensity image.
         * @param   img_out     Downsampled intensity image (of half width/height).
         */
        void downsample(const cv::Mat &img, cv::Mat &img_out) const;

        /**
         * @brief   Downsamples an input depth image.
         * @param   depth       Input depth image.
         * @param   depth_out   Downsampled depth image (of half width/height).
         */
        void downsampleDepth(const cv::Mat &depth, cv::Mat &depth_out) const;

        std::vector<Frame> pyramid_;
    };


    /**
     * @brief   Class for an image pyramid.
     * @author  Robert Maier
     */
    class ImagePyramid : public Pyramid
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief   Constructor that creates an image pyramid.
         * @param   w             Image width.
         * @param   h             Image height.
         * @param   num_levels    Number of pyramid levels.
         */
        ImagePyramid(int w, int h, int num_levels);

        /**
         * @brief   Get the downsampled image for a level.
         * @param   level     Pyramid level to retrieve.
         * @return  Const reference to the downsampled image.
         */
        const cv::Mat& at(const int level) const;

        /**
         * @brief   Get the downsampled image for a level.
         * @param   level     Pyramid level to retrieve.
         * @return  Reference to the downsampled image.
         */
        cv::Mat& at(const int level);

    private:
        std::vector<cv::Mat> pyramid_;
    };

} // namespace ldvo

#include <ldvo/detail/pyramid.inl>
