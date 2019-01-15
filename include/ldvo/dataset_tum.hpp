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

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include <ldvo/camera_model.hpp>
#include <ldvo/frame.hpp>


namespace ldvo
{

    /**
     * @brief   Class for loading a dataset (i.e. RGB-D frames)
     *          in the TUM RGB-D benchmark format.
     * @author  Robert Maier
     */
    class DatasetTUM
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        /**
         * @brief   Struct for configuring the dataset and
         *          RGB-D frame loading.
         * @author  Robert Maier
         */
        struct Config
        {
        public:
            std::string dataset_folder;
            std::string intrinsics_file = "";
            size_t max_num_frames = 0;
            float max_depth = 0.0f;

            /// Print out the dataset loading configuration.
            void print() const;
        };


        /**
         * @brief   Constructor that creates a dataset from a
         *          configuration struct.
         * @param   cfg     Configuration struct to configure
         *                  the dataset.
         */
        DatasetTUM(const DatasetTUM::Config &cfg);

        /// Destructor.
        ~DatasetTUM();

        /**
         * @brief   Initializes a dataset by loading it from
         *          TUM RGB-D benchmark format, i.e. loads the
         *          filenames of the RGB-D frame pairs and the
         *          camera intrinsics.
         * @return  True if successfully initialized, otherwise False.
         */
        bool init();

        /**
         * @brief   Get camera model used in the dataset.
         *          Note:   color and depth camera use the same camera
         *                  intrinsics, i.e. they are assumed to have the
         *                  same width and height and be pre-registered.
         * @return  Depth and color camera intrinsics.
         */
        const CameraModel& camera() const;

        /// Get number of frames in the dataset.
        size_t numFrames() const;

        /**
         * @brief   Load an RGB-D frame from the dataset.
         * @param   id      Id of the RGB-D frame to load.
         * @return  Shared pointer to the loaded RGB-D frame.
         */
        std::shared_ptr<Frame> loadFrame(const size_t id) const;

    private:

        /**
         * @brief   Load an associations file with RGB-D frame pairs.
         * @param   dataset     Dataset foldername (full path).
         * @param   assoc_file  Associations filename (full path).
         * @return  True if associations file could be loaded successfully.
         */
        bool loadAssoc(const std::string &dataset, const std::string &assoc_file);

        /**
         * @brief   Load a color image from disk.
         * @param   id      Id of the RGB-D frame to load.
         * @return  Loaded color image.
         */
        cv::Mat loadColor(const size_t id) const;

        /**
         * @brief   Load an intensity/grayscale image from disk.
         * @param   id      Id of the RGB-D frame to load.
         * @return  Loaded intensity/grayscale image.
         */
        cv::Mat loadGray(const size_t id) const;

        /**
         * @brief   Load a depth image from disk.
         * @param   id      Id of the RGB-D frame to load.
         * @return  Loaded depth image.
         */
        cv::Mat loadDepth(const size_t id) const;

        /**
         * @brief   Get the color timestamp of a frame.
         * @param   id      Id of the RGB-D frame to load.
         * @return  Color image timestamp.
         */
        double timeColor(const size_t id) const;

        /**
         * @brief   Get the depth timestamp of a frame.
         * @param   id      Id of the RGB-D frame to load.
         * @return  Depthimage timestamp.
         */
        double timeDepth(const size_t id) const;

        /**
         * @brief   Theshold a depth image, i.e. set all values smaller
         *          than depth_min and greater than depth_max to zero.
         * @param[in,out]   depth       Depth map to be thresholded.
         * @param[in]       depth_min   Minimum depth value.
         * @param[in]       depth_max   Maximum depth value.
         */
        void threshold(cv::Mat &depth, float depth_min, float depth_max) const;

        Config cfg_;
        CameraModel cam_;
        std::vector<std::string> files_color_;
        std::vector<std::string> files_depth_;
        std::vector<double> timestamps_depth_;
        std::vector<double> timestamps_color_;
    };

} // namespace ldvo

#include <ldvo/detail/dataset_tum.inl>
