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

#include <ldvo/dataset_tum.hpp>

#include <iostream>
#include <iomanip>
#include <fstream>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ldvo/camera_model.hpp>


namespace ldvo
{

    inline void DatasetTUM::Config::print() const
    {
        std::cout << "Dataset config: " << std::endl;
        std::cout << "   folder: " << dataset_folder << std::endl;
        std::cout << "   camera intrinsics: " << intrinsics_file << std::endl;
        std::cout << "   max num frames: " << max_num_frames << std::endl;
        std::cout << "   depth threshold: " << max_depth << std::endl;
    }


    inline DatasetTUM::DatasetTUM(const DatasetTUM::Config &cfg) :
        cfg_(cfg),
        cam_()
    {
    }


    inline DatasetTUM::~DatasetTUM()
    {
    }


    inline bool DatasetTUM::init()
    {
        // load assoc file
        bool ok = loadAssoc(cfg_.dataset_folder, cfg_.dataset_folder + "/rgbd_assoc.txt");
        if (!ok)
            ok = loadAssoc(cfg_.dataset_folder, cfg_.dataset_folder + "/assoc.txt");
        if (!ok)
        {
            std::cerr << "could not load dataset!" << std::endl;
            std::cerr << "    missing rgbd_assoc.txt / assoc.txt in folder " << cfg_.dataset_folder << std::endl;
            return false;
        }

        // load camera intrinsics
        if (!cam_.load(cfg_.intrinsics_file))
        {
            std::cout << "warning: could not load camera intrinsics from file ..." << std::endl;
        }

        // check whether dataset contains files
        if (files_depth_.empty())
            return false;

        return true;
    }


    inline const CameraModel& DatasetTUM::camera() const
    {
        return cam_;
    }


    inline size_t DatasetTUM::numFrames() const
    {
        return files_depth_.size();
    }


    std::shared_ptr<Frame> DatasetTUM::loadFrame(const size_t id) const
    {
        cv::Mat gray_prev = loadGray(id);
        cv::Mat depth_prev = loadDepth(id);
        double time_color_prev = timeColor(id);
        double time_depth_prev = timeDepth(id);

        std::shared_ptr<Frame> f = std::make_shared<Frame>(cam_.width(), cam_.height());
        f->fill(gray_prev, depth_prev, time_color_prev, time_depth_prev);
        return f;
    }


    inline bool DatasetTUM::loadAssoc(const std::string &dataset, const std::string &assoc_file)
    {
        files_depth_.clear();
        files_color_.clear();
        timestamps_depth_.clear();
        timestamps_color_.clear();

        if (dataset.empty() || assoc_file.empty())
            return false;

        //load transformations from CVPR RGBD datasets benchmark
        std::ifstream file;
        file.open(assoc_file.c_str());
        if (!file.is_open())
            return false;

        // first load all groundtruth timestamps and poses
        std::string line;
        while (std::getline(file, line))
        {
            if (line.empty() || line.compare(0, 1, "#") == 0)
                continue;
            std::istringstream iss(line);
            double timestamp_depth, timestamp_color;
            std::string file_depth, file_color;
            if (!(iss >> timestamp_color >> file_color >> timestamp_depth >> file_depth))
                break;

            files_depth_.push_back(dataset + "/" + file_depth);
            files_color_.push_back(dataset + "/" + file_color);
            timestamps_depth_.push_back(timestamp_depth);
            timestamps_color_.push_back(timestamp_color);

            if (cfg_.max_num_frames > 0 && files_depth_.size() >= cfg_.max_num_frames)
                break;
        }
        file.close();

        return true;
    }


    inline cv::Mat DatasetTUM::loadColor(const size_t id) const
    {
        if (id >= files_color_.size())
            return cv::Mat();
        cv::Mat color8 = cv::imread(files_color_[id]);
        // convert gray to float
        cv::Mat color;
        color8.convertTo(color, CV_32FC3, 1.0 / 255.0);
        return color;
    }


    inline cv::Mat DatasetTUM::loadGray(const size_t id) const
    {
        if (id >= files_color_.size())
            return cv::Mat();
        cv::Mat gray8 = cv::imread(files_color_[id], cv::IMREAD_GRAYSCALE);
        // convert gray to float
        cv::Mat gray;
        gray8.convertTo(gray, CV_32FC1, 1.0 / 255.0);
        return gray;
    }


    inline cv::Mat DatasetTUM::loadDepth(const size_t id) const
    {
        if (id >= files_depth_.size())
            return cv::Mat();
        //fill/read 16 bit depth image
        cv::Mat depth16 = cv::imread(files_depth_[id], cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);
        cv::Mat depth;
        depth16.convertTo(depth, CV_32FC1, (1.0 / 5000.0));

        // threshold depth if maximum depth specified
        if (cfg_.max_depth > 0.0f)
            threshold(depth, 0.0f, cfg_.max_depth);

        return depth;
    }


    inline double DatasetTUM::timeColor(const size_t id) const
    {
        if (id >= timestamps_color_.size())
            return 0.0;
        return timestamps_color_[id];
    }


    inline double DatasetTUM::timeDepth(const size_t id) const
    {
        if (id >= timestamps_depth_.size())
            return 0.0;
        return timestamps_depth_[id];
    }


    inline void DatasetTUM::threshold(cv::Mat &depth, float depth_min, float depth_max) const
    {
        if (depth.type() != CV_32FC1)
            return;

        cv::threshold(depth, depth, static_cast<double>(depth_min), 0.0, cv::THRESH_TOZERO);
        cv::threshold(depth, depth, static_cast<double>(depth_max), 0.0, cv::THRESH_TOZERO_INV);
    }

} // namespace ldvo
