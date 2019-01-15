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

#include <ldvo/timed_pose.hpp>

#include <iostream>
#include <iomanip>
#include <fstream>


namespace ldvo
{

    inline TimedPose::TimedPose(const Eigen::Matrix4f &pose, const double timePose) :
        pose_(pose),
        timePose_(timePose)
    {
    }


    Eigen::Matrix4f TimedPose::pose() const
    {
        return pose_;
    }


    double TimedPose::timeForPose() const
    {
        return timePose_;
    }


    inline bool TimedPose::save(const std::string &filename, const std::vector<TimedPose> &poses)
    {
        if (filename.empty() || poses.empty())
            return false;

        // open output file for TUM RGB-D benchmark poses
        std::ofstream out_file;
        out_file.open(filename.c_str());
        if (!out_file.is_open())
            return false;

        // write poses into output file
        out_file << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < poses.size(); i++)
        {
            //write into evaluation file
            double timestamp = poses[i].timePose_;
            Eigen::Matrix4f pose = poses[i].pose_;
            //timestamp
            out_file << timestamp << " ";
            //translation
            Eigen::Vector3f translation = pose.topRightCorner(3,1);
            out_file << translation[0] << " " << translation[1] << " " << translation[2];
            //rotation (quaternion)
            Eigen::Matrix3f rot = pose.topLeftCorner(3,3);
            Eigen::Quaternionf quat(rot);
            out_file << " " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
        }
        out_file.close();

        return true;
    }

} // namespace ldvo
