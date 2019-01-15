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
#include <vector>
#include <Eigen/Dense>


namespace ldvo
{

    /**
     * @brief   Class for storing a camera pose with its associated timestamp.
     * @author  Robert Maier (robert.maier@tum.de)
     */
    class TimedPose
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief   Constructor that fills a TimedPose object with a
         *          pose and timestamp.
         */
        TimedPose(const Eigen::Matrix4f &pose, const double timePose);

        /// Get the stored pose.
        Eigen::Matrix4f pose() const;

        /// Get the stored timestamp for the pose.
        double timeForPose() const;

        /**
         * @brief   Static method for saving a vector of TimedPose objects to a file.
         * @return  True if successful, false otherwise.
         */
        static bool save(const std::string &filename, const std::vector<TimedPose> &poses);

    protected:
        Eigen::Matrix4f pose_;
        double timePose_;
    };

} // namespace ldvo

#include <ldvo/detail/timed_pose.inl>
