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

#include <ldvo/tracker.hpp>

#include <iostream>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <ldvo/math/lie_algebra.hpp>
#include <ldvo/math/util.hpp>
#include <ldvo/math/normal_equations.hpp>


namespace ldvo
{

    inline void Tracker::Config::print() const
    {
        std::cout << "Tracker config: " << std::endl;
        std::cout << "   pyramid levels: " << num_levels << std::endl;
        std::cout << "   min pyramid level: " << min_level << std::endl;
        std::cout << "   max pyramid level: " << max_level << std::endl;
        std::cout << "   iterations: " << num_iterations << std::endl;
        std::cout << "   update threshold: " << update_thres << std::endl;
        std::cout << "   huber weights: " << int(use_huber_weights) << std::endl;
        std::cout << "   depth weighting: " << int(use_depth_weighting) << std::endl;

    }


    inline void Tracker::Config::validate()
    {
        // check pyramid levels
        if (num_levels < 1 || num_levels > 10)
            num_levels = 1;
        if (min_level < 0 || min_level >= num_levels)
            min_level = 0;
        if (max_level < 0 || max_level >= num_levels || max_level < min_level)
            max_level = num_levels - 1;
    }


    inline Tracker::Tracker(const Config &cfg, const CameraModel &cam) :
        cfg_(cfg),
        cam_(cam, cfg.num_levels),
        residuals_(cam.width(), cam.height(), cfg.num_levels)
    {
    }


    inline Tracker::~Tracker()
    {
    }


    inline const Tracker::Config& Tracker::config() const
    {
        return cfg_;
    }


    inline bool Tracker::align(const FramePyramid &prev_pyramid,
                               const FramePyramid &cur_pyramid,
                               Eigen::Matrix4f &pose_prev_to_cur)
    {
        if (prev_pyramid.levels() != cfg_.num_levels ||
                cur_pyramid.levels() != cfg_.num_levels)
            return false;

        // convert transformation matrix to twist coordinates
        Eigen::Matrix<float, 6, 1> xi = math::lie::se3::log<float>(pose_prev_to_cur);

        for (int lvl = cfg_.max_level; lvl >= cfg_.min_level; --lvl)
        {
            for (int itr = 0; itr < cfg_.num_iterations; ++itr)
            {
                // compute residuals and update
                Eigen::Matrix<float, 6, 1> xi_update = computeUpdate(prev_pyramid.at(lvl),
                                                                     cur_pyramid.at(lvl),
                                                                     lvl, xi);
                // apply update: left-multiplicative increment on SE3
                xi = math::lie::se3::concatenate<float>(xi_update, xi);

                // break if update becomes small
                if (cfg_.update_thres > 0.0f && xi_update.norm() < cfg_.update_thres)
                    break;
            }
        }

        // store to output pose
        pose_prev_to_cur = math::lie::se3::exp<float>(xi);

        return true;
    }


    inline void Tracker::calculateResiduals(const cv::Mat &gray_prev,
                                            const cv::Mat &depth_prev,
                                            const cv::Mat &gray_cur,
                                            const CameraModel &cam,
                                            const Eigen::Affine3f &pose_prev_to_cur,
                                            cv::Mat &residuals) const
    {
        // fill residuals
        residuals.setTo(0.0);
        for (int y = 0; y < gray_prev.rows; ++y)
        {
            for (int x = 0; x < gray_prev.cols; ++x)
            {
                float d_prev = depth_prev.at<float>(y, x);
                if (d_prev == 0.0f)
                    continue;

                // backproject 2d point into 3d using its depth
                const Eigen::Vector3f pt3_prev = cam.backProject(x, y, d_prev);
                // transform reference 3d point into current frame
                const Eigen::Vector3f pt3_cur = pose_prev_to_cur * pt3_prev;
                if (pt3_cur[2] <= 0.0f)
                    continue;

                // project 3d point to 2d and sample intensity
                float val_cur = math::util::interpolate(gray_cur, cam.project(pt3_cur));
                if (!std::isnan(val_cur))
                {
                    residuals.at<float>(y, x) = val_cur - gray_prev.at<float>(y, x);
                }
            }
        }
    }


    inline Eigen::Matrix<float, 6, 1> Tracker::calculateJacobian(const float g_x,
                                                                 const float g_y,
                                                                 const CameraModel &cam,
                                                                 const Eigen::Vector3f &pt3_cur) const
    {
        // shorthands
        const float fg_x = cam.fx() * g_x;
        const float fg_y = cam.fy() * g_y;
        const float pt3_z_inv = 1.0f / pt3_cur[2];
        const float pt3_z_inv_sqr = pt3_z_inv * pt3_z_inv;

        // analytic partial derivates (for more details see Christian Kerl's master's thesis)
        Eigen::Matrix<float, 6, 1> jacobian;
        jacobian[0] = fg_x * pt3_z_inv;
        jacobian[1] = fg_y * pt3_z_inv;
        jacobian[2] = - (fg_x * pt3_cur[0] + fg_y * pt3_cur[1]) * pt3_z_inv_sqr;
        jacobian[3] = - (fg_x * pt3_cur[0] * pt3_cur[1]) * pt3_z_inv_sqr - fg_y * (1 + (pt3_cur[1] * pt3_z_inv) * (pt3_cur[1] * pt3_z_inv));
        jacobian[4] = fg_x * (1.0f + (pt3_cur[0] * pt3_z_inv) * (pt3_cur[0] * pt3_z_inv)) + (fg_y * pt3_cur[0] * pt3_cur[1]) * pt3_z_inv_sqr;
        jacobian[5] = (- fg_x * pt3_cur[1] + fg_y * pt3_cur[0]) * pt3_z_inv;
        return jacobian;
    }


    inline float Tracker::calculateResidualWeight(const float residual,
                                                  const float residuals_stddev,
                                                  const float depth) const
    {
        float w = 1.0f;
        if (cfg_.use_huber_weights)
        {
            // compute parameter for Huber weights
            const float huber_weights_k = 1.345f * residuals_stddev;
            // compute robust Huber weight for residual
            w *= math::util::calculateHuberWeight(residual, huber_weights_k);
        }
        if (cfg_.use_depth_weighting)
        {
            // integrate depth into weight (depth certainty decreases quadratically with depth)
            w /= (depth * depth);
        }
        return w;
    }


    inline Eigen::Matrix<float, 6, 1> Tracker::computeUpdate(const Frame& prev_frame,
                                                             const Frame& cur_frame,
                                                             const int level,
                                                             const Eigen::Matrix<float, 6, 1> &xi)
    {
        // shorthands
        const CameraModel& cam = cam_.at(level);
        cv::Mat& residuals = residuals_.at(level);

        // convert SE3 to affine transformation
        Eigen::Affine3f pose_prev_to_cur(math::lie::se3::exp<float>(xi));

        // calculate per-pixel residuals
        calculateResiduals(prev_frame.gray(), prev_frame.depth(), cur_frame.gray(),
                           cam, pose_prev_to_cur, residuals);
        // compute standard deviation of residuals
        const float residuals_stddev = math::util::calculateStdDev(residuals);

        // fill normal equations
        math::NormalEquations<6> ne;
        for (int y = 0; y < prev_frame.height(); ++y)
        {
            for (int x = 0; x < prev_frame.width(); ++x)
            {
                const float d_prev = prev_frame.depth().at<float>(y, x);
                if (d_prev <= 0.0f)
                    continue;

                // project 2d point back into 3d using its depth
                const Eigen::Vector3f pt3_prev = cam.backProject(x, y, d_prev);
                // transform reference 3d point into current frame
                const Eigen::Vector3f pt3_cur = pose_prev_to_cur * pt3_prev;
                if (pt3_cur[2] <= 0.0f)
                    continue;

                // project 3d point to 2d
                const Eigen::Vector2f pt2_cur = cam.project(pt3_cur);
                // compute interpolated image gradient
                const float g_x = math::util::interpolate(cur_frame.gradientX(), pt2_cur);
                const float g_y = math::util::interpolate(cur_frame.gradientY(), pt2_cur);
                if (!std::isnan(g_x) && !std::isnan(g_y))
                {
                    // compute Jacobian row, residual and residual weight
                    Eigen::Matrix<float, 6, 1> jacobian_row = calculateJacobian(g_x, g_y, cam, pt3_cur);
                    float residual = residuals.at<float>(y, x);
                    float residual_weight = calculateResidualWeight(residual, residuals_stddev, d_prev);
                    // update normal equations
                    ne.update(jacobian_row, residual, residual_weight);
                }
            }
        }

        // solve normal equations
        const Eigen::Matrix<float, 6, 1> xi_update = ne.solve();
        // return se3 pose update
        return xi_update;
    }

} // namespace ldvo
