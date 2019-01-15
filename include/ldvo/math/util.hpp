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

#include <iostream>
#include <Eigen/Dense>

#include <opencv2/core/core.hpp>


namespace ldvo
{
namespace math
{

/**
 * @brief   Various math utility functions.
 * @author  Robert Maier
 */
namespace util
{

    /**
     * @brief   Get an interpolated image value for a sub-pixel point (x,y)
     *          through bilinear interpolation.
     * @param   data    Input image.
     * @param   x       x-coordinate for lookup.
     * @param   y       y-coordinate for lookup.
     * @param   w       Input image width (for boundary checks).
     * @param   h       Input image height (for boundary checks).
     * @return  Interpolated image value at input point (x,y).
     */
    float interpolate(const float* data, const float x, const float y, const int w, const int h);

    /**
     * @brief   Get an interpolated image value for a sub-pixel point pt
     *          through bilinear interpolation.
     * @param   data    Input image.
     * @param   pt      2D point to be sampled.
     * @return  Interpolated image value at input point.
     */
    float interpolate(const cv::Mat &data, const Eigen::Vector2f &pt);

    /**
     * @brief   Compute the gradient of an image for a specified direction.
     * @param   data        Input image.
     * @param   grad_out    Output image gradient.
     * @param   direction   Direction in which the gradient is computed,
     *                      i.e. 0=x-direction, 1=y-direction.
     */
    void computeGradient(const cv::Mat &data, cv::Mat &grad_out, int direction);

    /**
     * @brief   Calculate the mean value over all pixels of an image.
     * @param   data    Input image.
     * @return  Mean value of all image pixels.
     */
    float calculateMean(const cv::Mat &data);

    /**
     * @brief   Calculate the standard deviation of all pixels of an image.
     * @param   data    Input image.
     * @param   mean    Pre-computed mean value of all image pixels.
     * @return  Standard deviation of all image pixels.
     */
    float calculateStdDev(const cv::Mat &data, const float mean);

    /**
     * @brief   Calculate the standard deviation of all pixels of an image.
     *          (Computes mean internally first)
     * @param   data    Input image.
     * @return  Standard deviation of all image pixels.
     */
    float calculateStdDev(const cv::Mat &data);

    /**
     * @brief   Calculates the Huber weight for a residual.
     * @param   residual    Input residual value.
     * @param   huber_k     Huber constant.
     * @return  Calculated Huber weight.
     */
    float calculateHuberWeight(const float residual, const float huber_k);

} // namespace util
} // namespace math
} // namespace ldvo

#include <ldvo/math/util.inl>
