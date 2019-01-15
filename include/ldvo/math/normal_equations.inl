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

#include <ldvo/math/normal_equations.hpp>

#include <iostream>
#include <fstream>


namespace ldvo
{
namespace math
{

    template<int N>
    inline NormalEquations<N>::NormalEquations() :
        JtJ_(Eigen::Matrix<float, N, N>::Zero()),
        JtR_(Eigen::Matrix<float, N, 1>::Zero()),
        error_(0.0),
        num_residuals_(0)
    {
    }


    template<int N>
    inline NormalEquations<N>::~NormalEquations()
    {
    }


    template<int N>
    void NormalEquations<N>::reset()
    {
        JtJ_.setZero();
        JtR_.setZero();
        error_ = 0.0;
        num_residuals_ = 0;
    }


    template<int N>
    void NormalEquations<N>::update(const Eigen::Matrix<float, N, 1> &jacobian_row,
                                 const float residual,
                                 const float weight)
    {
        for (int k = 0; k < N; ++k)
        {
            // update A = Jt*J (including robust weights)
            for (int j = 0; j < N; ++j)
                JtJ_(k, j) += jacobian_row[j] * weight * jacobian_row[k];

            // update b = Jt*r (apply robust weights)
            JtR_[k] += jacobian_row[k] * weight * residual;
        }

        // update overall error
        error_ += static_cast<double>(residual * weight * residual);

        // update number of added residuals
        ++num_residuals_;
    }


    template<int N>
    Eigen::Matrix<float, N, 1> NormalEquations<N>::solve() const
    {
        // solve normal equations using Cholesky LDLT decomposition
        return -(JtJ_.ldlt().solve(JtR_));
    }


    template<int N>
    double NormalEquations<N>::error() const
    {
        return error_;
    }


    template<int N>
    double NormalEquations<N>::averageError() const
    {
        // calculate and return final overall error
        if (num_residuals_ > 0)
            return error_ / static_cast<double>(num_residuals_);
        else
            return 0.0;
    }

} // namespace math
} // namespace ldvo
