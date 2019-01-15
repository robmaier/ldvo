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
#include <Eigen/Cholesky>


namespace ldvo
{
namespace math
{

    /**
     * @brief   Normal equations for solving least squares problems.
     *          JtJ and JtR are always updated on-the-fly, the full
     *          Jacobian is never stored in memory.
     * @author  Robert Maier
     */
    template <int N>
    class NormalEquations
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// Constructor for creating empty normal equations.
        NormalEquations();

        /// Destructor.
        ~NormalEquations();

        /// Reset normal equations.
        void reset();

        /**
         * @brief   Update normal equations with a new residual and its
         *          respective weight and Jacobian.
         *          JtJ and JtR are updated directly on-the-fly.
         * @param   jacobian_row    Jacobian row for the residual.
         * @param   residual        Residual.
         * @param   weight          Weight of the residual/Jacobian.
         */
        void update(const Eigen::Matrix<float, N, 1> &jacobian_row,
                    const float residual,
                    const float weight = 1.0f);

        /**
         * @brief   Solve normal equations using Cholesky LDLT decomposition.
         * @return  Solution, i.e. parameter update.
         */
        Eigen::Matrix<float, N, 1> solve() const;

        /// Get overall cost (computed from added residuals and their weights).
        double error() const;

        /// Get average per-residual cost.
        double averageError() const;

    protected:
        Eigen::Matrix<float, N, N> JtJ_;
        Eigen::Matrix<float, N, 1> JtR_;
        double error_;
        size_t num_residuals_;
    };

} // namespace math
} // namespace ldvo

#include <ldvo/math/normal_equations.inl>
