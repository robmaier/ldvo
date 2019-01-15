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



namespace ldvo
{
namespace math
{

/**
 * @brief   Minimal SO3/SE3 Lie Algebra implementation.
 * @details Reference:
 *          @article{blanco2010tutorial,
 *              title={A tutorial on se (3) transformation parameterizations and on-manifold optimization},
 *              author={Blanco, Jose-Luis},
 *              year={2010},
 *              url={http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf}
 *          }
 * @author  Robert Maier
 */
namespace lie
{

/**
 * @brief   Hat operator implementation, i.e.
 *          build the skew-symmetric matrix for a vector.
 * @param   vec     Input 3D vector.
 * @return  Skew-symmetric 3x3 matrix hat(vec).
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> hat(const Eigen::Matrix<T, 3, 1> &vec)
{
    Eigen::Matrix<T, 3, 3> mat_hat;
    mat_hat << 0, -vec[2], vec[1],
               vec[2], 0, -vec[0],
               -vec[1], vec[0], 0;
    return mat_hat;
}


/**
 * @brief   SO3 Lie Algebra.
 * @author  Robert Maier
 */
namespace so3
{

    /**
     * @brief   Exponential map to convert from 3D twist coordinates
     *          to 3x3 rotation matrix.
     * @param   w   Input 3D twist coordinates.
     * @return  3x3 rotation matrix.
     */
    template <typename T>
    inline Eigen::Matrix<T, 3, 3> exp(const Eigen::Matrix<T, 3, 1> &w)
    {
        // calculate rotation matrix from twist coordinates
        T w_length = w.norm();
        // hat operator
        Eigen::Matrix<T, 3, 3> w_hat = hat(w);
        // calculate rotation matrix
        Eigen::Matrix<T, 3, 3> R;
        if (w_length == 0.0f)
        {
            R = Eigen::Matrix<T, 3, 3>::Identity();
        }
        else
        {
            R = Eigen::Matrix<T, 3, 3>::Identity() +
                    w_hat / w_length * std::sin(w_length) +
                    (w_hat * w_hat) / (w_length * w_length) * (1.0f - std::cos(w_length));
        }
        return R;
    }


    /**
     * @brief   Logarithm map to convert from 3x3 rotation matrix
     *          to 3D twist coordinates.
     * @param   w   Input 3x3 rotation matrix.
     * @return  3D twist coordinates.
     */
    template <typename T>
    inline Eigen::Matrix<T, 3, 1> log(const Eigen::Matrix<T, 3, 3> &R)
    {
        // calculate twist coordinates for rotation
        Eigen::Matrix<T, 3, 1> twist;
        T w_length = std::acos((R.trace() - 1.0f) * 0.5f);
        if (w_length == 0.0f)
        {
            twist = Eigen::Matrix<T, 3, 1>::Zero();
        }
        else
        {
            Eigen::Matrix<T, 3, 1> a(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1));
            twist = 1.0f / (2.0f * std::sin(w_length)) * a * w_length;
        }
        return twist;
    }

} // namespace so3


/**
 * @brief   SE3 Lie Algebra.
 * @author  Robert Maier
 */
namespace se3
{

    /**
     * @brief   Exponential map to convert from 6D twist coordinates
     *          (rotation + translation) to 4x4 transformation matrix.
     * @param   w   Input 6D twist coordinates.
     * @return  4x4 transformation matrix.
     */
    template <typename T>
    inline Eigen::Matrix<T, 4, 4> exp(const Eigen::Matrix<T, 6, 1> &twist)
    {
        // extract v and w from 6D twist vector
        Eigen::Matrix<T, 3, 1> v = twist.template topRows<3>();
        Eigen::Matrix<T, 3, 1> w = twist.template bottomRows<3>();

        // calculate rotation matrix from twist
        Eigen::Matrix<T, 3, 3> R = so3::exp(w);

        // hat operator
        Eigen::Matrix<T, 3, 3> w_hat = hat(w);
        T w_length = w.norm();
        // compute translation vector
        Eigen::Matrix<T, 3, 1> t;
        if (w_length == 0.0f)
        {
            t = Eigen::Matrix<T, 3, 1>::Zero();
        }
        else
        {
            Eigen::Matrix<T, 3, 3> V = Eigen::Matrix<T, 3, 3>::Identity() +
                    w_hat * (1.0f - std::cos(w_length)) / (w_length * w_length) +
                    (w_hat * w_hat) * (w_length - std::sin(w_length)) / (w_length * w_length * w_length);
            t = V * v;
        }

        // fill output matrix
        Eigen::Matrix<T, 4, 4> mat = Eigen::Matrix<T, 4, 4>::Identity();
        mat.template topLeftCorner<3,3>() = R;
        mat.template topRightCorner<3,1>() = t;
        return mat;
    }


    /**
     * @brief   Logarithm map to convert from 4x4 transformation matrix
     *          to 6D twist coordinates (that contain both rotation and
     *          translation).
     * @param   w   Input 4x4 transformation matrix.
     * @return  6D twist coordinates.
     */
    template <typename T>
    inline Eigen::Matrix<T, 6, 1> log(const Eigen::Matrix<T, 4, 4> &pose)
    {
        // extract rotation matrix and translation vector
        Eigen::Matrix<T, 3, 3> R = pose.template topLeftCorner<3,3>();
        Eigen::Matrix<T, 3, 1> t = pose.template topRightCorner<3,1>();

        // calculate twist coordinates for rotation
        Eigen::Matrix<T, 3, 1> w = so3::log(R);

        // hat operator
        Eigen::Matrix<T, 3, 3> w_hat = hat(w);
        // calculate translational part
        T w_length = w.norm();
        Eigen::Matrix<T, 3, 3> V_inv = Eigen::Matrix<T, 3, 3>::Identity() -
                0.5f * w_hat +
                (1.0f - (w_length * std::cos(w_length * 0.5f)) / (2.0f * std::sin(w_length * 0.5f))) *
                    (w_hat * w_hat) / (w_length * w_length);
        Eigen::Matrix<T, 3, 1> v = V_inv * t;

        // combine v and w in 6D twist vector
        Eigen::Matrix<T, 6, 1> twist;
        twist.template topRows<3>() = v;
        twist.template bottomRows<3>() = w;
        return twist;
    }


    /**
     * @brief   Concatenation of two transformations in 6D twist coordinates
     *          through matrix-matrix-multiplication of the respective
     *          4x4 transformation matrices.
     * @param   xi0     First transformation (6D twist coordinates).
     * @param   xi1     First transformation (6D twist coordinates).
     * @return  6D twist coordinates after concatenation.
     */
    template <typename T>
    inline Eigen::Matrix<T, 6, 1> concatenate(const Eigen::Matrix<T, 6, 1> &xi0,
                                              const Eigen::Matrix<T, 6, 1> &xi1)
    {
        return se3::log<T>(se3::exp<T>(xi0) * se3::exp<T>(xi1));
    }

} // namespace se3

} // namespace lie
} // namespace math
} // namespace ldvo
