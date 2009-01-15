/*
 * lb_template_function.h
 *
 *  Created on: Jan 15, 2009
 *      Author: mahisorn
 *
 *  Copyright (c) <2009> <Mahisorn Wongphati>
 *  Permission is hereby granted, free of charge, to any person
 *  obtaining a copy of this software and associated documentation
 *  files (the "Software"), to deal in the Software without
 *  restriction, including without limitation the rights to use,
 *  copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the
 *  Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be
 *  included in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *  OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef LB_TEMPLATE_FUNCTION_H_
#define LB_TEMPLATE_FUNCTION_H_

#include "lb_common.h"

/** \defgroup g_user_template User-friendly template functions */
/* @{ */
/**
 * Normalize the angle to \f$(-\pi, \pi)\f$ radian unit
 * \param a input angle value
 * \return normalized angle
 */
template<typename T>
inline T normalize_angle(T a) {
    int m = (int)(a / (2.0*M_PI));
    a = a - (m * M_PI * 2.0);
    if (a < (-M_PI))
        a += (2.0 * M_PI);
    if (a >= M_PI)
        a -= (2.0 * M_PI);
    return a;
};

/**
 * Find the minimum angle distance between two input angle in radian unit
 * \param a input angle value
 * \param b input angle value
 * \return minimum angle distance from a -> b
 */
template<typename T>
inline T minimum_angle_distance(T a, T b) {
    a = normalize_angle(a);
    b = normalize_angle(b);

    T diff = b - a;

    if(fabs(diff) > M_PI) {
        if(diff < 0)
            diff += (2 * M_PI);
        else
            diff -= (2 * M_PI);
    }
    return diff;
}

/**
 * Compute a cumulative summation.
 * @param v number vector
 * @param sum output vector
 */
template<typename T>
inline void cumulative_sum(const std::vector<T>& v, std::vector<T>& sum) {
    if(v.size() == 0) return;
    if(sum.size() != v.size()) sum.resize(v.size());
    sum[0] = v[0];
    for(size_t i = 1; i < v.size(); i++) {
        sum[i] = sum[i-1] + v[i];
    }
}

/**
 * Compute a square summation
 * @param v input
 * @return square sum of all value in v
 */
template<typename T>
inline T square_sum(const std::vector<T>& v) {
    T sum = 0;
    for(size_t i = 0; i < v.size(); i++) {
        sum += LB_SQR(v[i]);
    }
    return sum;
}

/* @} */


#endif /* LB_TEMPLATE_FUNCTION_H_ */
