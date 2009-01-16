/*
 * lb_misc_function.h
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

#ifndef LB_MISC_FUNCTION_H_
#define LB_MISC_FUNCTION_H_

#include "lb_common.h"
#include "lb_data_type.h"

namespace librobotics {

/** \defgroup g_user_template User-friendly template functions */
/* @{ */



inline void build_angle_table(LB_FLOAT start, LB_FLOAT end, int step, std::vector<LB_FLOAT>& table) {
    if(table.size() != (size_t)step) table.resize(step);
    LB_FLOAT angle_step = (end - start) / step;
    for(int i = 0; i < step; i++) {
        table[i] = start + (i * angle_step);
    }
}

inline void build_cos_sin_table(LB_FLOAT start, LB_FLOAT end, int step,
                                std::vector<LB_FLOAT>& cos_table,
                                std::vector<LB_FLOAT>& sin_table)
{
    if(cos_table.size() != (size_t)step) cos_table.resize(step);
    if(sin_table.size() != (size_t)step) sin_table.resize(step);
    LB_FLOAT angle_step = (end - start) / step;
    for(int i = 0; i < step; i++) {
        cos_table[i] = cos(start + (i * angle_step));
        sin_table[i] = sin(start + (i * angle_step));
    }
}

inline void build_cos_sin_table(const std::vector<LB_FLOAT>& angle_table,
                                std::vector<LB_FLOAT>& cos_table,
                                std::vector<LB_FLOAT>& sin_table)
{
    int step = angle_table.size();
    if(cos_table.size() != (size_t)step) cos_table.resize(step);
    if(sin_table.size() != (size_t)step) sin_table.resize(step);
    for(int i = 0; i < step; i++) {
        cos_table[i] = cos(angle_table[i]);
        sin_table[i] = sin(angle_table[i]);
    }
}

/**
 * Normalize the angle to \f$(-\pi, \pi)\f$ radian unit
 * \param a input angle value
 * \return normalized angle
 */

inline LB_FLOAT normalize_angle(LB_FLOAT a) {
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

inline LB_FLOAT minimum_angle_distance(LB_FLOAT a, LB_FLOAT b) {
    a = normalize_angle(a);
    b = normalize_angle(b);

    LB_FLOAT diff = b - a;

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

inline void cumulative_sum(const std::vector<LB_FLOAT>& v, std::vector<LB_FLOAT>& sum) {
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
inline LB_FLOAT square_sum(const std::vector<LB_FLOAT>& v) {
    LB_FLOAT sum = 0;
    for(size_t i = 0; i < v.size(); i++) {
        sum += LB_SQR(v[i]);
    }
    return sum;
}

/* @} */

}


#endif /* LB_MISC_FUNCTION_H_ */
