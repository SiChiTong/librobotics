/*
 * lb_regression.h
 *
 *  Created on: Jan 19, 2009
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

#ifndef LB_REGRESSION_H_
#define LB_REGRESSION_H_

#include "lb_common.h"
#include "lb_data_type.h"


namespace librobotics {

template<typename T>
void lb_line_define(const std::vector<vec2<T> >& points,
                    LB_FLOAT &a,
                    LB_FLOAT &b,
                    LB_FLOAT &c)
{
    size_t end_idx = points.size() - 1;

    //Ax+By+C=0
    LB_FLOAT m1,m2;
    m1=points[0].y - points[end_idx].y;
    m2=points[0].x - points[end_idx].x;

    a = m1;
    b = -m2;
    c = m2*points[end_idx].y - m1*points[end_idx].x;
}

template<typename T>
inline void lb_liner_regression(const std::vector<vec2<T> >& points,
                                LB_FLOAT& m,
                                LB_FLOAT& b,
                                LB_FLOAT& r)
{
    size_t n = points.size();
    if(n > 0) {
        LB_FLOAT sum_x = 0;
        LB_FLOAT sum_y = 0;
        for(size_t i = 0; i < n; i++) {
            sum_x += points[i].x;
            sum_y += points[i].y;
        }

        LB_FLOAT mean_x = sum_x / n;
        LB_FLOAT mean_y = sum_y / n;

        std::vector<LB_FLOAT> x_err(n, 0);
        std::vector<LB_FLOAT> y_err(n, 0);

        for(size_t i = 0; i < n; i++) {
            x_err[i] = points[i].x - mean_x;
            y_err[i] = points[i].y - mean_y;
        }

        LB_FLOAT A = 0;
        LB_FLOAT sum_err_xy = 0;
        for (size_t i = 0; i < n; i++) {
            A = A + (LB_SQR(x_err[i]) - LB_SQR(y_err[i]));
            sum_err_xy += (x_err[i] * y_err[i]);
        }

        if (sum_err_xy == 0)
            sum_err_xy = 1e-6;

        A = A / sum_err_xy;

        // m^2 + A*m - 1 = 0
        LB_FLOAT m1 = (-A + sqrt(A*A + 4)) * 0.5;
        LB_FLOAT m2 = (-A - sqrt(A*A + 4)) * 0.5;

        LB_FLOAT b1 = mean_y - (m1 * mean_x);
        A = m1;
        LB_FLOAT B = -1;
        LB_FLOAT C = b1;

        //calculate the maximum error on m1
        r = 0;
        LB_FLOAT aux = sqrt(A*A + B*B);
        LB_FLOAT dist;
        for (size_t i = 0; i < n; i++) {
            dist = fabs((points[i].x*A + points[i].y*B + C)/aux);
            if (dist > r) r = dist;
        }

        LB_FLOAT r1 = r;
        LB_FLOAT b2 = mean_y - (m2 * mean_x);
        A = m2;
        B = -1;
        C = b2;

        //calculate the maximum error on m2
        r = 0;
        aux = sqrt(A*A + B*B);
        for (size_t i = 0; i < n; i++) {
            dist = fabs((points[i].x*A + points[i].y*B + C)/aux);
            if (dist > r) r = dist;
        }

        LB_FLOAT r2 = r;
        if (r1 > r2) {
            m = m2; b = b2; r = r2;
        } else {
            m = m1; b = b1; r = r1;
        }
    } else {
        m = b = r = 0;
        warn("%s number of point == 0", __FUNCTION__);
    }
}


}


#endif /* LB_REGRESSION_H_ */
