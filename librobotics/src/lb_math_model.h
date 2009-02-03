/*
 * lb_math_model.h
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

#ifndef LB_MATH_MODEL_H_
#define LB_MATH_MODEL_H_

#include "lb_common.h"
#include "lb_data_type.h"
#include "lb_statistic_function.h"

namespace librobotics {

/**
 * Measurement model for range sensor from CH6 of Probabilistic Robotics book.
 * http://robots.stanford.edu/probabilistic-robotics/ \n
 * @param x measurement data
 * @param x_hit expected measurement range
 * @param x_max maximum possible measurement range
 * @param var_hit variance of the measurement
 * @param rate_short rate of exponential distribution
 * @param z[4] weighted average for z_hit, z_short, z_max. z_rand and z[0]+z[1]+z[2]+z[3]=1;
 * @return
 */
inline LB_FLOAT lb_beam_range_finder_measurement_model(const LB_FLOAT x,
                                                       const LB_FLOAT x_hit,
                                                       const LB_FLOAT x_max,
                                                       const LB_FLOAT var_hit,
                                                       const LB_FLOAT rate_short,
                                                       const LB_FLOAT z[4])
{
    LB_FLOAT p_hit =   ((x > 0) && (x <= x_max)) ? lb_pdf_normal_dist(var_hit, x_hit, x) : 0.0;
    LB_FLOAT p_short = ((x > 0) && (x <= x_hit)) ? lb_pdf_exponential_dist(rate_short, x) : 0.0;
    p_short *= 1.0 / (1.0 - exp(-rate_short * x_hit));
    LB_FLOAT p_max =   (x <= 0 || x >= x_max ? 1.0 : 0.0);
    LB_FLOAT p_rand =  ((x > 0) && (x < x_max)) ? 1.0/x_max : 0.0;
    return (z[0] * p_hit) + (z[1] * p_short) + (z[2] * p_max)  + (z[3] * p_rand);
}


/**
 * Sample base velocity motion model for sampling pose \f$x_t = (x',y',\theta')^T\f$
 * from give initial pose and control.
 * @param ut control \f$(v, w)^T\f$
 * @param p initial pose \f$(x, y, \theta)^T\f$
 * @param dt update time
 * @param var robot specific motion error parameters \f$(\sigma_1 ... \sigma_6) \f$
 * @return pose \f$x_t = (x',y',\theta')^T\f
 */

inline pose2f lb_velocity_motion_model_sample(const vec2f& ut,
                                              const pose2f& p,
                                              const LB_FLOAT dt,
                                              const LB_FLOAT var[6])
{
    LB_FLOAT v2 = LB_SQR(ut.x);
    LB_FLOAT w2 = LB_SQR(ut.y);
    LB_FLOAT v = ut.x + lb_sample_normal_dist(var[0]*v2 + var[1]*w2);
    LB_FLOAT w = ut.y + lb_sample_normal_dist(var[2]*v2 + var[3]*w2);
    LB_FLOAT r = lb_sample_normal_dist(var[4]*v2 + var[5]*w2);

    LB_FLOAT x, y, a, v_w;
    if(w != 0) {
        v_w = v/w;
        x = p.x - v_w*sin(p.a) + v_w*sin(p.a + w*dt);
        y = p.y + v_w*cos(p.a) - v_w*cos(p.a + w*dt);
        a = lb_normalize_angle(p.a + w*dt + r*dt);
    } else {
        x = p.x + v*cos(p.a);
        y = p.y + v*sin(p.a);
        a = lb_normalize_angle(p.a + r*dt);
    }
    return pose2f(x, y, a);
}

/**
 * Closed form velocity motion model for compute \f$p(x_t|u_t, x_{t-1})\f$
 * @param pt    hypothesized pose \f$(x', y', \theta')^T\f$
 * @param ut    control \f$(v, w)^T\f$
 * @param p     initial pose \f$(x, y, \theta)^T\f$
 * @param dt    update time
 * @param var   robot specific motion error parameters \f$(\sigma_1 ... \sigma_6) \f$
 * @return \f$p(x_t|u_t, x_{t-1})\f$
 */

inline LB_FLOAT velocity_motion(pose2f pt,
                                vec2f ut,
                                pose2f p,
                                LB_FLOAT dt,
                                LB_FLOAT var[6])
{
    LB_FLOAT x_x = p.x - pt.x;
    LB_FLOAT y_y = p.y - pt.y;
    LB_FLOAT tmp0 = ((x_x*cos(p.a)) + (y_y*sin(p.a)));
    LB_FLOAT tmp1 = ((y_y*cos(p.a)) - (x_x*sin(p.a)));
    LB_FLOAT u, xx, yy, rr, aa, v, w;

    u = 0.0;
    if(tmp1 != 0) {
        u = 0.5 * (tmp0/tmp1);
    }

    //compute center and radius of the circle
    xx = ((p.x + pt.x) * 0.5) + (u*(p.y - pt.y));
    yy = ((p.y + pt.y) * 0.5) + (u*(pt.x - p.x));
    rr = sqrt(LB_SQR(pt.x - xx) + LB_SQR(pt.y - yy));

    aa = lb_minimum_angle_distance(atan2(p.y - yy, p.x - xx), atan2(pt.y - yy, pt.x - xx));
    v = aa/dt * rr;

    if(ut.x >= 0) {
        if(LB_SIGN(yy) == LB_SIGN(aa))
            v = LB_SIGN(ut.x) * fabs(v);   //check sign with control input
        else
            return 0;
    } else {
        if(LB_SIGN(yy) != LB_SIGN(aa))
            v = LB_SIGN(ut.x) * fabs(v);   //check sign with control input
        else
            return 0;
    }

    w = aa/dt;

    LB_FLOAT v2 = v*v;
    LB_FLOAT w2 = w*w;
    LB_FLOAT r = lb_minimum_angle_distance(w, (pt.a - p.a)/dt);


    LB_FLOAT p0 = lb_pdf_normal_dist(var[0]*v2 + var[1]*w2, 0.0, v - ut.x);
    LB_FLOAT p1 = lb_pdf_normal_dist(var[2]*v2 + var[3]*w2, 0.0, lb_minimum_angle_distance(ut.y, w));
    LB_FLOAT p2 = lb_pdf_normal_dist(var[4]*v2 + var[5]*w2, 0.0, r);

    return p0 * p1 * p2;
}


/**
 * Sample based odometry motion model
 * @param u_pt
 * @param u_p
 * @param p
 * @param var
 * @return
 */

inline pose2f lb_odometry_motion_model_sample(const pose2f& u_pt,
                                              const pose2f& u_p,
                                              const pose2f& p,
                                              const LB_FLOAT var[4])
{
    LB_FLOAT rot1 = lb_minimum_angle_distance(u_p.a, atan2(u_pt.y - u_p.y, u_pt.x - u_p.x));
    LB_FLOAT tran = (u_pt.get_vec2() - u_p.get_vec2()).size();
    LB_FLOAT rot2 = lb_minimum_angle_distance(rot1, lb_minimum_angle_distance(u_p.a, u_pt.a));

    LB_FLOAT rot1_sqr = LB_SQR(rot1);
    LB_FLOAT tran_sqr = LB_SQR(tran);
    LB_FLOAT rot2_sqr = LB_SQR(rot2);

    LB_FLOAT nrot1 = rot1 + lb_sample_normal_dist(var[0]*rot1_sqr + var[1]*tran_sqr);
    LB_FLOAT ntran = tran + lb_sample_normal_dist(var[2]*tran_sqr + var[3]*rot1_sqr + var[3]*rot2_sqr);
    LB_FLOAT nrot2 = rot2 + lb_sample_normal_dist(var[0]*rot2_sqr + var[1]*tran_sqr);

    LB_FLOAT x = p.x + ntran*cos(p.a + nrot1);
    LB_FLOAT y = p.y + ntran*sin(p.a + nrot1);
    LB_FLOAT a = lb_normalize_angle(p.a + nrot1 + nrot2);
    return pose2f(x, y, a);
}

/**
 * Odometry motion model
 * @param pt
 * @param u_pt
 * @param u_p
 * @param p
 * @param var
 * @return
 */
LB_FLOAT odometry_motion(pose2f pt,
                         pose2f u_pt,
                         pose2f u_p,
                         pose2f p,
                         LB_FLOAT var[6])
{
    LB_FLOAT rot1 = lb_minimum_angle_distance(u_p.a, atan2(u_pt.y - u_p.y, u_pt.x - u_p.x));
    LB_FLOAT tran = (u_pt.get_vec2() - u_p.get_vec2()).size();
    LB_FLOAT rot2 = lb_minimum_angle_distance(rot1, lb_minimum_angle_distance(u_p.a, u_pt.a));

    LB_FLOAT nrot1 = lb_minimum_angle_distance(p.a, atan2(pt.y - p.y, pt.x - p.x));
    LB_FLOAT ntran = (pt.get_vec2() - p.get_vec2()).size();
    LB_FLOAT nrot2 = lb_minimum_angle_distance(nrot1, lb_minimum_angle_distance(p.a, pt.a));

    LB_FLOAT nrot1_sqr = LB_SQR(nrot1);
    LB_FLOAT ntran_sqr = LB_SQR(ntran);
    LB_FLOAT nrot2_sqr = LB_SQR(nrot2);

    LB_FLOAT p0 = lb_pdf_normal_dist(var[0]*nrot1_sqr + var[1]*ntran_sqr, 0.0, lb_minimum_angle_distance(nrot1, rot1));
    LB_FLOAT p1 = lb_pdf_normal_dist(var[2]*ntran_sqr + var[3]*nrot1_sqr + var[3]*nrot2_sqr , 0.0, tran - ntran);
    LB_FLOAT p2 = lb_pdf_normal_dist(var[0]*nrot2_sqr + var[1]*ntran_sqr, 0.0, lb_minimum_angle_distance(nrot2, rot2));

    return p0 * p1 * p2;
}

}


#endif /* LB_MATH_MODEL_H_ */
