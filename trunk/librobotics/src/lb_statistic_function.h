/*
 * lb_statistic_function.h
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
 *  OTHER DEALINGS IN THE SOFTWARE.*
 */

#ifndef LB_STATISTIC_FUNCTION_H_
#define LB_STATISTIC_FUNCTION_H_

#include "lb_common.h"
#include "lb_macro_function.h"
#include "lb_tools.h"

namespace librobotics {

/**
 * Probability density function (PDF) of the normal distribution.
 * @param v variance
 * @param m mean
 * @param x
 * @return PDF(x)
 */
inline LB_FLOAT lb_pdf_normal_dist(LB_FLOAT v, LB_FLOAT m, LB_FLOAT x) {
    if(v <= 0) return 0;
    return (1.0/sqrt(2*M_PI*v)) * exp(-LB_SQR(x - m) / (2.0 * v));
}

/**
 * Probability density function (PDF) of the triangular distribution.
 * @param v
 * @param m
 * @param x
 * @return PDF(x)
 */
inline LB_FLOAT lb_pdf_normal_triangular_dist(LB_FLOAT v, LB_FLOAT m, LB_FLOAT x) {
    //SQRT6 = 2.449489743
    if(v <= 0) return 0;
    LB_FLOAT p = (1.0/(2.449489743 * sqrt(v))) - (fabs(x - m)/(6.0 * v));
    return LB_MAX(0.0, p);
}

/**
 * PDF of the exponential distribution.
 * @param rate
 * @param x
 * @return PDF(x)
 */
inline LB_FLOAT lb_pdf_exponential_dist(LB_FLOAT rate, LB_FLOAT x) {
    if(x < 0) return 0;
    return rate * exp(-rate * x);
}

/**
 * Use a specific srand initialization to avoid multi-threading problems
 * (executed only one time for a single program).
 */
inline void  lb_srand() {
    static bool first_run = true;
    if(first_run) {
        std::srand(utils_get_current_time());
        unsigned char *const rand_mem = new unsigned char[1+std::rand()%2048];
        std::srand((unsigned int)(std::rand() + (unsigned long)rand_mem));
        delete[] rand_mem;
        first_run = false;
    }
}

/**
 * Return a random variable between \f$[0,1]\f$ with respect to an uniform distribution.
 */
inline LB_FLOAT lb_rand() {
    return (LB_FLOAT)std::rand()/RAND_MAX;
}

/**
 * Return a random variable between \f$[-1,1]\f$ with respect to an uniform distribution.
 */
inline LB_FLOAT lb_crand() {
    return 1.0 - (2.0 * lb_rand());
}

/**
 * Sample a random value from (approximate) normal distribution with zero mean.
 * @param v variance
 * @return random sample from normal distribution with zero mean
 */
inline LB_FLOAT lb_sample_normal_dist(LB_FLOAT v) {
    lb_srand();
    LB_FLOAT sum = 0;
    for(int i = 0; i < 12; i++) {
        sum += (lb_crand() * v);
    }
    return sum/2.0;
}

/**
 * Sample a random value from (approximate) triangular distribution with zero mean.
 * @param v variance
 * @return random sample from triangular distribution with zero mean
 */
inline LB_FLOAT lb_sample_triangular_dist(LB_FLOAT v) {
    //SQRT(6) / 2 = 1.224744871
    lb_srand();
    return 1.224744871 * ((lb_crand()*v) +  (lb_crand()*v));
}

/**
 * Sample a random value from uniform distribution in a circle.
 * (http://www.comnets.uni-bremen.de/itg/itgfg521/per_eval/p001.html)
 * @param a angle result \f$[-\pi, \pi]\f$
 * @param r radius result \f$[-1, 1]\f$
 */
inline void lb_sample_circle_uniform_dist(LB_FLOAT& a, LB_FLOAT& r) {
    lb_srand();
    a = lb_crand() * M_PI;
    r = sqrt(lb_rand());
}


inline void lb_stratified_random(std::vector<LB_FLOAT>& v, size_t n) {
    LB_FLOAT k = 1.0/n;
    LB_FLOAT k_2 = k * 0.5;
    if(v.size() != n) v.resize(n);
    for(size_t i = 0; i < n; i++) {
        v[i] = (k_2 + (k*i)) + (lb_rand() * k) - k_2;
    }
}

/**
 * Compute a cumulative summation.
 * @param v number vector
 * @param sum output vector
 */

inline void lb_cumulative_sum(const std::vector<LB_FLOAT>& v, std::vector<LB_FLOAT>& sum) {
    size_t n = v.size();
    if(n == 0) return;
    if(sum.size() != v.size()) sum.resize(v.size());
    sum[0] = v[0];
    for(size_t i = 1; i < n; i++) {
        sum[i] = sum[i-1] + v[i];
    }
}

/**
 * Compute a square summation
 * @param v input
 * @return square sum of all value in v
 */
inline LB_FLOAT lb_square_sum(const std::vector<LB_FLOAT>& v) {
    size_t n = v.size();
    LB_FLOAT sum = 0;
    for(size_t i = 0; i < n; i++) {
        sum += LB_SQR(v[i]);
    }
    return sum;
}

inline LB_FLOAT lb_mean(const std::vector<LB_FLOAT>& v) {
    size_t n = v.size();
    if(n == 0) return 0;
    LB_FLOAT sum = 0;
    for(size_t i = 0; i < n; i++) {
        sum += v[i];
    }
    return sum / n;
}


inline LB_FLOAT lb_stdev(const std::vector<LB_FLOAT>& v, LB_FLOAT m) {
    size_t n = v.size();
    if(n < 2) return 0;
    LB_FLOAT sum_rms_err = 0;
    for(size_t i = 0; i < n; i++) {
        sum_rms_err += LB_SQR((v[i] - m));
    }
    return sqrt(sum_rms_err / (n - 1));
}

}

#endif /* LB_STATISTIC_FUNCTION_H_ */
