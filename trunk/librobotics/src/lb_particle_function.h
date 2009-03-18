/*
 * lb_particle_function.h
 *
 *  Created on: Feb 2, 2009
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

#ifndef LB_PARTICLE_FUNCTION_H_
#define LB_PARTICLE_FUNCTION_H_

#include "lb_common.h"
#include "lb_exception.h"
#include "lb_data_type.h"


namespace librobotics {

template<typename T>
inline size_t lb_particle_max_weight_index(std::vector<T>& p) {
    LB_FLOAT max_w = -1;
    size_t index = 0;
    for(size_t i = 0; i < p.size(); i++ ) {
        if(p[i].w > max_w) {
            index = i;
            max_w = p[i].w;
        }
    }
    return index;
}

template<typename T>
bool lb_particle_weight_compare(const T& p1, const T& p2) {
   return (p1.w < p2.w);
}

template<typename T>
void lb_particle_normalize_weight(std::vector<T>& p) {
    LB_FLOAT sum = 0;
    for(size_t i = 0; i < p.size(); i++ ) {
        sum += p[i].w;
    }
    for(size_t i = 0; i < p.size(); i++ ) {
        p[i].w /= sum;
    }
}

template<typename T>
bool lb_particle_stratified_resample(std::vector<T>& p, int n_min) {
    lb_particle_normalize_weight(p);
    size_t n = p.size();

    if(n < 2)
        return false;

    LB_FLOAT square_sum = 0;

    //square sum
    for(size_t i = 0; i < n; i++) {
        square_sum += LB_SQR(p[i].w);
    }

    int n_eff = (int)(1.0/square_sum);
    if(n_eff > n_min)
        return false;

    std::vector<LB_FLOAT> cum_sum_w(n, 0.0);
    std::vector<int> keep(n, 0);
    std::vector<LB_FLOAT> select(n, 0.0);

    //cumulative sum
    cum_sum_w[0] = p[0].w;
    for(size_t i = 1; i < n; i++) {
        cum_sum_w[i] = cum_sum_w[i-1] + p[i].w;
    }

    lb_stratified_random(select, n);
    size_t ctr = 0;
    for(size_t i = 0; i < n; i++) {
        while((ctr < n) && (select[ctr] < cum_sum_w[i])) {
            keep[ctr] = i;
            ctr++;
        }
    }

    std::vector<T> p_tmp(n);
    for(size_t i = 0; i < n; i++) {
        p_tmp[i] = p[keep[i]];
    }

    for(size_t i = 0; i < n; i++) {
        p[i] = p_tmp[i];
        p[i].w = 1.0/n;
    }
    return true;
}

/**
 * A Function for initialize 2D position of all particle to a selected mode
 * @param p std::vector<> of particle
 * @param map
 * @param mode
 * @param start
 * @param param0
 * @param param1
 */

template<typename P, typename M>
void lb_init_particle2(std::vector<P>& p,
                       const M& map,
                       int mode,
                       pose2f start,
                       LB_FLOAT param0,
                       LB_FLOAT param1)
{
    size_t i = 0;
    switch(mode) {
        case 0: //all at start point
            for(i = 0; i < p.size(); i++)
                p[i].p = start;
            break;
        case 1: //case 0 with uniform random angle
            for(i = 0; i < p.size(); i++) {
                p[i].p.x = start.x;
                p[i].p.y = start.y;
                p[i].p.a = lb_crand() * M_PI;
            }
           break;
       case 2: //all in circle uniform random  at start point with start angle
       {
           LB_FLOAT r, a;
           for(i = 0; i < p.size(); i++) {
               lb_sample_circle_uniform_dist(a, r);
               p[i].p.x = start.x + (r * param0 * cos(a));
               p[i].p.y = start.y + (r * param0 * sin(a));
               p[i].p.a = start.a;
           }
           break;
       }
       case 3: //case 2 with uniform random angle
       {
           LB_FLOAT r, a;
           for(i = 0; i < p.size(); i++) {
               lb_sample_circle_uniform_dist(a, r);
               p[i].p.x = start.x + (r * param0 * cos(a));
               p[i].p.y = start.y + (r * param0 * sin(a));
               p[i].p.a = lb_crand() * M_PI;
           }
           break;
       }
       case 4: //case 2 with normal distribution random angle around start angle
       {
           LB_FLOAT r, a;
           for(i = 0; i < p.size(); i++) {
               lb_sample_circle_uniform_dist(a, r);
               p[i].p.x = start.x + (r * param0 * cos(a));
               p[i].p.y = start.y + (r * param0 * sin(a));
               p[i].p.a = lb_normalize_angle(start.a + lb_sample_normal_dist(param1));
           }
           break;
       }
       case 5: //all uniform random over map with start angle
       {
           vec2f pts;
           for(i = 0; i < p.size(); i++) {
               map.get_random_pts(pts);
               p[i].p.x = pts.x;
               p[i].p.y = pts.y;
               p[i].p.a = start.a;
           }
           break;
       }
       case 6: //case 5 with uniform random angle
       {
           vec2f pts;
           for(i = 0; i < p.size(); i++) {
               map.get_random_pts(pts);
               p[i].p.x = pts.x;
               p[i].p.y = pts.y;
               p[i].p.a = lb_crand() * M_PI;
           }
           break;
       }
       case 7: //case 5 with normal distribution random angle around start angle
       {
           vec2f pts;
           for(i = 0; i < p.size(); i++) {
               map.get_random_pts(pts);
               p[i].p.x = pts.x;
               p[i].p.y = pts.y;
               p[i].p.a = lb_normalize_angle(start.a + lb_sample_normal_dist(param1));
           }
           break;
       }
       case 8:
       default:
       {
           throw LibRoboticsWarningException("This case are not yet implement in %s", __FUNCTION__);
           break;
       }
   }//switch

   //initialize weight
   for(i = 0; i < p.size(); i++) {
       p[i].w = 1.0/p.size();
   }

}


}


#endif /* LB_PARTICLE_FUNCTION_H_ */
