/*
 * lb_vfh.h
 *
 *  Created on: Jan 29, 2009
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

#ifndef LB_VFH_H_
#define LB_VFH_H_

#include "lb_common.h"
#include "lb_exception.h"
#include "lb_data_type.h"

/*-------------------------------------------------------------
 *
 * Definition of the LibRobotics: Vector Field Histogram (VFH) obstacle avoidance
 * (http://www-personal.umich.edu/~johannb/vff&vfh.htm)
 *
 *-------------------------------------------------------------*/

namespace librobotics {

inline void lb_vfh_compute(std::vector<LB_FLOAT>& h,
                           const std::vector<vec2f>& obs,
                           const LB_FLOAT A,
                           const LB_FLOAT min_range,
                           const LB_FLOAT max_range,
                           const LB_FLOAT angle_res_in_degree)
{
    size_t step = (size_t)(360.0 / angle_res_in_degree) + 1;
    if(step != h.size()) h.resize(step);
    for(size_t i = 0; i < step; i++) {
        h[i] = 0;
    }

    size_t idx = 0;
    LB_FLOAT d = 0.0;
    LB_FLOAT m = 0.0;
    LB_FLOAT B = A / max_range;
    for(size_t i = 0; i < obs.size(); i++) {
        idx = (int)(obs[i].degree_360() / angle_res_in_degree);
        d = obs[i].size();
        if((d >= min_range) && (d <= max_range)) {
            m = (A - (B * d));
            h[idx] += m;
        }
    }
}


inline void lb_vfh_smooth(const std::vector<LB_FLOAT>& h,
                          const LB_FLOAT threshold,
                          const int smooth_wnd_size,
                          std::vector<LB_FLOAT>& hs)
{
    size_t n = h.size();
    if(hs.size() != n) hs.resize(n);

    int idx;
    LB_FLOAT sum = 0;
    for(int i = 0; i < (int)n; i++) {
        sum = 0;
        for(int l = -smooth_wnd_size; l <= smooth_wnd_size; l++) {
            idx = i + l;
            if(idx < 0) {
                idx = n + l;
            } else if(idx > (int)(n - 1)) {
                idx = l;
            }

            if(l < 0)
                sum += h[idx] * ((smooth_wnd_size + 1) + l);
            else
                sum += h[idx] * ((smooth_wnd_size + 1) - l);
        }
        sum = sum / (2*smooth_wnd_size + 1);

        if(sum >= threshold)
            hs[i] = sum;
        else
            hs[i] = 0.f;
    }
}

inline void lb_vfh_set_value(std::vector<LB_FLOAT>& hs,
                             const LB_FLOAT new_value,
                             const LB_FLOAT angle_res_in_degree,
                             const LB_FLOAT set_angle_in_degree,
                             const LB_FLOAT set_angle_half_side_in_degree)
{
    int idx = set_angle_in_degree / angle_res_in_degree;
    int wnd = set_angle_half_side_in_degree / angle_res_in_degree;
    int n = (int)hs.size();

    int j = 0;
    for(int i = -wnd; i <= wnd; i++) {
        j = idx + i;
        if(j < 0) {
            //under flow
            j = n + i;
        } else if(idx > (n - 1)) {
            //over flow
            j = i;
        }
        hs[j] = new_value;
    }

}


inline bool lb_vfh_find_open_angle(const vec2f& target,
                                   const std::vector<LB_FLOAT>& hs,
                                   const LB_FLOAT threshold,
                                   int open_segment,
                                   const LB_FLOAT angle_res,
                                   vec2f& result)
{
    size_t zero_cnt = 0;
    for(size_t i = 0; i < hs.size(); i++) {
        if(hs[i] < threshold) {
            zero_cnt++;
        }
    }

    //no obstacle, directly move to the target
    if(zero_cnt == hs.size()) {
        result =  target;
        return true;
    }

    //extract open segment
    std::vector<int> k_begin;
    std::vector<int> k_end;
    int count = 0;
    bool is_begin = false;
    int start_step;
    for(size_t i = 0; i < hs.size(); i++) {
        if(hs[i] < threshold) {
            if(is_begin) {
                count++;
            } else {
                is_begin = true;
                count = 0;
                start_step = i;
            }
        } else {
            if(is_begin) {
                k_begin.push_back(start_step);
                k_end.push_back(start_step + count);
                is_begin = false;
                count = 0;
            }
        }

        //last segment
        if(is_begin && (i == (hs.size() - 1))) {
            k_begin.push_back(start_step);
            k_end.push_back(start_step + count);
        }
    }

    //check first and last segment
    if((k_begin[0] == 0) && (k_end[k_end.size() - 1] == (int)(hs.size() - 1))) {
        //this is the same open segment --> merge them
        k_end[k_end.size() - 1] = k_end[0];

        //remove first segment
        k_begin.erase(k_begin.begin());
        k_end.erase(k_end.begin());
    }

    int center;
    int good_index = -1;
    vec2f good_dir;
    LB_FLOAT d_theta = (std::numeric_limits<LB_FLOAT>::max)();
    int  num_subseg = 0;
    bool found = false;
    int tmp1, tmp2, tmp_center;

    for(size_t i = 0; i < k_begin.size(); i++) {
        num_subseg = -1;
        if(k_end[i] >  k_begin[i]) {
            num_subseg = (k_end[i] - k_begin[i]);
            center =  (k_end[i] +  k_begin[i]) / 2;
        } else if(k_end[i] <  k_begin[i]) {
            tmp1 = hs.size() - k_begin[i];
            tmp2 = k_end[i];
            num_subseg = (tmp1 + tmp2);

            tmp_center = (tmp1 + tmp2) / 2;
            if((k_begin[i] + tmp_center) >= (int)hs.size()) {
                center = k_end[i] - tmp_center;
            } else {
                center = k_begin[i] + tmp_center;
            }
        } else {
            continue;
        }

        LB_FLOAT good_angle;
        vec2f dir;
        LB_FLOAT d, c, diff;

        if(num_subseg >= open_segment) {
            num_subseg -= open_segment;
            do {
                good_angle = (k_begin[i] + (open_segment/2) + num_subseg) * angle_res;
                good_angle = LB_DEG2RAD(good_angle);
                dir.x = cos(good_angle);
                dir.y = sin(good_angle);
                d = dir ^ target.get_normalize();
                c = dir * target.get_normalize();
                diff = fabs(atan2(c,d));
                if(diff < d_theta) {
                    good_index = i;
                    d_theta = diff;
                    good_dir = dir;
                    found = true;
                }
                num_subseg--;
            } while (num_subseg >= 0);
        } else {
            good_angle = LB_DEG2RAD(center * angle_res);
            dir.x = cos(good_angle);
            dir.y = sin(good_angle);
            d = dir ^ target.get_normalize();
            c = dir * target.get_normalize();
            diff = fabs(atan2(c,d));
            if(diff < d_theta) {
                good_index = i;
                d_theta = diff;
                good_dir = dir;
                found = true;
            }
        }//if
    }//for

    if(found)
        result = good_dir;
    else
        result = vec2f();

    return found;
}//vfh_find_open_angle

}//namespace

#endif /* LB_VFH_H_ */
