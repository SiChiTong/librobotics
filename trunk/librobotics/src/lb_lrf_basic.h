/*
 * lb_lrf_basic.h
 *
 *  Created on: Jan 16, 2009
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

#ifndef LB_LRF_BASIC_H_
#define LB_LRF_BASIC_H_

#include "lb_common.h"
#include "lb_exception.h"
#include "lb_macro_function.h"
#include "lb_misc_function.h"
#include "lb_data_type.h"

namespace librobotics {



template<typename T>
inline void lb_lrf_get_scan_point_from_scan_range(const std::vector<T>& ranges,
                                                  const std::vector<LB_FLOAT>& cos_table,
                                                  const std::vector<LB_FLOAT>& sin_table,
                                                  std::vector<vec2f>& scan_points,
                                                  const int start,
                                                  const int end,
                                                  const int cluster,
                                                  const LB_FLOAT scale = 1.0,
                                                  const bool flip = false)
{
    size_t n = ranges.size();
    if(scan_points.size() != n) scan_points.resize(n);

    if((cos_table.size() <= (size_t)end) || (sin_table.size() <= (size_t)end)) {
        throw librobotics::LibRoboticsRuntimeException("cos/sin table size are not correct");
    }

    LB_FLOAT r = 0;
    int idx = 0;
    for(size_t i = 0; i < n; i++) {
        if(ranges[i] == 0) {
            if(flip){
                scan_points[(n - 1) - i].x = 0.0;
                scan_points[(n - 1) - i].y = 0.0;
            } else {
                scan_points[i].x = 0.0;
                scan_points[i].y = 0.0;
            }
            continue;
        }

        //get angle index
        idx = start + (i*cluster) + (cluster >> 1);
        r = ranges[i] * scale;
        if(flip){
            scan_points[(n - 1) - i].x = r * cos_table[idx];
            scan_points[(n - 1) - i].y = -r * sin_table[idx];
        } else {
            scan_points[i].x = r * cos_table[idx];
            scan_points[i].y = r * sin_table[idx];
        }
    }
}

template<typename T>
inline void lb_lrf_get_scan_range_from_scan_point(const std::vector<vec2f>& scan_points,
                                                  std::vector<T>& result,
                                                  const LB_FLOAT scale = 1.0)
{
    size_t n = scan_points.size();
    if(result.size() < n) result.resize(n);
    for(size_t i = 0; i < n; i++ ) {
        result[i] = (T)(scan_points[i].size() * scale);
    }
}

inline void lb_lrf_scan_point_offset(std::vector<vec2f>& scan_points,
                                     const pose2f& local_offset,
                                     const pose2f& global_offset,
                                     const bool ignore_zero = true)
{
    size_t n = scan_points.size();
    for(size_t i = 0; i < n; i++ ) {
        if(ignore_zero) {
            if(scan_points[i].is_zero()) {
                scan_points[i].x = 0.0;
                scan_points[i].y = 0.0;
                continue;
            }
        }

        scan_points[i].rotate(local_offset.a);
        scan_points[i] += local_offset.get_vec2();
        scan_points[i].rotate(global_offset.a);
        scan_points[i] += global_offset.get_vec2();
    }
}

inline void lb_lrf_get_scan_point_offset(const std::vector<vec2f>& scan_points,
                                         const pose2f& local_offset,
                                         const pose2f& global_offset,
                                         std::vector<vec2f>& result,
                                         const bool ignore_zero = true)
{
    size_t n = scan_points.size();
    if(result.size() < n) result.resize(n);

    for(size_t i = 0; i < n; i++ ) {
        if(ignore_zero) {
            if(scan_points[i].is_zero()) {
                result[i].x = 0.0;
                result[i].y = 0.0;
                continue;
            }
        }

        result[i] = scan_points[i];
        result[i].rotate(local_offset.a);
        result[i] += local_offset.get_vec2();
        result[i].rotate(global_offset.a);
        result[i] += global_offset.get_vec2();
    }
}

template<typename T>
inline void lb_lrf_range_condition_check(const std::vector<T>& ranges,
                                         std::vector<lrf_range_condition>& cond,
                                         const T min_range,
                                         const T max_range)
{
    size_t n = ranges.size();
    for(size_t i = 0; i < n; i++ ) {
        if(ranges[i] < min_range) {
            cond[i] |= LRF_COND_EMPTY;
        } else if(ranges[i] > max_range) {
            cond[i] |= LRF_COND_FAR;
        } else {
            cond[i] = LRF_COND_NONE;
        }
    }
}

template<typename T>
inline void lb_lrf_range_threshold_filter(std::vector<T>& ranges,
                                          const T min_range,
                                          const T new_min = 0,
                                          const T max_range = (std::numeric_limits<T>::max)(),
                                          const T new_max = 0)
{
    size_t n = ranges.size();
    for(size_t i = 0; i < n; i++ ) {
        if(ranges[i] < min_range) {
            ranges[i] = new_min;
        }
        else
        if(ranges[i] > max_range) {
            ranges[i] = new_max;
        }
    }
}

template<typename T>
inline void lb_lrf_range_median_filter(std::vector<T>& ranges,
                                       const size_t half_windows_size = 2)
{
    size_t n = ranges.size();
    if(n < ((half_windows_size * 2) + 1))
        return;

    std::vector<T> r((half_windows_size * 2) + 1, 0);
    int k = 0, l = 0;
    for(size_t i = 0; i < n; i++) {
        k = 0;
        for(int j = i - half_windows_size ; j <= (int)(i + half_windows_size); j++) {
            l = (j < 0) ? 0 : j;
            l = (j >= (int)n) ? (n - 1) : l;
            r[k] = ranges[l];
            k++;
        }
        std::sort(r.begin(), r.end());
        ranges[i] = r[half_windows_size];
    }
}

template<typename T>
inline int lb_lrf_range_segment(const std::vector<T>& ranges,
                                std::vector<int>& seg,
                                const T threshold,
                                const T min_range = 0)
{
    size_t n = ranges.size();
    if(seg.size() != n) {
        seg.resize(n, -1);
    } else {
        for(size_t i = 0; i < n; i++) {
            seg[i] = -1;
        }
    }

    bool new_segment = true;
    T last_range = 0;
    int n_segment = 0;
    T range_diff = 0;

    for(size_t i = 0; i < n; i++) {
        if(ranges[i] > min_range) {
            if(new_segment) {
                n_segment++;

                //start new segment
                seg[i] = n_segment;
                new_segment = false;
                last_range = ranges[i];
            } else {
                range_diff = ranges[i] - last_range;
                if(fabs((double)range_diff) > threshold) {
                    //end current segment
                    n_segment++;

                    //start new segment
                    seg[i] = n_segment;
                    last_range = ranges[i];
                } else {
                    //continue current segment
                    seg[i] = n_segment;
                    last_range = ranges[i];
                }
                new_segment = false;
            }
        } else {
            if(!new_segment) {
                //end last segment
                new_segment = true;
            }
            seg[i] = -1;
        }
    }

    return n_segment;
}

template<typename T>
inline int lb_lrf_point_segment(const std::vector<vec2f>& points,
                                std::vector<int>& seg,
                                const T threshold,
                                const T min_range = 0)
{
    size_t n = points.size();
    if(seg.size() != n) {
       seg.resize(n, -1);
    } else {
       for(size_t i = 0; i < n; i++) {
           seg[i] = -1;
       }
    }

    bool new_segment = true;
    vec2f last_point;
    int n_segment = 0;
    LB_FLOAT dist_diff = 0;

    for(size_t i = 0; i < n; i++) {
        if(points[i].size() > min_range) {
            if(new_segment) {
                n_segment++;

                //start new segment
                seg[i] = n_segment;
                new_segment = false;
                last_point = points[i];
            } else {
                dist_diff = (points[i] - last_point).size();
                if(dist_diff > threshold) {
                    //end current segment
                    n_segment++;

                    //start new segment
                    seg[i] = n_segment;
                    last_point = points[i];
                } else {
                    //continue current segment
                    seg[i] = n_segment;
                    last_point = points[i];
                }
                new_segment = false;
            }
        } else {
            if(!new_segment) {
                //end last segment
                new_segment = true;
            }
            seg[i] = -1;
        }
    }

    return n_segment;
}

template<typename T>
inline void lb_lrf_create_segment(const std::vector<T>& input,
                                  const std::vector<int>& seg,
                                  std::vector<std::vector<T> >& result)
{
    result.clear();

    size_t n = input.size();
    if(n == 0) return;

    if(input.size() != seg.size()) return;

    std::vector<T> tmp;
    size_t i = 0;
    int last_segment = seg[0];
    while(i < n) {
        if(seg[i] != last_segment) {
            if(seg[i] == -1) {
                //end
                if(!tmp.empty()) {
                    result.push_back(tmp);
                    tmp.clear();
                }
            } else {
                //new
                result.push_back(tmp);
                tmp.clear();

                tmp.push_back(input[i]);
            }
        } else {
            //continue
            if(seg[i] != -1) {
                tmp.push_back(input[i]);
            }
        }
        last_segment = seg[i++];
    }

    //last segment
    if(!tmp.empty()) {
        result.push_back(tmp);
    }
}


}


#endif /* LB_LRF_BASIC_H_ */
