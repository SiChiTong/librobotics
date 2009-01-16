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
 *  OTHER DEALINGS IN THE SOFTWARE.*
 */

#ifndef LB_LRF_BASIC_H_
#define LB_LRF_BASIC_H_

#include "lb_common.h"
#include "lb_macro_function.h"
#include "lb_misc_function.h"
#include "lb_data_type.h"

namespace librobotics {

    void lrf_scan_point_from_scan_range(const std::vector<int>& ranges,
                                        const std::vector<LB_FLOAT>& cos_table,
                                        const std::vector<LB_FLOAT>& sin_table,
                                        std::vector<vec2f>& scan_point,
                                        int start,
                                        int end,
                                        int cluster,
                                        LB_FLOAT scale = 1.0,
                                        bool flip = false)
    {
        size_t n = ranges.size();
        if(scan_point.size() != n) scan_point.resize(n);

        if((cos_table.size() <= (size_t)end) || (sin_table.size() <= (size_t)end)) {
            throw librobotics::LibRoboticsRuntimeException("cos/sin table size are not correct");
        }

        LB_FLOAT r = 0;
        int idx = 0;
        for(size_t i = 0; i < n; i++) {
            //get angle index
            idx = start + (i*cluster) + (cluster >> 1);
            r = ranges[i] * scale;
            if(flip){
                scan_point[(n - 1) - i].x = r * cos_table[idx];
                scan_point[(n - 1) - i].y = -r * sin_table[idx];
            } else {
                scan_point[i].x = r * cos_table[idx];
                scan_point[i].y = r * sin_table[idx];
            }
        }
    }

}


#endif /* LB_LRF_BASIC_H_ */
