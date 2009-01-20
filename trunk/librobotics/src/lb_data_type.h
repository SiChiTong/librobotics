/*
 * lb_data_type.h
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

#ifndef LB_DATA_TYPE_H_
#define LB_DATA_TYPE_H_

#include "lb_common.h"
#include "lb_vec2.h"
#include "lb_pose2.h"

namespace librobotics {
    //vector
    typedef vec2<LB_INT> vec2i;
    typedef vec2<LB_FLOAT> vec2f;

    //position
    typedef pose2<LB_FLOAT> pose2f;

    //bounding box
    template<typename T>
    struct bbox2 {
        vec2<T> min, max;
    };
    typedef bbox2<LB_INT> bbox2i;
    typedef bbox2<LB_FLOAT> bbox2f;

    //Laser range finder
    enum lrf_range_condition {
        LRF_COND_NONE        = 0x00,
        LRF_COND_FAR         = 0x01,     //too far
        LRF_COND_MOVE        = 0x02,     //moving object
        LRF_COND_MIXED       = 0x04,     //mixed pixel
        LRF_COND_OCCLUDED    = 0x08,     //occluded
        LRF_COND_EMPTY       = 0x10      //no measurement
    };

    enum lrf_object_type {
        LRF_OBJ_SEGMENT     = 0x00,
        LRF_OBJ_LINE        = 0x01,
        LRF_OBJ_ARC         = 0x02,
        LRF_OBJ_CORNER      = 0x04,
        LRF_OBJ_SMALL_OBJ   = 0x08,
        LRF_OBJ_LEG         = 0x10,
        LRF_OBJ_LEG2        = 0x20,
        LRF_OBJ_HUMAN       = 0x40,
    };

    //laser segment
    struct lrf_segment {
        std::vector<LB_FLOAT> ranges;
        std::vector<vec2f> points;
    };

    //laser object
    struct lrf_object {
        std::vector<vec2f> points;
        lrf_object_type type;
        vec2f extra_point[3];
        LB_FLOAT extra_param[3];
        int  segment_id;

        lrf_object() :
            type(LRF_OBJ_SEGMENT),
            segment_id(-1)
        { }
    };



}

#endif /* LB_DATA_TYPE_H_ */
