/*
 * lb_data_type.h
 *
 *  Created on: Jan 15, 2009
 *      Author: mahisorn
 */

#ifndef LB_DATA_TYPE_H_
#define LB_DATA_TYPE_H_

#include "lb_common.h"
#include "lb_vec2.h"
#include "lb_pose2.h"

namespace librobotics {
    //vector
    typedef vec2<int> vec2i;
    typedef vec2<LB_FLOAT> vec2f;

    //position
    typedef pose2<LB_FLOAT> pose2f;

    //bounding box
    template<typename T>
    struct bbox2 {
        vec2<T> min, max;
    };
    typedef bbox2<int> bbox2i;
    typedef bbox2<LB_FLOAT> bbox2f;

    //range measurement
    struct range_data {
        std::vector<int> range;
        std::vector<LB_FLOAT> angle;
        std::vector<vec2f> point;
    };



}

#endif /* LB_DATA_TYPE_H_ */
