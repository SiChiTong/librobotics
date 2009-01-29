/*
 * lb_pose2.h
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

#ifndef LB_POSE2_H_
#define LB_POSE2_H_

#include "lb_common.h"
#include "lb_macro_function.h"
#include "lb_vec2.h"

namespace librobotics {
    /**
     * Template class for 2D position.
     */
    template<typename T>
    struct pose2 {
        T x;    //!< X position
        T y;    //!< Y position
        T a;    //!< direction

        ///Constructor
        pose2() : x(0), y(0), a(0)
        { }

        ///Constructor
        template<typename T1>
        pose2(const T1 xx,const T1 yy,const  T1 aa) :
            x(xx), y(yy), a(aa)
        { }

        ///Constructor
        template<typename T1>
        pose2(const vec2<T1>& p, const T1 aa) :
            x(p.x), y(p.y), a(aa)
        { }

        ///Copy constructor
        template<typename T1>
        pose2(const pose2<T1>& p2) :
            x(p2.x), y(p2.y), a(p2.a)
        { }


        ///Addition
        template<typename T1>
        pose2 operator + (const pose2<T1>& p) const {
            return pose2(x + p.x, y + p.y, normalize_angle(a + p.a));
        }

        ///Addition
        template<typename T1>
        pose2 operator + (const vec2<T1>& v) const {
            return pose2(x + v.x, y + v.y, a);
        }

        ///Subtraction
        template<typename T1>
        pose2 operator - (const pose2<T1>& p) const {
            return pose2(x - p.x, y - p.y, normalize_angle(a - p.a));
        }

        ///Subtraction
        template<typename T1>
        pose2 operator - (const vec2<T1>& v) const {
            return pose2(x - v.x, y - v.y, a);
        }

        vec2<T> get_vec2() const {
            return vec2<T>(x, y);
        }

        template<typename T1>
        vec2<T> get_vec2_to(const pose2<T1>& p) const {
            return vec2<T>(p.x - x, p.y - y);
        }

        template<typename T1>
        T dist_to(const pose2<T1>& p) {
            return sqrt(LB_SQR(p.x - x) + LB_SQR(p.y - y));
        }

        template<typename T1>
        T angle_to(const pose2<T1>& p) {
            return minimum_angle_distance(a, p.a);
        }

        /// support for output stream
        friend std::ostream& operator << (std::ostream& os, const pose2<T>& p) {
            return os << p.x << " " << p.y << " " << p.a;
        }

        /// support for input stream
        friend std::istream& operator >> (std::istream& is, pose2<T>& p) {
            is >> p.x >> p.y >> p.a;
            return is;
        }
    };
}

#endif /* LB_POSE2_H_ */
