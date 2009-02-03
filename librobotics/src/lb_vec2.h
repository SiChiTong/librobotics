/*
 * lb_vec2.h
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

#ifndef LB_VEC2_H_
#define LB_VEC2_H_

#include "lb_common.h"
#include "lb_macro_function.h"

namespace librobotics {

    /**
     * Template class for 2D vector
     */
    template<typename T>
    struct vec2 {
        T x, y;

        ///Constructor
        vec2() :
            x(0), y(0) {
        }

        ///Constructor
        template<typename T1>
        vec2(const T1 xx, const T1 yy) :
            x(xx), y(yy) {
        }

        ///Copy constructor
        template<typename T1>
        vec2(const vec2<T1>& v) :
            x((T) v.x), y((T) v.y) {
        }

        ///Unary minus
        vec2 operator -() const {
            return vec2(-x, -y);
        }

        ///Addition for vector/vector
        template<typename T1>
        vec2 operator +(const vec2<T1>& v) const {
            return vec2(x + (T) v.x, y + (T) v.y);
        }

        ///Subtraction for vector/vector
        template<typename T1>
        vec2 operator -(const vec2<T1>& v) const {
            return vec2(x - (T) v.x, y - (T) v.y);
        }

        ///Basic assignment for vector/vector
        template<typename T1>
        vec2& operator =(const vec2<T1>& v) {
            x = v.x;
            y = v.y;
            return *this;
        }

        ///Assignment by addition for vector/vector
        template<typename T1>
        vec2& operator +=(const vec2<T1>& v) {
            x += v.x;
            y += v.y;
            return *this;
        }

        ///Assignment by subtraction for vector/vector
        template<typename T1>
        vec2& operator -=(const vec2<T1>& v) {
            x -= v.x;
            y -= v.y;
            return *this;
        }

        ///Dot product between two vectors
        template<typename T1>
        T operator ^(const vec2<T1>& v) const {
            return (T) ((x * v.x) + (y * v.y));
        }

        ///Cross product between two vectors
        template<typename T1>
        T operator *(const vec2<T1>& v) const {
            return (T) ((x * v.y) - (y * v.x));
        }

        ///Multiply by scalar value
        template<typename T1>
        vec2<T1> operator *(const T1& s) const {
            return vec2<T1> (x * s, y * s);
        }

        ///Divide by scalar value
        template<typename T1>
        vec2<T1> operator /(const T1& s) const {
            return vec2<T1> (x / s, y / s);
        }

        ///Assignment by multiply with scalar value
        template<typename T1>
        vec2& operator *=(const T1& s) {
            x *= s;
            y *= s;
            return (*this);
        }

        ///Assignment by divide with scalar value
        template<typename T1>
        vec2& operator /=(const T1& s) {
            x /= s;
            y /= s;
            return (*this);
        }

        ///Get size
        T size() const {
            return (T) sqrt(x * x + y * y);
        }

        ///Get square size
        T sqr_size() const {
            return (T) (x * x + y * y);
        }

        ///Get angle in \f$(-\pi, \pi)\f$ radian range
        T theta() const {
            return (T) atan2(y, x);
        }

        ///Get angle in \f$(0, 2\pi)\f$ radian range
        T theta_2PI() const {
            T tmp = atan2(y, x);
            if (tmp < 0)
                tmp += 2 * M_PI;
            return tmp;
        }

        ///Get angle in \f$(-180, 180)\f$ degree range
        T degree() const {
            return (T) LB_RAD2DEG(atan2(y, x));
        }

        ///Get angle in \f$(0, 360)\f$ degree range
        T degree_360() const {
            T tmp = atan2(y, x);
            if (tmp < 0)
                tmp += 2 * M_PI;
            return LB_RAD2DEG(tmp);
        }

        ///Check for zero size vector
        bool is_zero() const {
            return ((x == 0) && (y == 0));
        }

        ///Get normalized vector
        vec2 get_normalize() const {
            return vec2((*this) / size());
        }

        ///Normalize the vector
        vec2& normalize() {
            return (*this) / size();
        }

        /**
         * Get rotated vector
         * \param angle rotate angle
         * \param angle unit select (true for radian, false for degree)
         */
        vec2 get_rotate(const LB_FLOAT angle, const  bool rad = true) const {
            LB_FLOAT tmp = angle;
            if (!rad)
                tmp = LB_DEG2RAD(angle);
            T c = cos(tmp);
            T s = sin(tmp);
            return vec2((T) (x * c - y * s), (T) (x * s + y * c));
        }

        /**
         * Rotate the vector
         * \param angle rotate angle
         * \param angle unit select (true for radian, false for degree)
         */
        vec2& rotate(const LB_FLOAT angle, const bool rad = true) {
            LB_FLOAT a = angle;
            if (!rad)
                a = LB_DEG2RAD(angle);

            T c = cos(angle);
            T s = sin(angle);
            T tmpx = (x * c - y * s);
            y = (x * s + y * c);
            x = tmpx;
            return *this;
        }

        ///Support for output stream operator
        friend std::ostream& operator <<(std::ostream& os, const vec2<T>& v) {
            return os << v.x << " " << v.y;
        }

        ///Support for input stream operator
        friend std::istream& operator >>(std::istream& is, vec2<T>& v) {
            is >> v.x >> v.y;
            return is;
        }
    };

    template<typename T>
    bool vec2_angle_compare( const vec2<T>& i, const vec2<T>& j) {
        double ai = atan2(i.y, i.x);
        double aj = atan2(j.y, j.x);
        return ai < aj;
    }

    template<typename T>
    bool vec2_size_compare( const vec2<T>& i, const vec2<T>& j) {
        double si = i.size();
        double sj = j.size();
        return si < sj;
    }

    template<typename T>
    vec2<T> vec2_average(const std::vector<vec2<T> >& v) {
        size_t n = v.size();
        if(n == 0) return vec2<T>();
        if(n == 1) return v[0];
        T x = 0;
        T y = 0;
        for(size_t i = 0; i <  n; i++) {
            x += v[i].x;
            y += v[i].y;
        }
        return vec2<T>(x/n, y/n);
    }

}


#endif /* LB_VEC2_H_ */
