/*
 * lb_cimg_draw.h
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

#ifndef LB_CIMG_DRAW_H_
#define LB_CIMG_DRAW_H_

#include "lb_common.h"
#include "lb_data_type.h"

#define lb_cimg_draw_offset_data( ) \
    int dimx = img.dimx(); \
    int dimy = img.dimy(); \
    if (x_offset == -1) \
        x_offset = dimx >> 1; \
    if (y_offset == -1) \
        y_offset = dimy >> 1;


#define lb_cimg_draw_check(a) \
    if (flip_x) a.x = -a.x; \
    if (flip_y) a.y = -a.y; \
    if (scale != 1.0) { \
        a.x *= scale; \
        a.y *= scale; \
    } \
    a.x += x_offset; \
    a.y += y_offset;

#define lb_cimg_draw_check2(a, b) \
    if (flip_x) { \
        a.x = -a.x; \
        b.x = -b.x; \
    } \
    if (flip_y) { \
        a.y = -a.y; \
        b.y = -b.y; \
    } \
    if (scale != 1.0) { \
        a.x *= scale; \
        a.y *= scale; \
        b.x *= scale; \
        b.y *= scale; \
    } \
    a.x += x_offset; \
    a.y += y_offset; \
    b.x += x_offset; \
    b.y += y_offset;


namespace librobotics {

#if (librobotics_use_cimg == 1)

inline void draw_points_cimg(cimg8u& img,
                             const std::vector<vec2f>& points,
                             const unsigned char color[],
                             LB_FLOAT scale = 1.0,
                             LB_FLOAT angle = 0.0,
                             int x_offset = -1,
                             int y_offset = -1,
                             bool line = false,
                             bool flip_x = false,
                             bool flip_y = true)
{
    using namespace cimg_library;

    lb_cimg_draw_offset_data();

    vec2f tmp;
    CImgList<int> cimg_points;

    for (size_t i = 0; i < points.size(); i++) {
        tmp = points[i];
        if (angle != 0.0)
            tmp.rotate(angle);

        lb_cimg_draw_check(tmp);

        cimg_points.push_back(CImg<>::vector(tmp.x, tmp.y));
    }

    if (line) {
        img.draw_line(cimg_points, color);
    } else {
        img.draw_point(cimg_points, color);
    }
}


inline void draw_lrf_line_object_cimg(cimg8u& img,
                                      const lrf_object& line,
                                      const unsigned char color[],
                                      LB_FLOAT scale = 1.0,
                                      LB_FLOAT angle = 0.0,
                                      int x_offset = -1,
                                      int y_offset = -1,
                                      bool flip_x = false,
                                      bool flip_y = true)
{
    using namespace cimg_library;

    lb_cimg_draw_offset_data();

    vec2f p0 = line.points[0];
    vec2f p1 = line.points[line.points.size() - 1];
    LB_FLOAT m = line.extra_param[0];
    LB_FLOAT c = line.extra_param[1];
    vec2f l0 = p0;
    vec2f l1 = p1;

    if(fabs(m) > 1.0) {
        l0.x = (l0.y - c)/m;
        l1.x = (l1.y - c)/m;
    } else {
        l0.y = m*l0.x + c;
        l1.y = m*l1.x + c;
    }

    if(angle != 0.0) {
        l0.rotate(angle);
        l1.rotate(angle);
    }

    lb_cimg_draw_check2(l0, l1);
    img.draw_line(l0.x, l0.y, l1.x, l1.y, color, 1.0);
}

inline void draw_lrf_arc_object_cimg(cimg8u& img,
                                     const lrf_object& arc,
                                     const unsigned char color[],
                                     LB_FLOAT scale = 1.0,
                                     LB_FLOAT angle = 0.0,
                                     int x_offset = -1,
                                     int y_offset = -1,
                                     bool flip_x = false,
                                     bool flip_y = true)
{
    using namespace cimg_library;

    lb_cimg_draw_offset_data();

    vec2f c = arc.extra_point[0];
    LB_FLOAT r = arc.extra_param[0];

    if(angle != 0.0) {
        c.rotate(angle);
    }

    lb_cimg_draw_check(c);
    r *= scale;
    img.draw_circle(c.x, c.y, r, color, 1.0, 5);
}


#endif


}


#endif /* LB_CIMG_DRAW_H_ */
