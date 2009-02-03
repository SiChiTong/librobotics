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

static const unsigned char
    red[] = {255, 0, 0}, green[] = {0, 255, 0}, blue[] = {0, 0, 255},
    yellow[] = {255, 255, 0}, margenta[] = {255, 0, 255}, cyan[] = {0, 255, 255},
    black[] = {0, 0 ,0}, gray[] = {127, 127, 127},
    red2[] = {128, 0, 0}, green2[] = {0, 128, 0}, blue2[] = {0, 0, 128},
    yellow2[] = {128, 128, 0}, margenta2[] = {128, 0, 128}, cyan2[] = {0, 128, 128},
    orage[] = {255, 128, 0 },


    white[] = {255, 255, 255};


inline void lb_draw_points_cimg(cimg8u& img,
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


inline void lb_draw_lrf_line_object_cimg(cimg8u& img,
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

inline void lb_draw_lrf_arc_object_cimg(cimg8u& img,
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


inline void lb_draw_vfh_cimg(cimg8u& img,
                             const std::vector<LB_FLOAT>& histogram,
                             const unsigned char color[],
                             LB_FLOAT scale = 1.0,
                             LB_FLOAT angle = 0.0,
                             int x_offset = -1,
                             int y_offset = -1,
                             bool draw_line = false,
                             bool flip_x = false,
                             bool flip_y = true)
{
    using namespace cimg_library;

    lb_cimg_draw_offset_data();

    CImgList<int> points;
    LB_FLOAT theta_step = (2*M_PI)/histogram.size();
    vec2f tmp;

    for(size_t i = 0; i < histogram.size(); i++) {
        tmp = vec2f(1,0).get_rotate(theta_step*i) * histogram[i];
        if(angle != 0)
            tmp = tmp.get_rotate(angle);

        lb_cimg_draw_check(tmp);

        img.draw_line(x_offset, y_offset, (int)tmp.x, (int)tmp.y, color, 0.4f);

        if(draw_line)
            points.push_back(CImg<int>::vector(tmp.x, tmp.y));
    }
    if(draw_line) {
        img.draw_line(points, color, 0.8f);
    }
}

template<typename P, typename M>
inline void lb_draw_paticle2(cimg8u& img,
                             const std::vector<P>& p,
                             const M& map,
                             const unsigned char color[],
                             int size_in_pixel,
                             LB_FLOAT scale = 1.0,
                             LB_FLOAT angle = 0.0,
                             int x_offset = -1,
                             int y_offset = -1,
                             bool draw_dir = false,
                             bool flip_x = false,
                             bool flip_y = true )
{
    lb_cimg_draw_offset_data();

    vec2i grid_pose;
    vec2f tmp, dir;
    for(size_t i = 0; i < p.size(); i++) {
        map.get_grid_coordinate(p[i].p.x, p[i].p.y, grid_pose);

        tmp = grid_pose * scale;
        if(angle != 0)
            tmp = tmp.get_rotate(angle);

        if(flip_x) tmp.x = dimx - tmp.x;
        if(flip_y) tmp.y = dimy - tmp.y;


        img.draw_circle(tmp.x + x_offset, tmp.y + y_offset, size_in_pixel*scale, color);

        if(draw_dir) {
            dir = vec2f(size_in_pixel * 2.0, 0.0).get_rotate(p[i].p.a);
            if(angle != 0)
                dir = dir.get_rotate(angle);
            lb_cimg_draw_check(dir);
            dir += tmp;
            img.draw_line(tmp.x, tmp.y, dir.x, dir.y, black);
        }
    }
}

template<typename M>
inline void lb_draw_map2_grid_ray_cast(cimg8u& img,
                                       const vec2i& grid,
                                       const M& map,
                                       const unsigned char color[],
                                       LB_FLOAT scale = 1.0,
                                       LB_FLOAT angle = 0.0,
                                       int x_offset = -1,
                                       int y_offset = -1,
                                       bool draw_dir = false,
                                       bool flip_x = false,
                                       bool flip_y = true )
{
    if(!map.is_inside(grid.x, grid.y)) return;

    lb_cimg_draw_offset_data();

    LB_FLOAT r, rad;
    vec2f tmp;
    vec2i tmp2;
    for(size_t i = 0; i < map.ray_casting_cache[grid.x][grid.y].size(); i++) {
        r = (map.ray_casting_cache[grid.x][grid.y][i]/map.resolution);
        rad = map.angle_res * i;
        tmp.x = r * cos(rad);
        tmp.y = r * sin(rad);

        if(angle != 0)
            tmp = tmp.get_rotate(angle);
        lb_cimg_draw_check(tmp);

        tmp2 = grid * scale;
        if(flip_x) tmp2.x = dimx - tmp2.x;
        if(flip_y) tmp2.y = dimy - tmp2.y;

        tmp += tmp2;
        img.draw_line(tmp2.x, tmp2.y, tmp.x, tmp.y, color, 0.8);
    }

}

#endif


}


#endif /* LB_CIMG_DRAW_H_ */
