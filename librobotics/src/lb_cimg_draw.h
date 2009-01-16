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
 *  OTHER DEALINGS IN THE SOFTWARE.*
 */

#ifndef LB_CIMG_DRAW_H_
#define LB_CIMG_DRAW_H_

#include "lb_common.h"
#include "lb_data_type.h"

namespace librobotics {

#if (librobotics_use_cimg == 1)

     void draw_lrf_point_cimg(cimg8u& img,
                              const std::vector<vec2f>& lrf_point,
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

         int dimx = img.dimx();
         int dimy = img.dimy();
         if(x_offset == -1) x_offset = dimx >> 1;
         if(y_offset == -1) y_offset = dimy >> 1;

         vec2f tmp;
         CImgList<int> points;

         for(size_t i = 0; i < lrf_point.size(); i++) {
             tmp = lrf_point[i];
             if(angle != 0.0)
                 tmp.rotate(angle);

             if(flip_x) tmp.x = -tmp.x;
             if(flip_y) tmp.y = -tmp.y;

             if(scale != 1.0) {
                 tmp.x *= scale;
                 tmp.y *= scale;
             }

             tmp.x += x_offset;
             tmp.y += y_offset;

             points.push_back(CImg<>::vector(tmp.x, tmp.y));
         }

         if(line) {
             img.draw_line(points, color);
         } else {
             img.draw_point(points, color);
         }
     }



#endif


}


#endif /* LB_CIMG_DRAW_H_ */
