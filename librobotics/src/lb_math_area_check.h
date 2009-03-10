/*
 * lb_math_area_check.h
 *
 *  Created on: Feb 25, 2009
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

#ifndef LB_MATH_AREA_CHECK_H_
#define LB_MATH_AREA_CHECK_H_

#include "lb_common.h"
#include "lb_data_type.h"

namespace librobotics {

//% [ inside ] = inpoly(polygon, point)
//%
//% Determines whether a given point is inside a polygon.
//% The polygon is determined by point pairs on its
//% boundary.  There is no error checking, and to be safe
//% only use convex polygons.
//%
//% input:
//%       polygon: column vector of (x,y) pairs representing the vertex
//%                points of the polygon.
//%       point: (x,y) pair representing point that is either
//%              in or out of the polygon
//%
//% output:
//%       inside: boolean true for inside, false otherwise.
//%
//% todo:
//%       Implement the ability to handle an array of points
//%       and return an array of boolean values.
//%
//% bugs and limitations:
//%
//%         Need to check whether convexity is a necessary
//%         condition for this algorithm (don't think it is).
//%
//%         Behavior with multiple loops, i.e., if sides cross
//%         over, is unknown.
//%
//%         Algorithm implemented in integer, first quadrant
//%         numbers.  Need to check generality of algorithm.
//%
//%         Points on the boundaries of polygons may not be handled
//%         correctly or consistently.
//
//%
//%         This code was derived from a public domain code
//%         copyright 1995-1996 Galacticomm, Inc., modifications
//%         allowed for any purpose provided redistribution is
//%         not restricted
//%
//% This script properly belongs in octave/compgeom/
//% (computational geometry).
//%
//% This modified code is copyright David M. Doolin and placed in
//% the public domain, with the exception that redistribution
//% is not restricted in accordance with the copyright of unmodified
//% (original) C code.
//%
//% $Author: doolin $  doolin at ce dot berkeley dot edu
//% $Date: 1999/03/26 18:42:00 $
//% $Source: /shag/homes/doolin/cvsroot/octave/compgeom/inpoly.m,v $
//% $Revision: 1.2 $
//
//
//function [ inside ] = inpoly(polygon, point)
//
//% Check for the correct number of arguments
//
//% Check for argument validation
//% If argument validation required, validate arguments.
//
//npoints = rows(polygon);
//inside = 0;
//
//xt = point(1);
//yt = point(2);
//
//xold = polygon(npoints,1);
//yold = polygon(npoints,2);
//
//for i = 1:1:npoints
//
//   xnew = polygon(i,1);
//   ynew = polygon(i,2);
//
//      if (xnew > xold)
//         x1=xold;
//         x2=xnew;
//         y1=yold;
//         y2=ynew;
//      else
//         x1=xnew;
//         x2=xold;
//         y1=ynew;
//         y2=yold;
//       endif
//
//      if ((xnew < xt) == (xt <= xold)         %/* edge "open" at left end */
//           && (yt-y1)*(x2-x1) < (y2-y1)*(xt-x1) )
//               inside=!inside;
//      endif
//
//      xold=xnew;
//      yold=ynew;
//
//end  % for loop


inline bool lb_point_is_inside_polygon(const std::vector<vec2f>& polygon,
                                       const vec2f& point)
{
#warning "NOT TEST YET"
    LB_FLOAT xt = point.x;
    LB_FLOAT yt = point.y;

    size_t n_point = polygon.size();
    if(n_point < 3) return false;

    bool inside = false;

    LB_FLOAT xold = polygon[n_point-1].x;
    LB_FLOAT yold = polygon[n_point-1].y;

    LB_FLOAT xnew, ynew, x1, y1, x2, y2;
    for(size_t i = 0; i < polygon.size(); i++) {
        xnew = polygon[i].x;
        ynew = polygon[i].y;

        if(xnew > xold) {
            x1 = xold;
            x2 = xnew;
            y1 = yold;
            y2 = ynew;
        } else {
            x1 = xnew;
            x2 = xold;
            y1 = ynew;
            y2 = yold;
        }

        if(((xnew < xt) == (xt <= xold)) && (((yt-y1)*(x2-x1)) < ((y2-y1)*(xt-x1))))
            inside=!inside;
        xold=xnew;
        yold=ynew;
    }
    return inside;
}

}

#endif /* LB_MATH_AREA_CHECK_H_ */
