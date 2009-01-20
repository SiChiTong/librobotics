/*
 * lb_lrf_object_detect.h
 *
 *  Created on: Jan 19, 2009
 *      Author: mahisorn
 */

#ifndef LB_LRF_OBJECT_DETECT_H_
#define LB_LRF_OBJECT_DETECT_H_

#include "lb_common.h"
#include "lb_data_type.h"
#include "lb_regression.h"
#include "lb_statistic_function.h"


namespace librobotics {


inline int lrf_recusive_line_fitting(const std::vector<vec2f>& points,
                                     std::vector<lrf_object>& objects,
                                     LB_FLOAT min_length,
                                     LB_FLOAT err_threshold,
                                     int id = -1,
                                     size_t min_point = 4)
{
    size_t n = points.size();

    //check total point
    if(n < min_point) {
        return 0;
    }

    //check length (simple)
    if((points[0] - points[n - 1]).size() < min_length) {
        return 0;
    }

    LB_FLOAT m = 0, b = 0, r = 0;
    lb_liner_regression(points, m, b, r);

    //check result
    if(m == 0 && b == 0 && r == 0) {
        return 0;
    }

    if(r < err_threshold) {
        //LB_PRINT_VAL("single line");
        LB_FLOAT theta=atan2(-1/m,1);

        if ( ((m > 0) && (b >0)) || ((m < 0) && (b < 0)) ) {
            theta =- (M_PI - theta);
        };

        LB_FLOAT rho = fabs(b/sqrt(LB_SQR(m*m)+1));
        vec2f p(rho*cos(theta), rho*sin(theta));

        //add line
        lrf_object line;
        line.points = points;
        line.type = LRF_OBJ_LINE;
        line.extra_point[0] = vec2f(cos(theta), sin(theta));    //line direction
        line.extra_point[1] = p;                                  //vector to line normal
        line.extra_param[0] = m;
        line.extra_param[1] = b;
        line.extra_param[2] = r;
        line.segment_id = id;
        objects.push_back(line);
        return 1;
    } else {
        //LB_PRINT_VAL("recursive check");
        LB_FLOAT A = 0, B = 0, C = 0, ss = 0;
        lb_line_define(points, A, B, C);
        ss = LB_SIZE(A, B);

        // search for lines break
        size_t n_break = 0;
        LB_FLOAT dist, dist_max = -1.0;
        for (size_t i = 0; i < n; i++) {
            dist = fabs( (points[i].x*A + points[i].y*B + C)/ss);
            if (dist > dist_max) {
                dist_max = dist;
                n_break = i;
            }
        }

        if (dist_max > err_threshold) {
            //First half
            std::vector<vec2f> first_beam;
            for(size_t i = 0; i < n_break; i++) {
                first_beam.push_back(points[i]);
            }
            int line1 = lrf_recusive_line_fitting(first_beam,
                                                  objects,
                                                  min_length,
                                                  err_threshold,
                                                  min_point);

            //Second half
            std::vector<vec2f> second_beam;
            for(size_t i = n_break; i < n; i++) {
                second_beam.push_back(points[i]);
            }

            int line2 = lrf_recusive_line_fitting(second_beam,
                                                  objects,
                                                  min_length,
                                                  err_threshold,
                                                  min_point);
            return line1 + line2;
        }//if
    }//if

    return 0;
}


inline bool lrf_arc_fiting(const std::vector<vec2f>& points,
                           std::vector<lrf_object>& objects,
                           LB_FLOAT min_aparture,
                           LB_FLOAT max_aparture,
                           LB_FLOAT max_stdev,
                           LB_FLOAT arc_ratio,
                           LB_FLOAT is_line_error,
                           LB_FLOAT is_line_stdev,
                           int id = -1,
                           bool check_corner = true,
                           size_t min_point = 10)
{
    size_t n = points.size();
    if(n < min_point) {
        return false;
    }

    vec2f middle = points[n >> 1];
    vec2f right = points[0];
    vec2f left = points[n - 1];
    vec2f center = (right + left) * 0.5;

    LB_PRINT_VAR(left);
    LB_PRINT_VAR(middle);
    LB_PRINT_VAR(right);

    LB_FLOAT dist_lr = (left - right).size();
    LB_FLOAT dist_mc = (middle - center).size();

    LB_PRINT_VAR(dist_lr);
    LB_PRINT_VAR(dist_mc);
    LB_PRINT_VAR(dist_mc/dist_lr);


    bool check_ratio = dist_mc > (arc_ratio * dist_lr);
    bool check_size = dist_mc < dist_lr;

    LB_PRINT_VAR(check_ratio);
    LB_PRINT_VAR(check_size);

    if(!(check_ratio && check_size)) {
        std::cout << "Not an arc" << std::endl;
        return false;
    }


    size_t slopes_size = n - 3;
    LB_FLOAT ma = 0, mb = 0;
    std::vector<LB_FLOAT>slopes(slopes_size, 0);

    // make vector of slopes
    vec2f xy_temp;
    for (size_t i = 0; i < slopes_size; i++) {
        xy_temp = points[i+1];
        ma = (left - xy_temp).theta();
        mb = (right - xy_temp).theta();
        slopes[i] = ma - mb;
    }


    if(check_corner) {
        LB_FLOAT min = std::numeric_limits<LB_FLOAT>::max();
        int min_idx = 0;
        // if is a corner exclude it
        if (slopes_size > 4) {
            //min slope
            for (size_t temp = 0;temp < slopes_size; temp++) {
                if (slopes[temp] < min) {
                    min = slopes[temp];
                    min_idx = temp;
                }
            }


            if (((slopes_size - min_idx) > 1) && (min_idx > 1)) {
                if (slopes[min_idx - 2] > slopes[min_idx - 1] &&
                    slopes[min_idx - 1] > slopes[min_idx + 0] &&
                    slopes[min_idx + 1] > slopes[min_idx + 0] &&
                    slopes[min_idx + 2] > slopes[min_idx + 1])
                {
                    std::cout << "Is a corner" << std::endl;
                    return false;// is a corner
                }
            }
        }
    }

    LB_FLOAT average = lb_mean(slopes);
    LB_FLOAT std_dev = lb_stdev(slopes, average);

//      PRINT_VAR(average);
//      PRINT_VAR(std_dev);



    //check if is line
    if ((fabs(average - M_PI) < is_line_error) && (std_dev < is_line_stdev)) {
        std::cout << "Is a line" << std::endl;
        return false;// is a line
    }

    LB_PRINT_VAR(std_dev);

    //check arc
    if (std_dev < max_stdev) {
        vec2f tmp = right - left;
        LB_FLOAT angle_to_rotate = atan2(tmp.y, tmp.x);
        tmp = tmp.rotate(-angle_to_rotate);
        LB_FLOAT mid = tmp.x / 2.0;
        LB_FLOAT q = average - (M_PI/2.0);
        LB_FLOAT height = mid * tan(q);
        vec2f center(mid, height);
        LB_FLOAT radius = center.size();
        center = center.rotate(angle_to_rotate);
        center = center + left;

        if(average < 0.f) {
            average = (2 * M_PI) + average;
        }

        LB_PRINT_VAR(average);

        if((average > min_aparture) && (average < max_aparture)) {
//              PRINT_VAR(center);
//              PRINT_VAR(radius);
//              PRINT_VAR(average);
//              PRINT_VAR(std_dev);

            lrf_object arc;
            arc.points = points;
            arc.type = LRF_OBJ_ARC;
            arc.extra_point[0] = center;
            arc.extra_param[0] = radius;
            arc.extra_param[1] = average;
            arc.extra_param[2] = std_dev;
            arc.segment_id = id;
            objects.push_back(arc);
            return true;
        } //if ( average...
    }//if ( std_dev ...

    return false;
}


}


#endif /* LB_LRF_OBJECT_DETECT_H_ */
