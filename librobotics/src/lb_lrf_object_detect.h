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


inline int lb_lrf_recusive_line_fitting(const std::vector<vec2f>& points,
                                        std::vector<lrf_object>& objects,
                                        const LB_FLOAT min_length,
                                        const LB_FLOAT err_threshold,
                                        const int id = -1,
                                        const size_t min_point = 4)
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
            int line1 = lb_lrf_recusive_line_fitting(first_beam,
                                                     objects,
                                                     min_length,
                                                     err_threshold,
                                                     min_point);

            //Second half
            std::vector<vec2f> second_beam;
            for(size_t i = n_break; i < n; i++) {
                second_beam.push_back(points[i]);
            }

            int line2 = lb_lrf_recusive_line_fitting(second_beam,
                                                     objects,
                                                     min_length,
                                                     err_threshold,
                                                     min_point);
            return line1 + line2;
        }//if
    }//if

    return 0;
}


inline bool lb_lrf_arc_fiting(const std::vector<vec2f>& points,
                              std::vector<lrf_object>& objects,
                              const LB_FLOAT min_aparture,
                              const LB_FLOAT max_aparture,
                              const LB_FLOAT max_stdev,
                              const LB_FLOAT arc_ratio,
                              const LB_FLOAT is_line_error,
                              const LB_FLOAT is_line_stdev,
                              const LB_FLOAT min_diameter = -1,
                              const LB_FLOAT max_diameter = std::numeric_limits<LB_FLOAT>::max(),
                              const int id = -1,
                              const bool check_corner = true,
                              const size_t min_point = 20)
{
    size_t n = points.size();
    if(n < min_point) {
        return false;
    }

    vec2f middle = points[n >> 1];
    vec2f right = points[0];
    vec2f left = points[n - 1];
    vec2f center = (right + left) * 0.5;

//    LB_PRINT_VAR(left);
//    LB_PRINT_VAR(middle);
//    LB_PRINT_VAR(right);

    LB_FLOAT dist_lr = (left - right).size();
    LB_FLOAT dist_mc = (middle - center).size();

    if( (dist_lr < min_diameter) || (dist_lr > max_diameter) ) {
        return false;
    }

//    LB_PRINT_VAR(dist_lr);
//    LB_PRINT_VAR(dist_mc);
//    LB_PRINT_VAR(dist_mc/dist_lr);

    bool check_ratio = dist_mc > (arc_ratio * dist_lr);
    bool check_size = dist_mc < dist_lr;

//    LB_PRINT_VAR(check_ratio);
//    LB_PRINT_VAR(check_size);

    if(!(check_ratio && check_size)) {
//        std::cout << "Not an arc" << std::endl;
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
        slopes[i] = fabs(lb_normalize_angle(ma - mb));
    }

//    LB_PRINT_VEC(slopes);
    LB_FLOAT average = lb_mean(slopes);
    LB_FLOAT std_dev = lb_stdev(slopes, average);

//    LB_PRINT_VAR(average);
//    LB_PRINT_VAR(std_dev);


//    if(check_corner) {
//        LB_FLOAT min = std::numeric_limits<LB_FLOAT>::max();
//        int min_idx = 0;
//        // if is a corner exclude it
//        if (slopes_size > 4) {
//            //min slope
//            for (size_t temp = 0;temp < slopes_size; temp++) {
//                if (slopes[temp] < min) {
//                    min = slopes[temp];
//                    min_idx = temp;
//                }
//            }
//
//
//            if (((slopes_size - min_idx) > 1) && (min_idx > 1)) {
//                if (slopes[min_idx - 2] > slopes[min_idx - 1] &&
//                    slopes[min_idx - 1] > slopes[min_idx + 0] &&
//                    slopes[min_idx + 1] > slopes[min_idx + 0] &&
//                    slopes[min_idx + 2] > slopes[min_idx + 1])
//                {
//                    std::cout << "Is a corner" << std::endl;
//                    return false;// is a corner
//                }
//            }
//        }
//    }

    //check if is line
    if ((fabs(average - M_PI) < is_line_error) && (std_dev < is_line_stdev)) {
//        std::cout << "Is a line" << std::endl;
        return false;// is a line
    }

//    LB_PRINT_VAR(std_dev);

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

        if((average > min_aparture) && (average < max_aparture)) {
//            LB_PRINT_VAR(center);
//            LB_PRINT_VAR(radius);
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

inline int lb_lrf_leg_detect(const std::vector<vec2f>& points,
                             std::vector<lrf_object>& objects,
                             const LB_FLOAT min_size,
                             const LB_FLOAT max_size,
                             const LB_FLOAT leg_arc_ratio,
                             const int id = -1,
                             const size_t min_points = 10)
{
    size_t n = points.size();
    if(n < min_points) {
//        LB_PRINT_VAL("not enough point");
        return 0;
    }

    vec2f middle = points[n >> 1];
    vec2f right = points[0];
    vec2f left = points[n - 1];
    vec2f center = (right + left) * 0.5;

    LB_FLOAT dist_lr = (left - right).size();

    if(dist_lr < min_size) {
//        LB_PRINT_VAL("too small");
        return 0;
    }

    if(dist_lr > (2.0 * max_size)) {
//        LB_PRINT_VAL("too big");
        return 0;
    }

    LB_FLOAT dist_mc = (middle - center).size();
    bool check_ratio = false;


    if(dist_lr <= max_size) {
        LB_PRINT_VAL("check 1 leg");
        check_ratio = dist_mc > (leg_arc_ratio * dist_lr);

        if(check_ratio) {
            LB_PRINT_VAL("add 1 leg");
            lrf_object leg;
            leg.type = LRF_OBJ_LEG;
            leg.points = points;
            leg.extra_point[0] = center;
            leg.extra_param[0] = dist_lr / 2; //radius
            leg.segment_id = id;
            objects.push_back(leg);
        } else {
            LB_PRINT_VAL("1 leg fail");
        }
        return 1;
    } else {
        LB_PRINT_VAL("check 2 leg");
        //check 2 leg
        int cnt = 0;
        vec2f right_middle = points[n >> 2];
        vec2f right_center = (right + middle) * 0.5;
        LB_FLOAT dist_mr = (middle - right).size();
        LB_FLOAT dist_rm_rc = (right_middle - right_center).size();

        check_ratio = dist_rm_rc > (leg_arc_ratio * dist_mr);

        if(check_ratio) {
            //add right leg
            LB_PRINT_VAL("add right leg");
            lrf_object leg;
            for(size_t i = 0; i < (n >> 1); i++) {
                leg.points.push_back(points[i]);
            }
            leg.type = LRF_OBJ_LEG;
            leg.extra_point[0] = right_center;
            leg.extra_param[0] = dist_mr / 2; //radius
            leg.segment_id = id;
            objects.push_back(leg);
            cnt++;
        } else {
            LB_PRINT_VAL("right leg fail");
        }

        vec2f left_middle = points[(n >> 2) + (n >> 1)];
        vec2f left_center = (left + middle) * 0.5;
        LB_FLOAT dist_ml = (middle - left).size();
        LB_FLOAT dist_lm_lc = (left_middle - left_center).size();

        check_ratio = dist_lm_lc > (leg_arc_ratio * dist_ml);

        if(check_ratio) {
            //add left leg
            LB_PRINT_VAL("add left leg");
            lrf_object leg;
            for(size_t i = (n >> 1); i < n; i++) {
                leg.points.push_back(points[i]);
            }
            leg.type = LRF_OBJ_LEG;
            leg.extra_point[0] = left_center;
            leg.extra_param[0] = dist_ml / 2; //radius
            leg.segment_id = id;
            objects.push_back(leg);
            cnt++;
        } else {
            LB_PRINT_VAL("lelf leg fail");
        }
        return cnt;
    }
    return 0;


}

}


#endif /* LB_LRF_OBJECT_DETECT_H_ */
