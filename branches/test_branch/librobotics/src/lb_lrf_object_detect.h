/*
 * lb_lrf_object_detect.h
 *
 *  Created on: Jan 19, 2009
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

#ifndef LB_LRF_OBJECT_DETECT_H_
#define LB_LRF_OBJECT_DETECT_H_

#include "lb_common.h"
#include "lb_exception.h"
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
        debug("%s: not enough point", __FUNCTION__);
        return 0;
    }

    //check length (simple)
    if((points[0] - points[n - 1]).size() < min_length) {
        debug("%s: not enough length", __FUNCTION__);
        return 0;
    }

    LB_FLOAT m = 0, b = 0, r = 0;
    lb_liner_regression(points, m, b, r);

    //check result
    if(m == 0 && b == 0 && r == 0) {
        debug("%s: linear regression fail", __FUNCTION__);
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
                              const LB_FLOAT min_angle,
                              const LB_FLOAT max_angle,
                              const LB_FLOAT max_stdev,
                              const LB_FLOAT arc_ratio,
                              const LB_FLOAT is_line_error,
                              const LB_FLOAT is_line_stdev,
                              const LB_FLOAT min_diameter = -1,
                              const LB_FLOAT max_diameter = (std::numeric_limits<LB_FLOAT>::max)(),
                              const int id = -1,
                              const size_t min_point = 10)
{
    size_t n = points.size();
    if(n < min_point) {
        debug("%s: not enough point", __FUNCTION__);
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

    if(dist_lr <= min_diameter) {
        debug("%s: too small", __FUNCTION__);
        return false;
    }

    if(dist_lr >= max_diameter) {
        debug("%s: too big", __FUNCTION__);
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
        debug("%s: arc ratio and size not correct", __FUNCTION__);
        return false;
    }


    size_t n_angle = n - 3;
    LB_FLOAT ma = 0, mb = 0;
    std::vector<LB_FLOAT>angles(n_angle, 0);

    // angle inside arc
    vec2f xy_temp;
    for (size_t i = 0; i < n_angle; i++) {
        xy_temp = points[i+1];
        ma = (left - xy_temp).theta();
        mb = (right - xy_temp).theta();
        angles[i] = fabs(lb_normalize_angle(ma - mb));
    }

//    LB_PRINT_VEC(slopes);
    LB_FLOAT average = lb_mean(angles);
    LB_FLOAT std_dev = lb_stdev(angles, average);

//    LB_PRINT_VAR(average);
//    LB_PRINT_VAR(std_dev);

    //check if is line
    if ((fabs(average - M_PI) < is_line_error) && (std_dev < is_line_stdev)) {
        debug("%s: is a line", __FUNCTION__);
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

        if((average >= min_angle) && (average <= max_angle)) {
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
        } else {
            debug("%s: average angle outside range", __FUNCTION__);
        }
    } else {
        debug("%s: angle standard deviation > max_stdev ", __FUNCTION__);
    }

    return false;
}

inline int lb_lrf_leg_detect(const std::vector<vec2f>& points,
                             std::vector<lrf_object>& objects,
                             const LB_FLOAT min_size,
                             const LB_FLOAT max_size,
                             const LB_FLOAT leg_arc_ratio,
                             const int id = -1,
                             const bool do_ratio_check = false,
                             const size_t min_points = 5)
{
    size_t n = points.size();
    if(n < min_points) {
        debug("%s: not enough point", __FUNCTION__);
        return 0;
    }

    vec2f middle = points[n >> 1];
    vec2f right = points[0];
    vec2f left = points[n - 1];
    vec2f center = (right + left) * 0.5;

    LB_FLOAT dist_lr = (left - right).size();

    if(dist_lr < min_size) {
        debug("%s: too small", __FUNCTION__);
        return 0;
    }

    if(dist_lr > (2.0 * max_size)) {
        debug("%s: too big", __FUNCTION__);
        return 0;
    }

    LB_FLOAT dist_mc = (middle - center).size();
    bool check_ratio = false;

    if(dist_lr <= max_size) {
//        LB_PRINT_VAL("check 1 leg");
        check_ratio = dist_mc > (leg_arc_ratio * dist_lr);
        check_ratio |= (!do_ratio_check);

        if(check_ratio) {
//            LB_PRINT_VAL("add 1 leg");
            lrf_object leg;
            leg.type = LRF_OBJ_LEG;
            leg.points = points;
            leg.extra_point[0] = center;
            leg.extra_param[0] = dist_lr * 0.5; //radius
            leg.segment_id = id;
            objects.push_back(leg);
            return 1;
        } else {
            debug("%s: 1 leg ratio fail", __FUNCTION__);
            return 0;
        }

    } else {
//        LB_PRINT_VAL("check 2 leg");
        int cnt = 0;
        vec2f right_middle = points[n >> 2];
        vec2f right_center = (right + middle) * 0.5;
        LB_FLOAT dist_mr = (middle - right).size();
        LB_FLOAT dist_rm_rc = (right_middle - right_center).size();

        check_ratio = dist_rm_rc > (leg_arc_ratio * dist_mr);
        check_ratio |= (!do_ratio_check);
        if(check_ratio) {
            //add right leg
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
            debug("%s: right leg ratio fail", __FUNCTION__);
        }


        vec2f left_middle = points[(n >> 2) + (n >> 1)];
        vec2f left_center = (left + middle) * 0.5;
        LB_FLOAT dist_ml = (middle - left).size();
        LB_FLOAT dist_lm_lc = (left_middle - left_center).size();

        check_ratio = dist_lm_lc > (leg_arc_ratio * dist_ml);
        check_ratio |= (!do_ratio_check);
        if(check_ratio) {
            //add left leg
            lrf_object leg;
            for(size_t i = (n >> 1); i < n; i++) {
                leg.points.push_back(points[i]);
            }
            leg.type = LRF_OBJ_LEG;
            leg.extra_point[0] = left_center;
            leg.extra_param[0] = dist_ml * 0.5; //radius
            leg.segment_id = id;
            objects.push_back(leg);
            cnt++;
        } else {
            debug("%s: left leg ratio fail", __FUNCTION__);
        }

        if(cnt == 2) {
            //remove last two leg
            objects.pop_back();
            objects.pop_back();

            lrf_object leg2;
            leg2.points = points;
            leg2.type = LRF_OBJ_LEG2;
            leg2.extra_point[0] = center;
            leg2.extra_param[0] = dist_lr * 0.5;
            leg2.segment_id = id;
            objects.push_back(leg2);
        }
        return cnt;
    }
    return 0;
}

inline bool lb_lrf_group_detect(const std::vector<vec2f>& points,
                                std::vector<lrf_object>& objects,
                                const LB_FLOAT min_size,
                                const LB_FLOAT max_size,
                                const int id = -1,
                                const size_t min_points = 5)
{
    size_t n = points.size();
    if(n < min_points) {
        debug("%s: not enough point", __FUNCTION__);
        return false;
    }

    vec2f middle = points[n >> 1];
    vec2f right = points[0];
    vec2f left = points[n - 1];
    vec2f center = (right + left) * 0.5;

    LB_FLOAT dist_lr = (left - right).size();
    LB_FLOAT dist_mc = (middle - center).size();
    LB_FLOAT dist_max = LB_MAX(dist_lr, dist_mc);


    if(dist_max < min_size) {
        debug("%s: too small", __FUNCTION__);
        return false;
    }

    if(dist_max > max_size) {
        debug("%s: too big", __FUNCTION__);
        return false;
    }

    LB_FLOAT sum_x = 0;
    LB_FLOAT sum_y = 0;
    for(size_t i = 0; i < n; i++) {
        sum_x += points[i].x;
        sum_y += points[i].y;
    }
    sum_x /= n;
    sum_y /= n;

    lrf_object group;
    group.points = points;
    group.type = LRF_OBJ_GROUP;
    group.extra_point[0] = vec2f(sum_x, sum_y);
    group.extra_param[0] = dist_max / 2; //radius
    group.segment_id = id;
    objects.push_back(group);

    return true;
}

inline int lb_lrf_object_human_check(std::vector<lrf_object>& objects,
                                     const LB_FLOAT max_leg_distance,
                                     const LB_FLOAT min_group_size,
                                     const LB_FLOAT max_group_size,
                                     bool allow_one_leg = false)
{
    size_t n = objects.size();

    if(n == 0) return 0;

    int human_cnt = 0;
    std::list<lrf_object> leg_object;
    std::list<lrf_object>::iterator it;
    LB_FLOAT leg_dist;
    for(size_t i = 0; i < n; i++) {
        if(objects[i].type == LRF_OBJ_LEG2) {
            //add human
//            LB_PRINT_VAL("add 2 leg");
            lrf_object human = objects[i];
            human.type = LRF_OBJ_HUMAN;
            objects.push_back(human);
            human_cnt++;
        } else if(objects[i].type == LRF_OBJ_LEG) {
            if(leg_object.size() > 0) {
                //check current leg with leg_list
                for (it = leg_object.begin(); it != leg_object.end(); it++) {
                    //check distance
                    leg_dist = ((*it).extra_point[0] - objects[i].extra_point[0]).size();
                    if(leg_dist < max_leg_distance) {
//                        LB_PRINT_VAL("add 1+1 leg");
                        //add human
                        lrf_object human;
                        human.points = (*it).points;
                        human.points.insert( human.points.end(),
                                             objects[i].points.begin(),
                                             objects[i].points.end());
                        human.extra_point[0] =
                            ((*it).extra_point[0] + objects[i].extra_point[0]) * 0.5;
                        human.type = LRF_OBJ_HUMAN;
                        objects.push_back(human);
                        human_cnt++;


                        //remove
                        leg_object.erase(it);
                        break;
                    } else {
//                        LB_PRINT_VAL("add new leg to the list");
                        leg_object.push_back(objects[i]);
                    }
                }
            } else {
//                LB_PRINT_VAL("get 1st leg");
                leg_object.push_back(objects[i]);
            }
        }
    }


    //check all orphan leg
    int pass = 0;
    int group_idx = -1;
    while(!leg_object.empty() && allow_one_leg) {
        //check 1 leg condition
        it = leg_object.begin();
        pass = -10;
        group_idx = -1;
        for(size_t i = 0; i < n; i++) {
            if(objects[i].segment_id == (*it).segment_id) {
                switch(objects[i].type) {
                case LRF_OBJ_LINE : pass--; break;
                case LRF_OBJ_ARC : pass--; break;
                case LRF_OBJ_GROUP :
                    if(min_group_size <= objects[i].extra_param[0] &&
                       max_group_size >= objects[i].extra_param[0])
                    {
                        pass++;
                        group_idx = i;
                    }
                    break;
                default:
                    break;
                }
            }
        }

        if(pass >= 2) {
//            LB_PRINT_VAL("add 1 leg human");
            if(group_idx == -1) {
                lrf_object human = (*it);
                human.type = LRF_OBJ_HUMAN;
                objects.push_back(human);
            } else {
                lrf_object human = objects[group_idx];
                human.type = LRF_OBJ_HUMAN;
                objects.push_back(human);
            }
            human_cnt++;
        }
        leg_object.pop_front();
    }

    return human_cnt;
}


}


#endif /* LB_LRF_OBJECT_DETECT_H_ */
