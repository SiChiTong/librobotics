/*
 * lb_simple_control.h
 *
 *  Created on: Jan 28, 2009
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

#ifndef LB_SIMPLE_CONTROL_H_
#define LB_SIMPLE_CONTROL_H_

#include "lb_common.h"
#include "lb_data_type.h"
#include "lb_exception.h"
#include "lb_misc_function.h"

namespace librobotics {


inline void lb_diff_drive_encoder_to_odometry(LB_FLOAT d_encoder_left, LB_FLOAT d_encoder_right,
                                              LB_FLOAT encoder_CPT, LB_FLOAT wheel_dist,
                                              LB_FLOAT dist_pre_turn_left, LB_FLOAT dist_pre_turn_right,
                                              pose2f& last_odo)
{
    LB_FLOAT dist_left = dist_pre_turn_left * (d_encoder_left/encoder_CPT);
    LB_FLOAT dist_right = dist_pre_turn_right * (-d_encoder_right/encoder_CPT);
    LB_FLOAT ds = (dist_left + dist_right) * 0.5;
    LB_FLOAT da = (dist_right - dist_left) / wheel_dist;

    last_odo.a += da;
    last_odo.a = lb_normalize_angle(last_odo.a);
    last_odo.x += (ds*cos(last_odo.a));
    last_odo.y += (ds*sin(last_odo.a));
}


struct lb_pid {
    LB_FLOAT p_err, d_err, i_err;
    LB_FLOAT kp, ki, kd, il;

    lb_pid()
        : p_err(0), d_err(0), i_err(0),
          kp(0), ki(0), kd(0), il(0)
    {}

    lb_pid(LB_FLOAT p, LB_FLOAT i, LB_FLOAT d, LB_FLOAT l)
        : p_err(0), d_err(0), i_err(0),
          kp(p), ki(i), kd(d), il(l)
    {}

    lb_pid(const lb_pid& pid)
        :kp(pid.kp), ki(pid.ki), kd(pid.kd), il(pid.il)
    {}

    void update_gain(LB_FLOAT p, LB_FLOAT i, LB_FLOAT d, LB_FLOAT l) {
        kp = p; ki = i; kd = d; il = l;
        reset_err();
    }

    void update_kp(LB_FLOAT p) {kp = p; reset_err();}
    void update_ki(LB_FLOAT i) {ki = i; reset_err();}
    void update_kd(LB_FLOAT d) {kd = d; reset_err();}
    void update_il(LB_FLOAT l) {il = l; reset_err();}

    void reset_err() {
        p_err = 0; d_err = 0; i_err = 0;
    }

    LB_FLOAT update(LB_FLOAT err) {
        d_err = err - p_err;
        p_err = err;
        i_err += d_err;

        if(fabs(i_err) > il) {
            if(i_err > 0)
                i_err = il;
            else
                i_err = -il;
        }

        return (kp*p_err) + (kd*d_err) + (ki*i_err);
    }

    ///Support for output stream operator
    friend std::ostream& operator << (std::ostream& os, const lb_pid& p) {
        return os << p.kp << " " << p.ki << " " << p.kd << " " << p.il;
    }
};


struct lb_simple_velocity_control {
    LB_FLOAT target, current;
    LB_FLOAT max_vel;
    LB_FLOAT max_acc;

    lb_simple_velocity_control()
        : target(0), current(0),
          max_vel(0),
          max_acc(0.0)
    {}

    lb_simple_velocity_control(LB_FLOAT v, LB_FLOAT a)
        : target(0), current(0),
          max_vel(v),
          max_acc(a)
    {}

    void update_vel_acc(LB_FLOAT a, LB_FLOAT v) {
        max_acc = a;
        max_vel = v;
    }

    void update_acc(LB_FLOAT a) {
        max_acc = a;
    }

    void update_target(LB_FLOAT v) {
        target = v;
    }

    void reset() {
        target = 0;
        current = 0;
    }

    LB_FLOAT get_next_vel( ) {
        LB_FLOAT tmp;
        if(target > current)
            tmp = current + max_acc;
        else if(target < current)
            tmp = current - max_acc;
        else
            tmp = current;

        if(fabs(tmp) > max_vel) {
            if(tmp > 0)
                tmp = max_vel;
            else
                tmp = -max_vel;
        }
        current = tmp;
        return current;
    }

    LB_FLOAT get_vel_error(LB_FLOAT real) {
        return current - real;
    }

    ///Support for output stream operator
    friend std::ostream& operator << (std::ostream& os, const lb_simple_velocity_control& v) {
        return os << v.max_vel << " " << v.max_acc;
    }
};


}

#endif /* LB_SIMPLE_CONTROL_H_ */
