/*
 * lb_kalman_tracker2.h
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

#ifndef LB_KALMAN_TRACKER2_H_
#define LB_KALMAN_TRACKER2_H_

#include "lb_common.h"
#include "lb_exception.h"
#include "lb_data_type.h"

namespace librobotics {


/**
 * Using the Kalman Filter to track Human Interactive Motion
 * Modeling and Initialization of the Kalman Filter for Translation Motion
 * by Markus Kohler
 */
struct lb_kalman_tracker2 {
    lb_kalman_tracker2()
        : x_k(4), P_k(4,4),
          xk(4),
          Pk(boost::numeric::ublas::identity_matrix<LB_FLOAT>(4)),
          Kk(4,4), zk(4),
          A(boost::numeric::ublas::identity_matrix<LB_FLOAT>(4)),
          At(4,4),
          H(boost::numeric::ublas::identity_matrix<LB_FLOAT>(4)),
          Ht(4,4),
          R(boost::numeric::ublas::identity_matrix<LB_FLOAT>(4)),
          Q(boost::numeric::ublas::identity_matrix<LB_FLOAT>(4))
    { }


    ~lb_kalman_tracker2()
    { }


    /**
     * Initialize tracker
     */
    void init(const vec2f& x0,
              const LB_FLOAT max_a,
              const LB_FLOAT max_v,
              const LB_FLOAT update_time,
              const LB_FLOAT z_var_px,
              const LB_FLOAT z_var_py,
              const LB_FLOAT z_var_vx,
              const LB_FLOAT z_var_vy)
    {
        using namespace boost::numeric::ublas;
        matrix<LB_FLOAT> I = identity_matrix<LB_FLOAT>(2);

        max_acc = max_a;
        max_vel = max_v;
        dt = update_time;

        //  xminus_k = Ax_k + w_k
        //  A = [1.0    0.0     dt      0.0;
        //       0.0    1.0     0.0     dt;
        //       0.0    0.0     1.0     0.0;
        //       0.0    0.0     0.0     1.0];
        A(0,2) = dt;
        A(1,3) = dt;
        At = trans(A);

        //  z_k = Hx_k + v_k
        //  H = [1.0    0.0     0.0     0.0;
        //       0.0    1.0     0.0     0.0;
        //       0.0    0.0     0.0     0.0;
        //       0.0    0.0     0.0     0.0];
        H(0,0) = 1.0;
        H(1,1) = 1.0;
        H(2,2) = 0.0;
        H(3,3) = 0.0;
        Ht = trans(H);

        //  measurement covariance
        //  R = [px     0.0     0.0     0.0;
        //       0.0    py      0.0     0.0;
        //       0.0    0.0     vx      0.0;
        //       0.0    0.0     0.0     vy ];
        R(0,0) = z_var_px;
        R(1,1) = z_var_py;
        R(2,2) = z_var_vx;
        R(3,3) = z_var_vy;

        //  process covariance
        //  Q = (max_acc^2 * dt)/6.0 * [2*I*(dt*dt) 3*I*dt; 3*I*dt 6*I];
        LB_FLOAT tmp0 = (LB_SQR(max_a) * dt)/6.0;
        matrix<LB_FLOAT> Q0 = tmp0 * ((2*LB_SQR(dt))*I);
        matrix<LB_FLOAT> Q1 = tmp0 * ((3*dt)*I);
        matrix<LB_FLOAT> Q2 = tmp0 * ((3*dt)*I);
        matrix<LB_FLOAT> Q3 = tmp0 * (6*I);
        Q(0,0) = Q0(0,0); Q(0,1) = Q0(0,1); Q(0,2) = Q1(0,0); Q(0,3) = Q1(0,1);
        Q(1,0) = Q0(1,0); Q(1,1) = Q0(1,1); Q(1,2) = Q1(1,0); Q(1,3) = Q1(1,1);
        Q(2,0) = Q2(0,0); Q(2,1) = Q2(0,1); Q(2,2) = Q3(0,0); Q(2,3) = Q3(0,1);
        Q(3,0) = Q2(1,0); Q(3,1) = Q2(1,1); Q(3,2) = Q3(1,0); Q(3,3) = Q3(1,1);

        xk(0) = x0.x;
        xk(1) = x0.y;
        xk(2) = 0.0;
        xk(3) = 0.0;

        LB_FLOAT ds_max = max_vel * dt;
        LB_FLOAT s2 = LB_SQR(ds_max);
        LB_FLOAT v2 = LB_SQR(max_vel);
        Pk(0,0) = s2;
        Pk(1,1) = s2;
        Pk(2,2) = v2;
        Pk(3,3) = v2;
    }

    /**
     * prediction new position base on last data only
     * @return state vector [x,y,vx,vy]'
     */
    void predict(vec2f& p, vec2f& v)
    {
        using namespace boost::numeric::ublas;

        x_k = prod(A, xk);
        matrix<LB_FLOAT> tmp0 = prod(A, Pk);
        P_k = prod(tmp0, At) + Q;

        xk = x_k;
        Pk = P_k;

        p.x = xk(0);
        p.y = xk(1);
        v.x = xk(2);
        v.y = xk(3);
    }

    /**
     * prediction and update position using new measurement
     * @param pose new measurement data
     * @return state vector [x,y,vx,vy]'
     */
    void predict_update(const vec2f& z, vec2f& p, vec2f& v)
    {
        using namespace boost::numeric::ublas;

        //predict
        x_k = prod(A, xk); //x_k = A*x_0;
        matrix<LB_FLOAT> APk = prod(A, Pk);

        P_k = prod(APk, At) + Q; //P_k = A*P_0*A' + Q;

        //update
        //Kk = (P_k * H') * inv((H*P_k*H') + R);
        matrix<LB_FLOAT> P_kHt = prod(P_k, Ht);
        matrix<LB_FLOAT> HP_k = prod(H, P_k);
        matrix<LB_FLOAT> HP_kHt_R = prod(HP_k, Ht) + R;

        bool singular;
        matrix<LB_FLOAT> invert = gjinverse(HP_kHt_R, singular);

        if(singular) {
            warn("lb_kf_tracker2_white_noise_acc::%s cannot invert sigular matrix", __FUNCTION__);
            return;
        }

        Kk = prod(P_kHt, invert);

        //zk = [pose.x; pose.y; 0; 0];
        zk(0) = z.x;
        zk(1) = z.y;
        zk(2) = 0.f;
        zk(3) = 0.f;

        //xk = x_k + (Kk * (zk - (H*x_k)));
        xk =  x_k + prod(Kk, (zk - prod(H, x_k)));

        //Pk = (I - Kk*H)*P_k;
        Pk = prod(identity_matrix<LB_FLOAT>(4) - prod(Kk, H), P_k);

        p.x = xk(0);
        p.y = xk(1);
        v.x = xk(2);
        v.y = xk(3);
    }

    /**
     * get latest covariance matrix of filtered object position
     * @return [4x4] covariance matrix
     */


    LB_FLOAT max_acc;      /* maximum object linear acceleration*/
    LB_FLOAT max_vel;      /* maximum object linear velocity*/
    LB_FLOAT dt;

    boost::numeric::ublas::vector<LB_FLOAT> x_k;    /**< predicted state*/
    boost::numeric::ublas::matrix<LB_FLOAT> P_k;    /**< predicted covariance*/

    boost::numeric::ublas::vector<LB_FLOAT> xk;     /**< updated state*/
    boost::numeric::ublas::matrix<LB_FLOAT> Pk;     /**< updated covariance*/
    boost::numeric::ublas::matrix<LB_FLOAT> Kk;     /**< Kalman gain*/
    boost::numeric::ublas::vector<LB_FLOAT> zk;     /**< latest measurement*/


    boost::numeric::ublas::matrix<LB_FLOAT> A;
    boost::numeric::ublas::matrix<LB_FLOAT> At;     /**< transpose matrix of #A*/

    boost::numeric::ublas::matrix<LB_FLOAT> H;      /**< process noise covariance*/
    boost::numeric::ublas::matrix<LB_FLOAT> Ht;     /**< transpose matrix of #H*/
    boost::numeric::ublas::matrix<LB_FLOAT> R;      /**< measurement noise covariance*/
    boost::numeric::ublas::matrix<LB_FLOAT> Q;      /**< process noise covariance*/
};

struct lb_kalman_tracker2_object {
    static const int STATE_ERROR    = -1;
    static const int STATE_START    = 0;
    static const int STATE_BEGIN    = 1;
    static const int STATE_TRACK    = 2;
    static const int STATE_PREDICT  = 3;
    static const int STATE_LOST     = 4;
    static const int STATE_DIE      = 5;



    lb_kalman_tracker2_object() { }
    ~lb_kalman_tracker2_object() { }

    void init(const vec2f& x0,
              const LB_FLOAT max_a,
              const LB_FLOAT max_v,
              const LB_FLOAT update_time,
              const LB_FLOAT z_var_px,
              const LB_FLOAT z_var_py,
              const LB_FLOAT z_var_vx,
              const LB_FLOAT z_var_vy,
              const int max_lost_frame,
              const int min_found_frame,
              const LB_FLOAT dist_change_limit,
              int id = -1,
              bool check_moving_direction = false,
              LB_FLOAT moving_direction_threshold = 0.0)
    {
        tracker.init(x0,
                     max_a, max_v,
                     update_time,
                     z_var_px, z_var_py,
                     z_var_vx, z_var_vy);
        this->max_a = max_a;
        this->max_v = max_v;
        this->update_time = update_time;
        this->z_var_px = z_var_px;
        this->z_var_py = z_var_py;
        this->z_var_vx = z_var_vx;
        this->z_var_vy = z_var_vy;
        this->max_lost_frame = max_lost_frame;
        this->min_found_frame = min_found_frame;
        this->dist_change_limit = dist_change_limit;
        this->id = id;
        this->check_moving_direction = check_moving_direction;
        this->moving_direction_threshold = moving_direction_threshold;
        state = STATE_START;
        last_p = x0;
    }

    void reset_position(const vec2f & x0) {
        tracker.init(x0,
                     max_a, max_v,
                     update_time,
                     z_var_px, z_var_py,
                     z_var_vx, z_var_vy);
    }


    int update(const std::vector<vec2f>& points) {
        LB_FLOAT closest_dist = (std::numeric_limits<LB_FLOAT>::max)();
        vec2f move_dir;
        LB_FLOAT move_dir_size = 0;
        vec2f good_point;
        bool found = false;
        for(size_t i = 0; i < points.size(); i++) {
            move_dir = points[i] - last_p;
            move_dir_size = move_dir.size();

            if(check_moving_direction) {
                LB_PRINT_VAR("Not implement ");
                return STATE_ERROR;
            } else {
                //check with nearest point
                if(move_dir_size <= dist_change_limit) {
                    if(move_dir_size <= closest_dist) {
                        closest_dist = move_dir_size;
                        good_point = points[i];
                        found = true;
                    }
                } else {
                    warn("%s distance change limit", __FUNCTION__);
                }
            }

        }

        switch(state) {
            case STATE_START :
                found_cnt = max_lost_frame / 2; //start immediately

                if(found) {
                    nstate = STATE_TRACK;
//                    LB_PRINT_VAL("go STATE_BEGIN");
                } else {
                    nstate = STATE_START;
//                    LB_PRINT_VAL("repeat STATE_START");
                }

                break;
            case STATE_BEGIN :
                if(found) {
//                    LB_PRINT_VAL("Waiting");
                    found_cnt++;
                    if(found_cnt > min_found_frame) {
                        nstate = STATE_TRACK;
//                        LB_PRINT_VAL("go STATE_TRACK");
                        break;
                    }
                } else {
//                    LB_PRINT_VAL("lost during waiting");
                    found_cnt--;
                    if(found_cnt <= 0) {
                        nstate = STATE_LOST;
//                        LB_PRINT_VAL("go STATE_LOST");
                        break;
                    }
                }
                nstate = STATE_BEGIN;
//                LB_PRINT_VAL("repeat STATE_BEGIN");

                break;
            case STATE_TRACK :
                if(found) {
//                    LB_PRINT_VAL("Predict and update");
                    tracker.predict_update(good_point, last_p, last_v);
                    found_cnt++;
                    if(found_cnt > max_lost_frame)
                        found_cnt = max_lost_frame;
                    nstate = STATE_TRACK;
//                    LB_PRINT_VAL("repeat STATE_TRACK");
                } else {
                    tracker.predict(last_p, last_v);
                    found_cnt--;
                    if(found_cnt <= 0) {
                        nstate = STATE_LOST;
//                        LB_PRINT_VAL("go STATE_LOST");
                        break;
                    }
                    nstate = STATE_PREDICT;
//                    LB_PRINT_VAL("go STATE_PREDICT");
                }
                break;
            case STATE_PREDICT:
                if(!found) {
//                    LB_PRINT_VAL("Predict only");
                    tracker.predict(last_p, last_v);
                    found_cnt--;
                    if(found_cnt <= 0) {
                        nstate = STATE_LOST;
//                        LB_PRINT_VAL("go STATE_LOST");
                        break;
                    }
                } else {
                    tracker.predict_update(good_point, last_p, last_v);
                    found_cnt++;
                    nstate = STATE_TRACK;
//                    LB_PRINT_VAL("go STATE_TRACK");
                }

                break;
            case STATE_LOST :
                if(found) {
                    LB_PRINT_VAL("Recover");
                    found_cnt = 1;
                    nstate = STATE_BEGIN;
//                    LB_PRINT_VAL("go STATE_BEGIN");
                } else {
                    LB_PRINT_VAL("Lost");
                    found_cnt--;
                    if(found_cnt < -(max_lost_frame/2)) {
                        found_cnt = 0;
                        nstate = STATE_DIE;
//                        LB_PRINT_VAL("go STATE_DIE");
                    } else {
                        nstate = STATE_LOST;
//                        LB_PRINT_VAL("repeat STATE_LOST");
                    }
                }
                break;
            case STATE_DIE :
                nstate = STATE_DIE;
//                LB_PRINT_VAL("repeat STATE_DIE");
                break;
            case STATE_ERROR :
                break;
            default:
                break;
        }

        state = nstate;
        return state;
    }

    int state;
    int id;
    vec2f last_p;
    vec2f last_v;
    int found_cnt;
    lb_kalman_tracker2 tracker;

protected:
    int max_lost_frame;
    int min_found_frame;
    LB_FLOAT dist_change_limit;
    bool check_moving_direction;
    LB_FLOAT moving_direction_threshold;
    LB_FLOAT max_a, max_v;
    LB_FLOAT update_time;
    LB_FLOAT z_var_px, z_var_py;
    LB_FLOAT z_var_vx,  z_var_vy;
    int nstate;
};




}


#endif /* LB_KALMAN_TRACKER2_H_ */
