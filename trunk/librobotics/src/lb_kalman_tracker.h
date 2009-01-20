/*
 * lb_kalman_tracker.h
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

#ifndef LB_KALMAN_TRACKER_H_
#define LB_KALMAN_TRACKER_H_

#include "lb_common.h"
#include "lb_data_type.h"

namespace librobotics {


/**
 *
 */
struct lb_kf_tracker2_white_noise_acc {
    lb_kf_tracker2_white_noise_acc()
        : x_k(4), P_k(4,4),
          xk(4),
          Pk(boost::numeric::ublas::identity_matrix<float>(4)),
          Kk(4,4), zk(4),
          A(boost::numeric::ublas::identity_matrix<float>(4)),
          At(4,4),
          H(boost::numeric::ublas::identity_matrix<float>(4)),
          Ht(4,4),
          R(boost::numeric::ublas::identity_matrix<float>(4)),
          Q(boost::numeric::ublas::identity_matrix<float>(4))
    { }


    ~lb_kf_tracker2_white_noise_acc()
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
        matrix<double> I = identity_matrix<double>(2);

        max_acc = max_a;
        max_vel = max_v;
        dt = update_time;

        //  z_k = Hx_k + v_k
        //  H = [1.0    0.0     0.0     0.0;
        //       0.0    1.0     0.0     0.0;
        //       0.0    0.0     0.0     0.0;
        //       0.0    0.0     0.0     0.0];
        H(0,0) = 1.f;
        H(1,1) = 1.f;
        H(2,2) = 0.f;
        H(3,3) = 0.f;
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
        matrix<LB_FLOAT> Q0 = tmp0 * (2*I*LB_SQR(dt));
        matrix<LB_FLOAT> Q1 = tmp0 * (3*I*dt);
        matrix<LB_FLOAT> Q2 = tmp0 * (3*I*dt);
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
        P_k(0,0) = s2;
        P_k(1,1) = s2;
        P_k(2,2) = v2;
        P_k(3,3) = v2;
    }

    /**
     * prediction new position base on last data only
     * @return state vector [x,y,vx,vy]'
     */
    void predict(const LB_FLOAT dt,
                 vec2f& p,
                 vec2f& v)
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
    void predict_update(const vec2f& z,
                        const LB_FLOAT dt,
                        vec2f& p,
                        vec2f& v)
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
            LB_PRINT_VAR(singular);
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






}


#endif /* LB_KALMAN_TRACKER_H_ */
