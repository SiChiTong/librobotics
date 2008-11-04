/*
 * buggy_kf.h
 *
 *  Created on: Nov 3, 2008
 *      Author: mahisorn
 */

#ifndef BUGGY_KF_H_
#define BUGGY_KF_H_

#include "librobotics.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "boostMatrixInverse.h"

//#define COV_X SQR(0.001)
//#define COV_Y SQR(0.001)
//#define COV_A SQR(1.0)
//#define COV_V SQR(0.01)
//#define COV_W SQR(0.01)

#define COV_X SQR(0.01)
#define COV_Y SQR(0.01)
#define COV_A SQR(1.0)
#define COV_V SQR(0.1)
#define COV_W SQR(0.1)

//#define COV_za SQR(((2.0/180.0)*M_PI))   //compass angle
//#define COV_zv SQR(1.0)                  //encoder velocity
//#define COV_zw SQR(((5.0/180.0)*M_PI))   //gyro velocity

#define COV_za SQR(0.01)   //compass angle
#define COV_zv SQR(0.1)   //encoder velocity
#define COV_zw SQR(0.1)   //gyro velocity


//#define COV_zgx SQR(3.0)                 //GPS X
//#define COV_zgy SQR(3.0)                 //GPS Y
//#define COV_zgv SQR(1.0)                 //GPS V
//#define COV_zga SQR(1.0)                 //GPS A

#define COV_zgx SQR(3.0)                 //GPS X
#define COV_zgy SQR(3.0)                 //GPS Y
#define COV_zgv SQR(1.0)                 //GPS V
#define COV_zga SQR(0.5)                 //GPS A

class BuggyKF {
public:
    BuggyKF() :
        Xk_(boost::numeric::ublas::zero_vector<double>(5)),
        Pk_(boost::numeric::ublas::identity_matrix<double>(5,5)),
        X_k(boost::numeric::ublas::zero_vector<double>(5)),
        P_k(boost::numeric::ublas::identity_matrix<double>(5,5) * 0.001),

        A(boost::numeric::ublas::identity_matrix<double>(5,5)),
        Q(boost::numeric::ublas::identity_matrix<double>(5,5)),

        K_odo(boost::numeric::ublas::zero_matrix<double>(5,3)),
        K_odo_gps(boost::numeric::ublas::zero_matrix<double>(5,7)),

        z_odo(boost::numeric::ublas::zero_vector<double>(3)),
        z_odo_gps(boost::numeric::ublas::zero_vector<double>(7)),

        H_odo(boost::numeric::ublas::zero_matrix<double>(3,5)),
        H_odo_gps(boost::numeric::ublas::zero_matrix<double>(7,5)),

        R_odo(boost::numeric::ublas::identity_matrix<double>(3,3)),
        R_odo_gps(boost::numeric::ublas::identity_matrix<double>(7,7))
    {
        Q(0,0) = COV_X;
        Q(1,1) = COV_Y;
        Q(2,2) = COV_A;
        Q(3,3) = COV_V;
        Q(4,4) = COV_W;

        R_odo(0,0) = COV_za;
        R_odo(1,1) = COV_zv;
        R_odo(2,2) = COV_zw;

        R_odo_gps(0,0) = COV_za;
        R_odo_gps(1,1) = COV_zv;
        R_odo_gps(2,2) = COV_zw;
        R_odo_gps(3,3) = COV_zgx;
        R_odo_gps(4,4) = COV_zgy;
        R_odo_gps(5,5) = COV_zga;
        R_odo_gps(6,6) = COV_zgv;


        H_odo(0,2) = 1; //[ 0 0 1 0 0]
        H_odo(1,3) = 1; //[ 0 0 0 1 0]
        H_odo(2,4) = 1; //[ 0 0 0 0 1]

        H_odo_gps(0,2) = 1; //[ 0 0 1 0 0]
        H_odo_gps(1,3) = 1; //[ 0 0 0 1 0]
        H_odo_gps(2,4) = 1; //[ 0 0 0 0 1]
        H_odo_gps(3,0) = 1; //[ 1 0 0 0 0]
        H_odo_gps(4,1) = 1; //[ 0 1 0 0 0]
        H_odo_gps(5,2) = 1; //[ 0 0 1 0 0]
        H_odo_gps(6,3) = 1; //[ 0 0 0 1 0]



    }

    ~BuggyKF() { }


    void  init(double x, double y, double a, double v, double w) {
        X_k(0) = x;
        X_k(1) = y;
        X_k(2) = a;
        X_k(3) = v;
        X_k(4) = w;
    }


    /*
     * Xk_ = A*Xk
     * Pk_ = A*P_k*A' + Q
     *
     * Xk_             A          Xk
     * [x]   [1 0 0 cos(a)*dt  0] [x]    -> x position
     * [y]   [0 1 0 sin(a)*dt  0] [y]    -> y position
     * [a] = [0 0 1     0     dt] [a]    -> direction (east == 0, north = pi/2)
     * [v]   [0 0 0     1      0] [v]    -> speed (m/s)
     * [w]   [0 0 0     0      1] [w]    -> rotation speed(rad/s)
     */
    void predict(double dt) {
        using namespace boost::numeric::ublas;
        A(0,3) = cos(X_k(2)) * dt;
        A(1,3) = sin(X_k(2)) * dt;
        A(2,4) = dt;

        Xk_ = prod(A, X_k);

        matrix<double> AP_k = prod(A, P_k);
        Pk_ = prod(AP_k, trans(A)) + Q;
    }


    /*
     * K_k = (Pk_ * H') * inv((H*Pminus_k*H') + R);
     * X_k = Xk_ + K_k*(z_k - H*Xk_)
     * P_k = (I - K*H)*Pk_
     *
     * z_odo      H_odo  X
     * [a]   [0 0 1 0 0][x]    -> compass angle
     * [v] = [0 0 0 1 0][y]    -> encoder velocity
     * [w]   [0 0 0 0 1][a]    -> gyro rotation velocity
     *                  [v]
     *                  [w]
     *
     * z_odo_gps   H_odo_gps X
     * [a]       [0 0 1 0 0][x]
     * [v]       [0 0 0 1 0][y]
     * [w]       [0 0 0 0 1][a]
     * [gx]  =   [1 0 0 0 0][v]
     * [gy]      [0 1 0 0 0][w]
     * [ga]      [0 0 1 0 0]
     * [gv]      [0 0 0 1 0]
     *
     */
    void update_odo(double a, double v, double w) {
        using namespace boost::numeric::ublas;

        bool isSingular = false;
        matrix<double> HP = prod(H_odo, Pk_);
        //PRINTVAR(HP);

        matrix<double> HPHt = prod(HP, trans(H_odo)) + R_odo;
        //PRINTVAR(HPHt);

        matrix<double> invHPHt = gjinverse( HPHt,isSingular);
        //PRINTVAR(invHPHt);

        matrix<double> PHt = prod(Pk_, trans(H_odo));
        //PRINTVAR(PHt);

        if(!isSingular) {
            K_odo = prod(PHt, invHPHt);
        } else {
            std::cerr << "Cannot perform invert HPH'\n";
            return;
        }

        //PRINTVAR(v);
        //PRINTVAR(w);
        //PRINTVAR(a);

        z_odo(0) = a;
        z_odo(1) = v;
        z_odo(2) = w;

//        PRINTVAR(z_odo);
//        PRINTVAR(K_odo);
//        PRINTVAR(H_odo);
        //PRINTVAR(Xk_);

        //PRINTVAR(z_odo - prod(H_odo, Xk_));

        X_k = Xk_ + prod(K_odo, (z_odo - prod(H_odo, Xk_)));
        P_k = prod(identity_matrix<double>(5,5) -  prod(K_odo, H_odo), Pk_);
    }
    void update_gps(double a, double v, double w, double gx, double gy, double ga, double gv) {
        using namespace boost::numeric::ublas;

        bool isSingular = false;
        matrix<double> HP = prod(H_odo_gps, Pk_);
        //PRINTVAR(HP);

        matrix<double> HPHt = prod(HP, trans(H_odo_gps)) + R_odo_gps;
        //PRINTVAR(HPHt);

        matrix<double> invHPHt = gjinverse( HPHt,isSingular);
        //PRINTVAR(invHPHt);

        matrix<double> PHt = prod(Pk_, trans(H_odo_gps));
        //PRINTVAR(PHt);

        if(!isSingular) {
            K_odo_gps = prod(PHt, invHPHt);
        } else {
            std::cerr << "Cannot perform invert HPH'\n";
            return;
        }

        z_odo_gps(0) = a;
        z_odo_gps(1) = v;
        z_odo_gps(2) = w;
        z_odo_gps(3) = gx;
        z_odo_gps(4) = gy;
        z_odo_gps(5) = ga;
        z_odo_gps(6) = gv;

        X_k = Xk_ + prod(K_odo_gps, (z_odo_gps - prod(H_odo_gps, Xk_)));
        P_k = prod(identity_matrix<double>(5,5) -  prod(K_odo_gps, H_odo_gps), Pk_);
    }

    /*

     *

     *

     *

     */




    boost::numeric::ublas::vector<double> Xk_;  /**< predicted state*/
    boost::numeric::ublas::matrix<double> Pk_;  /**< predicted covariance*/

    boost::numeric::ublas::vector<double> X_k;  /**< updated state*/
    boost::numeric::ublas::matrix<double> P_k;  /**< updated covariance*/

    /** hold relation between x_k and x_{k-1} */
    boost::numeric::ublas::matrix<double> A;
    //boost::numeric::ublas::matrix<double> At;  /**< transpose matrix of #A*/
    boost::numeric::ublas::matrix<double> Q;    /**< process noise covariance*/

    boost::numeric::ublas::matrix<double> K_odo;            /**< Kalman gain*/
    boost::numeric::ublas::matrix<double> K_odo_gps;            /**< Kalman gain*/
    boost::numeric::ublas::vector<double> z_odo;            /**< latest measurement*/
    boost::numeric::ublas::vector<double> z_odo_gps;            /**< latest measurement*/


    /** hold relation between z_k and x_k */
    boost::numeric::ublas::matrix<double> H_odo;
    boost::numeric::ublas::matrix<double> H_odo_gps;

    /** measurement noise covariance*/
    boost::numeric::ublas::matrix<double> R_odo;
    boost::numeric::ublas::matrix<double> R_odo_gps;

};


#endif /* BUGGY_KF_H_ */
