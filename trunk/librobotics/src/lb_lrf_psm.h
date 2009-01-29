/*
 * lb_lrf_psm.h
 *
 *  Created on: Jan 26, 2009
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

#ifndef LB_LRF_PSM_H_
#define LB_LRF_PSM_H_

#include "lb_common.h"
#include "lb_data_type.h"

namespace librobotics {

struct lb_lrf_psm_cfg {
    lb_lrf_psm_cfg() :
        scale(1000),        //scale factor from 1 m
        max_error(1*scale),
        search_angle(LB_DEG2RAD(20)),
        max_range(4*scale),
        min_range(0.1*scale),
        min_point(50),
        max_iter(20),
        small_corr_cnt(5)

    { }
    LB_FLOAT scale;
    LB_FLOAT max_error;
    LB_FLOAT search_angle;
    LB_FLOAT max_range;
    LB_FLOAT min_range;
    int min_point;
    int max_iter;
    int small_corr_cnt;

};

    inline bool lrf_psm(const pose2f& ref_robot_pose,
                        const pose2f& ref_lrf_pose,
                        const std::vector<LB_FLOAT>& ref_ranges,
                        const std::vector<unsigned int>& ref_bad,
                        const std::vector<int>& ref_segment,
                        const pose2f& act_robot_pose,
                        const pose2f& act_lrf_pose,
                        const std::vector<LB_FLOAT>& act_ranges,
                        const std::vector<unsigned int>& act_bad,
                        const std::vector<int>& act_segment,
                        const std::vector<LB_FLOAT>& fi_table,      //angle lookup table
                        const std::vector<LB_FLOAT>& co_table,      //cosine lookup table
                        const std::vector<LB_FLOAT>& si_table,      //sine lookup table
                        const lrf_psm_cfg& cfg,
                        pose2f& rel_lrf_pose,                  //scan match result
                        pose2f& rel_robot_pose,
                        bool force_check = true)
    {
        //relative robot position
        vec2f rel_act_robot_pose =
            ref_robot_pose.get_vec2_to(act_robot_pose).get_rotate(-ref_robot_pose.a);
        LB_FLOAT rel_act_robot_theta = lb_normalize_angle(act_robot_pose.a - ref_robot_pose.a);

        //relative actual laser position
        vec2f rel_act_lrf_pose =
            rel_act_robot_pose + act_lrf_pose.get_vec2().get_rotate(rel_act_robot_theta);
        LB_FLOAT rel_act_lrf_theta = lb_normalize_angle(rel_act_robot_theta + act_lrf_pose.a);


        //relative position between ref and act laser position
        vec2f rel_lrf_pose =
            rel_act_lrf_pose - ref_lrf_pose.get_vec2();
        LB_FLOAT rel_lrf_theta = lb_normalize_angle(rel_act_lrf_theta - ref_lrf_pose.a);

        std::vector<LB_FLOAT> refranges(ref_ranges);
        std::vector<LB_FLOAT> actranges(act_ranges);

        //Initialize variables
        size_t n_point = refranges.size();
        std::vector<LB_FLOAT> r(n_point, 0);
        std::vector<LB_FLOAT> fi(n_point, 0);
        std::vector<LB_FLOAT> new_r(n_point, 0);
        std::vector<unsigned int> new_bad(n_point, LRF_COND_EMPTY);
        std::vector<int> index(n_point, 0);

        LB_FLOAT angleStep = pm_fi[1] - pm_fi[0];
        int small_corr_cnt = 0;
        int iter = -1;
        int n = 0;//, n2 = 0;
        LB_FLOAT dx = 0, dy = 0, dth = 0;
        LB_FLOAT ax = rel_lrf_pose.x,  ay = rel_lrf_pose.y, ath = rel_lrf_theta;
        LB_FLOAT delta = 0, x = 0, y = 0, xr = 0, yr = 0;
        LB_FLOAT abs_err = 0;
        LB_FLOAT ri = 0;
        int idx = 0;
        size_t i = 0;
        LB_FLOAT C = LB_SQR(0.7 * cfg.scale);

        while((++iter < cfg.max_iter) && (small_corr_cnt < cfg.small_corr_cnt)) {

        }


        return false;
    }
    /*


        //transformation of actual scan laser scanner coordinates into reference
        //laser scanner coordinates







        while((++iter < cfg.maxIter) && (small_corr_cnt < cfg.smallCorrCnt)) {

            if(iter > 10) C = (1 * cfg.scale);

            T corr = fabs(dx)+fabs(dy)+fabs(dth);

            if(corr < (0.001 * cfg.scale)) {
                small_corr_cnt++;
            }
            else
                small_corr_cnt = 0;

            // convert range readings into ref frame
            // this can be speeded up, by connecting it with the interpolation
            for(i = 0; i < nPts; i++)
            {
                delta   = ath + pm_fi[i];
                xr = (actScanRanges[i] * cos(delta));
                yr = (actScanRanges[i] * sin(delta));
                x       = xr + ax;
                y       = yr + ay;
                r[i]    = sqrt((x*x)+(y*y));
                fi[i]   = atan2(y,x);
                new_r[i]  = 1e6;            //initialize big interpolated r;
                new_bad[i] = ERR_EMPTY;
            }//for i

            //------------------------INTERPOLATION------------------------
            //calculate/interpolate the associations to the ref scan points

            for(i = 1; i < nPts; i++) {
                // i and i-1 has to be in the same segment, both shouldn't be bad
                // and they should be bigger than 0
                if( actseg[i] >= 0 &&                           //is a segment
                    actseg[i] == actseg[i-1] &&                 //same segment
                    actranges[i] > 0 &&                         //has a measurement
                    (actbad[i] == 0) && (actbad[i-1] == 0))     //is a good measurement
                {
                    //calculation of the "whole" parts of the angles
                    T fi0 = 0, fi1 = 0;
                    T r0 = 0, r1 = 0, a0 = 0, a1 = 0;
                    bool occluded = false;

                    //are the points visible?
                    if(fi[i] > fi[i-1]) {
                        occluded = false;
                        a0  = fi[i-1];
                        a1  = fi[i];
                        fi0 = fi[i-1];//fi0 is the meas. angle!
                        fi1 = fi[i];
                        r0  = r[i-1];
                        r1  = r[i];
                    } else {
                        //invisible - still have to calculate to filter out points which
                        occluded = true; //are covered up by these!
                        //flip the points-> easier to program
                        a0  = fi[i];
                        a1  = fi[i-1];
                        fi0 = fi[i];
                        fi1 = fi[i-1];
                        r0  = r[i];
                        r1  = r[i-1];
                    }

                    //interpolate for all the measurement bearings between fi0 and fi1

                    while(fi0 <= fi1)//if at least one measurement point difference, then ...
                    {
                        //linear interpolate by r and theta ratio
                        ri = (((r1-r0)/(a1-a0))*(fi0 - a0)) + r0;

                        //if fi0 -> falls into the measurement range and ri is shorter
                        //than the current range then overwrite it
                        idx = (int)(fi0/angleStep);
                        idx += (nPts/2);
                        if((idx < (int)nPts) && new_r[idx]>ri) {
                            new_r[idx]    = ri; //overwrite the previous reading
                            index[idx]    = i;  //store which reading was used at index fi0

                            new_bad[idx]  &= ~ERR_EMPTY;    //clear the empty flag

                            //check if it was occluded
                            if(occluded) {
                                new_bad[idx] = ERR_OCCLUDED;//set the occluded flag
                            } else {
                                new_bad[idx] = NO_ERROR;
                            }

                            //the new range reading also it has to inherit the other flags
                            new_bad[idx] |= actbad[i];
                            new_bad[idx] |= actbad[i-1];
                        }
                        fi0 += angleStep;//check the next measurement angle!
                    }//while
                }//if
            }//for

            //---------------ORIENTATION SEARCH-----------------------------------
            //search for angle correction using cross correlation
            if((iter % 2) == 0) {
                T e;
                std::vector<T> err;         // the error rating
                std::vector<T> beta;        // angle for the corresponding error
                int ii = 0;
                int min_i = 0, max_i = 0;
                int wnd = (int)(cfg.searchWndAngle / angleStep);
                T dr;

                for(int di = -wnd ; di <= wnd; di++) {
                    n = 0; e = 0;
                    min_i = MAX2(-di, 0);
                    max_i = MIN2(nPts, nPts-di);
                    for(ii = min_i; ii < max_i; ii++)//searching through the actual points
                    {
                        if((new_bad[ii] == 0) &&
                           (refbad[ii+di] == 0))
                        {
                            dr = fabs(new_r[ii]-refranges[ii+di]);
                            e += dr;
                            n++;
                        }
                    }//for i

                    if(n > 0)
                        err.push_back(e/n);
                    else
                        err.push_back(1e6); //very large error
                    beta.push_back(di*angleStep);
                }//for di

                //now search for the global minimum
                //later I can make it more robust
                //assumption: monomodal error function!
                T emin = 1e6;
                int imin = 0;
                for(i = 0; i < err.size(); i++) {
                    if(err[i] < emin) {
                        emin = err[i];
                        imin = i;
                    }
                }

                if(err[imin] >= 1e6)
                {
                    warn("lrf_psm: orientation search failed: %f", err[imin]);
                    dx = 10;
                    dy = 10;
                    if(forceCheck) {
                        warn("lrf_psm: force check");
                        continue;
                    }
                    else
                        return false;
                } else {
                    dth = beta[imin];
                    if(imin >= 1 && (imin < (int)(beta.size()-1)) &&
                       err[imin-1] < 1e6 && err[imin+1] < 1e6 ) //is it not on the extreme?
                    {//lets try interpolation
                        T D = err[imin-1] + err[imin+1] - 2.0*err[imin];
                        T d = 1000;
                        if((fabs(D) > 0.01) &&
                           (err[imin-1] > err[imin]) &&
                           (err[imin+1] > err[imin]))
                        {
                            d = ((err[imin-1] - err[imin+1]) / D) / 2.0;
//                            warn("lrf_psm: Orientation refinement: %f ", d);
                        }
                        if(fabs(d) < 1) {
                            dth += d * angleStep;
                        }
                        ath += dth;
                    }
                }
                continue;
            }//if

            //-----------------translation-------------
            // do the weighted linear regression on the linearized ...
            // include angle as well
            T hi1, hi2, hwi1, hwi2, hw1 = 0, hw2 = 0, hwh11 = 0;
            T hwh12 = 0, hwh21 = 0, hwh22 = 0, w;
            T dr;
            abs_err = 0;
            n = 0;
            for (i = 0; i < nPts; i++) {
                dr = refranges[i] - new_r[i];

                //weight calculation
                if (refbad[i] == 0 &&
                    new_bad[i] == 0 &&
                    refranges[i] > 0 &&
                    new_r[i] > 0 &&
                    new_r[i] < cfg.lrfMaxRange &&
                    new_r[i] > cfg.lrfMinRange &&
                    fabs(dr) < cfg.maxError )
                {
                    n++;
                    abs_err += fabs(dr);
                    w = C / (dr * dr + C);

                    //proper calculations of the jacobian
                    hi1 = pm_co[i];
                    hi2 = pm_si[i];

                    hwi1 = hi1 * w;
                    hwi2 = hi2 * w;

                    //par = (H^t*W*H)^-1*H^t*W*dr
                    hw1 += hwi1 * dr;//H^t*W*dr
                    hw2 += hwi2 * dr;

                    //H^t*W*H
                    hwh11 += hwi1 * hi1;
                    hwh12 += hwi1 * hi2;
                    //hwh21 += hwi2*hi1; //should take adv. of symmetricity!!
                    hwh22 += hwi2 * hi2;
                }
            }//for i

            if(n < cfg.minValidPts) {
                warn("lrf_psm: Not enough points for linearize: %d", n);
                dx = 10;
                dy = 10;
                if(forceCheck){
                    warn("Polar Match: force check");
                    continue;
                }
                else
                    return false;
            }

            //calculation of inverse
            T D;//determinant
            T inv11,inv21,inv12,inv22;//inverse matrix
            D = (hwh11*hwh22) - (hwh12*hwh21);
            if(D < 0.001)
            {
                warn("lrf_psm: Determinant too small: %f", D);
                dy = 10;
                dy = 10;
                if(forceCheck){
                    warn("lrf_psm: force check");
                    continue;
                }
                else
                    return false;
            }

            inv11 =  hwh22/D;
            inv12 = -hwh12/D;
            inv21 = -hwh12/D;
            inv22 =  hwh11/D;

            dx = inv11*hw1+inv12*hw2;
            dy = inv21*hw1+inv22*hw2;

            ax += dx;
            ay += dy;
        }//while

        relLaserPose.x = ax;
        relLaserPose.y = ay;
        relLaserPose.a = ath;

#warning "!!!lrf_psm: relRobotPose still not compute!!!"

        return true;
    }
*/

}

#endif /* LB_LRF_PSM_H_ */
