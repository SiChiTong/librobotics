/*
 *  File(s)     : librobotics.h (C++ header)
 *
 *  Description : The C++ Robotics Library.
 *                This file is the main part of the LibRobotics project.
 *                ( http://code.google.com/p/librobotics/ )
 *
 *  Created on  : Oct 24, 2008
 *      Author  : Mahisorn Wongphati
 *
 *  Copyright (c) <2008> <Mahisorn Wongphati>
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

#ifndef librobotics_version
#define librobotics_version 0.2

//base
#include "src/lb_common.h"
#include "src/lb_simple_gui.h"
#include "src/lb_cimg_draw.h"
#include "src/lb_macro_function.h"
#include "src/lb_misc_function.h"
#include "src/lb_regression.h"
#include "src/lb_tools.h"
#include "src/lb_log_file.h"
#include "src/lb_statistic_function.h"
#include "src/lb_data_type.h"
#include "src/lb_math_model.h"
#include "src/lb_particle_function.h"
#include "src/lb_math_area_check.h"

//low level control function
#include "src/lb_simple_control.h"

//sensor function
#include "src/lb_lrf_basic.h"
#include "src/lb_lrf_object_detect.h"

//object tracker
#include "src/lb_kalman_tracker2.h"

//path planning
#include "src/lb_vfh.h"

//Localization and/or Mapping
#include "src/lb_map2_grid.h"
#include "src/lb_mcl2.h"


//
//        void reset_dyn_map() {
//            //for loop checking may faster??
//            dyn_mapprob = mapprob;
//        }
//
//        void add_dyn_object(const vec2<T>&, T size, T value) {
//
//        }
//
//
//        bool get_LPN_path(const vec2<T>& start,
//                          const vec2<T>& goal,
//                          std::vector<vec2i>& path,
//                          int grid_clearance = 3,
//                          T threshold = 0.6)
//        {
//            vec2i grid_start;
//            vec2i grid_goal;
//
//
//            //check start point
//            if(get_grid_coordinate(start.x, start.y, grid_start)) {
//                if(get_grid_value(start.x, start.y) > threshold) {
//                    std::cerr << "start position is inside the obstacle\n";
//                    return false;
//                }
//            } else {
//                std::cerr << "start position is outside the map\n";
//                return false;
//            }
//
//            //check goal point
//            if(get_grid_coordinate(goal.x, goal.y, grid_goal)) {
//                if(get_grid_value(goal.x, goal.y) > threshold) {
//                    std::cerr << "goal position is inside the obstacle\n";
//                    return false;
//                }
//            } else {
//                std::cerr << "goal position is outside the map\n";
//                return false;
//            }
//
//            std::cout << "Move from: " << start << " (" << grid_start
//                      <<  ") to: " << goal << "  (" << grid_goal << ")\n";
//
//            std::list<vec2i> active_list;
//
//            vec2i q;
//            int cost = 0;
//            int start_cost = 0;
//
//            PRINTVALUE("Start intrinsic");
//
//            // ============== create intrinsic cost ==============
//            //clear gradient map
//            for(int i = 0; i < mapsize.x; i++) {
//                for(int j = 0; j < mapsize.y; j++) {
//                    gradient_intr[i][j] = 0;
//                    if(dyn_mapprob[i][j] > 0.0) {
//                        gradient_map[i][j] = 0; //obstacle is target
//                        active_list.push_back(vec2i(i,j));
//                    } else {
//                        gradient_map[i][j] = std::numeric_limits<int>::max();
//                    }
//                }
//            }
//
//            PRINTVALUE("Find min dist");
//            //find min distance
//
//            long  start_time = utils_get_current_time();
//            while(active_list.size() > 0) {
//                q = active_list.front();
//                active_list.pop_front();
//                start_cost = gradient_map[q.x][q.y];
//                for(int x = q.x-1; x <= q.x+1; x++) {
//                    for(int y = q.y-1; y <= q.y+1; y++) {
//
//                        if((x >= 0) && (y >= 0) &&
//                           (x < mapsize.x) && (y < mapsize.y))
//                        {
//                            if((x == q.x) && (y == q.y)) continue;
//
//                            if(x ==  q.x || y == q.y)
//                                cost = start_cost + 10;
//                            else
//                                cost = start_cost + 14;
//
//
//                            if(cost < gradient_map[x][y]) {
//                                gradient_map[x][y] = cost;
//                                active_list.push_back(vec2i(x, y));
//                            }
//                        }
//                    }
//                }
//            }
//
//            PRINTVAR( utils_get_current_time() - start_time);
//
//
//            PRINTVALUE("compute intr");
//            start_time = utils_get_current_time();
//            //compute intrinsic cost and clear gradient map
//            for(int i = 0; i < mapsize.x; i++) {
//                for(int j = 0; j < mapsize.y; j++) {
//                    if(gradient_map[i][i] <= grid_clearance) {
//                        gradient_intr[i][j] = 100000;
//                    } else {
//                        gradient_intr[i][j] = 100 - LB_SQR(gradient_map[i][j]);
//                        if(gradient_intr[i][j] < 0)
//                            gradient_intr[i][j] = 0;
//                    }
//                    gradient_map[i][j] = std::numeric_limits<int>::max();
//                }
//            }
//            PRINTVAR( utils_get_current_time() - start_time);
//            // ===================================================
//
//
//            PRINTVALUE("start find path");
//            start_time = utils_get_current_time();
//            //add goal position
//            active_list.clear();
//            active_list.push_back(grid_goal);
//            gradient_map[grid_goal.x][grid_goal.y] = 0;
//
//            bool reach = false;
//
//            while(!reach && (active_list.size() > 0)) {
//                q = active_list.front();
//                active_list.pop_front();
//                start_cost = gradient_map[q.x][q.y];
//                for(int x = q.x - 1; x <= q.x+1 && (!reach); x++) {
//                    for(int y = q.y - 1; y <= q.y+1 && (!reach); y++) {
//                        if((x >= 0) && (y >= 0) &&
//                           (x < mapsize.x) && (y < mapsize.y))
//                        {
//                            if((x == grid_start.x) && (y == grid_start.y)) {
//                                reach = true;
//                                break;
//                            }
//
//                            if((x == q.x) && (y == q.y)) continue;
//
//                            //compute cost
//                            if(x ==  q.x || y == q.y) {
//                                cost = gradient_intr[x][y] + start_cost + 10;
//                            } else {
//                                cost = gradient_intr[x][y] + start_cost + 14;
//                            }
//
//
//                            if(cost < gradient_map[x][y]) {
//
//                                if(dyn_mapprob[x][y] < threshold) {
//                                    gradient_map[x][y] = cost;
//                                    active_list.push_back(vec2i(x, y));
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//
//            PRINTVAR( utils_get_current_time() - start_time);
//            PRINTVALUE("done find path");
//
//            if(reach) {
//                path.clear();
//                reach = false;
//                int v;
//                q = grid_start;
//                vec2i next_q;
//                while(!reach) {
//                    v = gradient_map[q.x][q.y];
//                    for(int x = q.x - 1; (x <= q.x+1) && !reach; x++) {
//                        for(int y = q.y - 1; (y <= q.y+1) && !reach; y++) {
//
//                            if((x == q.x) && (y == q.y)) continue;
//
//                            if((x >= 0) && (y >= 0) &&
//                               (x < mapsize.x) && (y < mapsize.y))
//                            {
//                                if(x == grid_goal.x && y == grid_goal.y) {
//                                    reach = true;
//                                }
//
//                                if(gradient_map[x][y] < v) {
//                                    v = gradient_map[x][y];
//                                    next_q.x = x;
//                                    next_q.y = y;
//                                }
//                            }
//                        }
//                    }//for
//
//                    path.push_back(next_q);
//                    q = next_q;
//                }
//
//                return true;
//            //search path
//            } else {
//                return false;
//            }
//
//
//
//
//
//
//
//        }
//
//

//
//
//

//
//
//    /*-------------------------------------------------------------------------
//     *
//     * Definition of the LibRobotics: Particle filter Functions
//     *
//     -------------------------------------------------------------------------*/

//

//

//







//    template<typename T>
//    void lrf_save_to_file(const std::string& filename,
//                          const std::vector<T>& lrf_range,
//                          const std::string& sperator = ",")
//    {
//        std::ofstream file;
//        open_file_with_exception(file, filename);
//        for(size_t i = 0; i < lrf_range.size(); i++ ) {
//            file << i << sperator << lrf_range[i] << std::endl;
//        }
//        file.close();
//    }
//
//    template<typename T1, typename T2>
//    void lrf_save_to_file(const std::string& filename,
//                          const std::vector<T1>& lrf_range0,
//                          const std::vector<T2>& lrf_range1,
//                          const std::string& sperator = ",")
//    {
//        std::ofstream file;
//        open_file_with_exception(file, filename);
//        warn_vsize2(lrf_range0, lrf_range1);
//        for(size_t i = 0;
//            (i < lrf_range0.size()) && (i < lrf_range0.size());
//            i++ )
//        {
//            file << i << sperator << lrf_range0[i]
//                      << sperator << lrf_range1[i] << std::endl;
//        }
//        file.close();
//    }
//
//    template<typename T1, typename T2, typename T3>
//    void lrf_save_to_file(const std::string& filename,
//                      const std::vector<T1>& lrf_range0,
//                      const std::vector<T2>& lrf_range1,
//                      const std::vector<T3>& lrf_range2,
//                      const std::string& sperator = ",")
//    {
//
//    }
//
//
//    template<typename T>
//    void lrf_save_to_file(const std::string& filename,
//                          const std::vector<std::vector<T> >& lrf_ranges_array,
//                          int step = -1,
//                          const std::string& sperator = ",")
//    {
//        std::ofstream file;
//        open_file_with_exception(file, filename);
//
//
//
//    }
//
//    template<typename T>
//    void lrf_save_to_file(const std::string& filename,
//                          const std::vector<vec2<T> >& lrf_pts,
//                          const std::string& sperator = ",")
//    {
//        std::ofstream file;
//        open_file_with_exception(file, filename);
//        for(size_t i = 0; i < lrf_pts.size(); i++ ) {
//            file << i << sperator << lrf_pts[i].x << sperator << lrf_pts[i].y << std::endl;
//        }
//        file.close();
//    }
//
//    template<typename T>
//    void lrf_save_to_file(const std::string& filename,
//                          const std::vector<std::vector<vec2<T> > >& lrf_pts_array,
//                          int step = -1,
//                          const std::string& sperator = ",")
//    {
//
//    }
//


//    /*-----------------------------------------------------------
//     *
//     * Definition of the LibRobotics: Polar Scan Match Algorithm
//     * (http://www.irrc.monash.edu.au/adiosi/downloads.html)
//     *
//     -----------------------------------------------------------*/
//
//
//    struct lrf_psm_cfg {
//        lrf_psm_cfg() :
//            scale(1.0),         //scale factor from 1 m
//            maxError(1*scale),
//            searchWndAngle(LB_DEG2RAD(20)),
//            lrfMaxRange(4*scale),
//            lrfMinRange(0.1*scale),
//            minValidPts(50),
//            maxIter(20),
//            smallCorrCnt(5)
//
//        { }
//        double scale;
//        double maxError;
//        double searchWndAngle;
//        double lrfMaxRange;
//        double lrfMinRange;
//        int minValidPts;
//        int maxIter;
//        int smallCorrCnt;
//
//    };
//
//    /**
//     * Polar Scan-match function for Albert Diosi
//     * http://www.irrc.monash.edu.au/adiosi/downloads.html
//     *
//     * @param ref_robot_pose
//     * @param ref_laser_pose
//     * @param ref_scan_ranges
//     * @param refbad
//     * @param refseg
//     * @param act_robot_pose
//     * @param act_laser_pose
//     * @param act_scan_ranges
//     * @param actbad
//     * @param actseg
//     * @param pm_fi
//     * @param pm_co
//     * @param pm_si
//     * @param cfg
//     * @param rel_laser_pose
//     * @param rel_robot_pose
//     * @param force_check
//     * @return
//     */
//    template <typename T, typename T2>
//    bool lrf_psm(const pose2<T>& ref_robot_pose,
//                 const pose2<T>& ref_laser_pose,
//                 const std::vector<T2>& ref_scan_ranges,
//                 const std::vector<unsigned int>& refbad,
//                 const std::vector<int>& refseg,
//                 const pose2<T>& act_robot_pose,
//                 const pose2<T>& act_laser_pose,
//                 const std::vector<T2>& act_scan_ranges,
//                 const std::vector<unsigned int>& actbad,
//                 const std::vector<int>& actseg,
//                 const std::vector<T>& pm_fi,      //angle lookup table
//                 const std::vector<T>& pm_co,      //cosine lookup table
//                 const std::vector<T>& pm_si,      //sine lookup table
//                 const lrf_psm_cfg& cfg,
//                 pose2<T>& rel_laser_pose,           //scan match result
//                 pose2<T>& rel_robot_pose,
//                 bool force_check = true)           //scan match result)
//    {
//        vec2<T> relRbPose = ref_robot_pose.vec_to(act_robot_pose).rot(-ref_robot_pose.a);
//        T relRbTheta = act_robot_pose.a -ref_robot_pose.a;
//
//        //transformation of actual scan laser scanner coordinates into reference
//        //laser scanner coordinates
//        vec2<T> actLrfPose = relRbPose + act_laser_pose.vec().rot(relRbTheta);
//        T actLrfTheta = relRbTheta + act_laser_pose.a;
//
//        vec2<T> relLrfPose = actLrfPose - ref_laser_pose.vec();
//        T relLrfTheta = norm_a_rad(actLrfTheta - ref_laser_pose.a);
//
//        std::vector<T2> refranges(ref_scan_ranges);
//        std::vector<T2> actranges(act_scan_ranges);
//
//        //some variables
//        size_t nPts = refranges.size();
//        std::vector<T> r(nPts, 0);
//        std::vector<T> fi(nPts, 0);
//        std::vector<T> new_r(nPts, 0);
//        std::vector<unsigned int> new_bad(nPts, ERR_EMPTY);
//        std::vector<int> index(nPts, 0);
//
//        double angleStep = pm_fi[1] - pm_fi[0];
//        int small_corr_cnt = 0;
//        int iter = -1;
//        int n = 0;//, n2 = 0;
//        T dx = 0, dy = 0, dth = 0;
//        T ax = relLrfPose.x,  ay = relLrfPose.y, ath = relLrfTheta;
//        T delta = 0, x = 0, y = 0, xr = 0, yr = 0;
//        T abs_err = 0;
//        T ri = 0;
//        int idx = 0;
//        size_t i = 0;
//        T C = LB_SQR(0.7 * cfg.scale);
//
//        while((++iter < cfg.maxIter) && (small_corr_cnt < cfg.smallCorrCnt)) {
//
//            if(iter > 10) C = (1 * cfg.scale);
//
//            T corr = fabs(dx)+fabs(dy)+fabs(dth);
//
//            if(corr < (0.001 * cfg.scale)) {
//                small_corr_cnt++;
//            }
//            else
//                small_corr_cnt = 0;
//
//            // convert range readings into ref frame
//            // this can be speeded up, by connecting it with the interpolation
//            for(i = 0; i < nPts; i++)
//            {
//                delta   = ath + pm_fi[i];
//                xr = (act_scan_ranges[i] * cos(delta));
//                yr = (act_scan_ranges[i] * sin(delta));
//                x       = xr + ax;
//                y       = yr + ay;
//                r[i]    = sqrt((x*x)+(y*y));
//                fi[i]   = atan2(y,x);
//                new_r[i]  = 1e6;            //initialize big interpolated r;
//                new_bad[i] = ERR_EMPTY;
//            }//for i
//
//            //------------------------INTERPOLATION------------------------
//            //calculate/interpolate the associations to the ref scan points
//
//            for(i = 1; i < nPts; i++) {
//                // i and i-1 has to be in the same segment, both shouldn't be bad
//                // and they should be bigger than 0
//                if( actseg[i] >= 0 &&                           //is a segment
//                    actseg[i] == actseg[i-1] &&                 //same segment
//                    actranges[i] > 0 &&                         //has a measurement
//                    (actbad[i] == 0) && (actbad[i-1] == 0))     //is a good measurement
//                {
//                    //calculation of the "whole" parts of the angles
//                    T fi0 = 0, fi1 = 0;
//                    T r0 = 0, r1 = 0, a0 = 0, a1 = 0;
//                    bool occluded = false;
//
//                    //are the points visible?
//                    if(fi[i] > fi[i-1]) {
//                        occluded = false;
//                        a0  = fi[i-1];
//                        a1  = fi[i];
//                        fi0 = fi[i-1];//fi0 is the meas. angle!
//                        fi1 = fi[i];
//                        r0  = r[i-1];
//                        r1  = r[i];
//                    } else {
//                        //invisible - still have to calculate to filter out points which
//                        occluded = true; //are covered up by these!
//                        //flip the points-> easier to program
//                        a0  = fi[i];
//                        a1  = fi[i-1];
//                        fi0 = fi[i];
//                        fi1 = fi[i-1];
//                        r0  = r[i];
//                        r1  = r[i-1];
//                    }
//
//                    //interpolate for all the measurement bearings between fi0 and fi1
//
//                    while(fi0 <= fi1)//if at least one measurement point difference, then ...
//                    {
//                        //linear interpolate by r and theta ratio
//                        ri = (((r1-r0)/(a1-a0))*(fi0 - a0)) + r0;
//
//                        //if fi0 -> falls into the measurement range and ri is shorter
//                        //than the current range then overwrite it
//                        idx = (int)(fi0/angleStep);
//                        idx += (nPts/2);
//                        if((idx < (int)nPts) && new_r[idx]>ri) {
//                            new_r[idx]    = ri; //overwrite the previous reading
//                            index[idx]    = i;  //store which reading was used at index fi0
//
//                            new_bad[idx]  &= ~ERR_EMPTY;    //clear the empty flag
//
//                            //check if it was occluded
//                            if(occluded) {
//                                new_bad[idx] = ERR_OCCLUDED;//set the occluded flag
//                            } else {
//                                new_bad[idx] = ERR_NONE;
//                            }
//
//                            //the new range reading also it has to inherit the other flags
//                            new_bad[idx] |= actbad[i];
//                            new_bad[idx] |= actbad[i-1];
//                        }
//                        fi0 += angleStep;//check the next measurement angle!
//                    }//while
//                }//if
//            }//for
//
//            //---------------ORIENTATION SEARCH-----------------------------------
//            //search for angle correction using cross correlation
//            if((iter % 2) == 0) {
//                T e;
//                std::vector<T> err;         // the error rating
//                std::vector<T> beta;        // angle for the corresponding error
//                int ii = 0;
//                int min_i = 0, max_i = 0;
//                int wnd = (int)(cfg.searchWndAngle / angleStep);
//                T dr;
//
//                for(int di = -wnd ; di <= wnd; di++) {
//                    n = 0; e = 0;
//                    min_i = LB_MAX(-di, 0);
//                    max_i = LB_MIN(nPts, nPts-di);
//                    for(ii = min_i; ii < max_i; ii++)//searching through the actual points
//                    {
//                        if((new_bad[ii] == 0) &&
//                           (refbad[ii+di] == 0))
//                        {
//                            dr = fabs(new_r[ii]-refranges[ii+di]);
//                            e += dr;
//                            n++;
//                        }
//                    }//for i
//
//                    if(n > 0)
//                        err.push_back(e/n);
//                    else
//                        err.push_back(1e6); //very large error
//                    beta.push_back(di*angleStep);
//                }//for di
//
//                //now search for the global minimum
//                //assumption: monomodal error function!
//                T emin = 1e6;
//                int imin = 0;
//                for(i = 0; i < err.size(); i++) {
//                    if(err[i] < emin) {
//                        emin = err[i];
//                        imin = i;
//                    }
//                }
//
//                if(err[imin] >= 1e6)
//                {
//                    warn("lrf_psm: orientation search failed: %f", err[imin]);
//                    dx = 10;
//                    dy = 10;
//                    if(force_check) {
//                        warn("lrf_psm: force check");
//                        continue;
//                    }
//                    else
//                        return false;
//                } else {
//                    dth = beta[imin];
//                    if(imin >= 1 && (imin < (int)(beta.size()-1)) &&
//                       err[imin-1] < 1e6 && err[imin+1] < 1e6 ) //is it not on the extreme?
//                    {//lets try interpolation
//                        T D = err[imin-1] + err[imin+1] - 2.0*err[imin];
//                        T d = 1000;
//                        if((fabs(D) > 0.01) &&
//                           (err[imin-1] > err[imin]) &&
//                           (err[imin+1] > err[imin]))
//                        {
//                            d = ((err[imin-1] - err[imin+1]) / D) / 2.0;
////                            warn("lrf_psm: Orientation refinement: %f ", d);
//                        }
//                        if(fabs(d) < 1) {
//                            dth += d * angleStep;
//                        }
//                        ath += dth;
//                    }
//                }
//                continue;
//            }//if
//
//            //-----------------translation-------------
//            // do the weighted linear regression on the linearized ...
//            // include angle as well
//            T hi1, hi2, hwi1, hwi2, hw1 = 0, hw2 = 0, hwh11 = 0;
//            T hwh12 = 0, hwh21 = 0, hwh22 = 0, w;
//            T dr;
//            abs_err = 0;
//            n = 0;
//            for (i = 0; i < nPts; i++) {
//                dr = refranges[i] - new_r[i];
//
//                //weight calculation
//                if (refbad[i] == 0 &&
//                    new_bad[i] == 0 &&
//                    refranges[i] > 0 &&
//                    new_r[i] > 0 &&
//                    new_r[i] < cfg.lrfMaxRange &&
//                    new_r[i] > cfg.lrfMinRange &&
//                    fabs(dr) < cfg.maxError )
//                {
//                    n++;
//                    abs_err += fabs(dr);
//                    w = C / (dr * dr + C);
//
//                    //proper calculations of the jacobian
//                    hi1 = pm_co[i];
//                    hi2 = pm_si[i];
//
//                    hwi1 = hi1 * w;
//                    hwi2 = hi2 * w;
//
//                    //par = (H^t*W*H)^-1*H^t*W*dr
//                    hw1 += hwi1 * dr;//H^t*W*dr
//                    hw2 += hwi2 * dr;
//
//                    //H^t*W*H
//                    hwh11 += hwi1 * hi1;
//                    hwh12 += hwi1 * hi2;
//                    //hwh21 += hwi2*hi1; //should take adv. of symmetricity!!
//                    hwh22 += hwi2 * hi2;
//                }
//            }//for i
//
//            if(n < cfg.minValidPts) {
//                warn("lrf_psm: Not enough points for linearize: %d", n);
//                dx = 10;
//                dy = 10;
//                if(force_check){
//                    warn("Polar Match: force check");
//                    continue;
//                }
//                else
//                    return false;
//            }
//
//            //calculation of inverse
//            T D;//determinant
//            T inv11,inv21,inv12,inv22;//inverse matrix
//            D = (hwh11*hwh22) - (hwh12*hwh21);
//            if(D < 0.001)
//            {
//                warn("lrf_psm: Determinant too small: %f", D);
//                dy = 10;
//                dy = 10;
//                if(force_check){
//                    warn("lrf_psm: force check");
//                    continue;
//                }
//                else
//                    return false;
//            }
//
//            inv11 =  hwh22/D;
//            inv12 = -hwh12/D;
//            inv21 = -hwh12/D;
//            inv22 =  hwh11/D;
//
//            dx = inv11*hw1+inv12*hw2;
//            dy = inv21*hw1+inv22*hw2;
//
//            ax += dx;
//            ay += dy;
//        }//while
//
//        rel_laser_pose.x = ax;
//        rel_laser_pose.y = ay;
//        rel_laser_pose.a = ath;
//
//        warn("!!!lrf_psm: rel_robot_pose still not compute!!!");
//
//        return true;
//    }
//
//
//
//    namespace slam {
//
//        /*-------------------------------------------------
//         *
//         * Definition of the LibRobotics: DPSLAM Algorithm
//         * (http://www.openslam.org/dpslam.html)
//         *
//         -------------------------------------------------*/
//
//        namespace dpslam {
//
//        }
//
//        /*----------------------------------------------------
//         *
//         * Definition of the LibRobotics: GridSLAM Algorithm
//         * (http://www.openslam.org/gridslam.html)
//         *
//         ----------------------------------------------------*/
//
//        namespace gridslam {
////            struct grid_line_t{
////              int                   numgrids;
////              std::vector<vec2i>    grid;
////              grid_line_t() :
////                  numgrids(0)
////              { }
////            } ;
////
////            struct QUAD_TREE {
////                struct QUAD_TREE*   elem[4];
////                vec2i               center;
////                unsigned char       level;
////                bool                inuse;
////            };
////
////            template<typename T> struct gridmap2 {
////                QUAD_TREE                           qtree;
////                pose2<T>                            offset;
////                T                                   resolution;
////                std::vector<std::vector<bool> >     updated;
////                std::vector<std::vector<T> >        maphit;
////                std::vector<std::vector<int> >      mapsum;
////                std::vector<std::vector<T> >        mapprob;
////                std::vector<std::vector<T> >        calc;
////                vec2i                               mapsize;
////                vec2<T>                             center;
////
////                bool inline isInside(int x, int y) {
////                    return (x >= 0) && (x < mapsize.x) && (y >= 0) && (y < mapsize.y);
////                }
////            };
//        }
//
//        /*----------------------------------------------------
//         *
//         * Definition of the LibRobotics: EKF SLAM in 2D space
//         *
//         ----------------------------------------------------*/
//
//        namespace ekf_slam2D {
//
//        }
//
//    }
//
//    /**
//     * Namespace for Localization algorithm
//     */
//    namespace localization {
//
//        /*-------------------------------------------------------------
//         *
//         * Definition of the LibRobotics: EKF Localization in 2D space
//         *
//         -------------------------------------------------------------*/
//        namespace ekf_feature2 {
//
//        }
//
//        /*-------------------------------------------------------------
//         *
//         * Definition of the LibRobotics: Monte Carlo Localization MCL in 2D space
//         * (for feature map)
//         *
//         -------------------------------------------------------------*/
//        namespace mcl_feature2 {
//
//        }
//



//
//            template<typename T>
//            int update_with_odomety(mcl_grid2::configuration<T>& cfg,
//                                    mcl_grid2::data<T>& data,
//                                    std::vector<vec2<T> > z,
//                                    pose2<T> odo_pose)
//            {
//                for(int n = 0; n < cfg.n_particles; n++) {
//                    //predict position
//                    data.p_tmp[n].pose =
//                        math_model::odometry_motion_sample(odo_pose,
//                                                           data.last_odo_pose,
//                                                           data.p[n].pose,
//                                                           cfg.motion_var);
//                    //check measurement
//                    vec2i grid_coor;
//                    T sense_angle = 0;
//                    int sense_idx = 0;
//                    T zp;
//
//                    if(data.map.get_grid_coordinate(data.p_tmp[n].pose.x, data.p_tmp[n].pose.y, grid_coor)) {
//                        if(data.map.ray_casting_cache[grid_coor.x][grid_coor.y].size() != 0) {
//                            data.p_tmp[n].w = 1.0;
//                            for(size_t i = 0; i < z.size(); i++) {
//                            //find nearest measurement in pre-computed ray casting
//
//                                //compute sense angle (convert from local coordinate to global coordinate)
//                                sense_angle = norm_a_rad(z[i].theta() + data.p_tmp[n].pose.a);
//
//                                //get index
//                                sense_idx = (int)(sense_angle/data.map.angle_res);
//                                if(sense_idx < 0) sense_idx += data.map.angle_step;
//
//                                //compute PDF (can speed up by lookup table)
//                                zp = math_model::beam_range_finder_measurement(z[i].size(),
//                                                                               data.map.ray_casting_cache[grid_coor.x][grid_coor.y][sense_idx],
//                                                                               cfg.z_max_range,
//                                                                               cfg.z_hit_var,
//                                                                               cfg.z_short_rate,
//                                                                               cfg.z_weight);
//                                data.p_tmp[n].w *= zp;
//                            }
//                        } else {
//                            data.p_tmp[n].w = 0;
//                        }
//
//                    } else {
//                        data.p_tmp[n].w = 0;
//                    }
//                    data.p[n].pose = data.p_tmp[n].pose;
//                    data.p[n].w = data.p_tmp[n].w;
//                }
//                data.last_odo_pose = odo_pose;
//                return 0;
//            }
//
//
//            template<typename T>
//            int update_with_odomety_augmented(mcl_grid2::configuration<T>& cfg,
//                                              mcl_grid2::data<T>& data,
//                                              std::vector<vec2<T> > z,
//                                              pose2<T> odo_pose,
//                                              int n_min)
//            {
//                //=====================================================
//                //                    Motion Update
//                //=====================================================
//                T w_avg = 0.0;
//                for(size_t n = 0; n < data.p.size(); n++) {
//                    //predict position
//                    data.p_tmp[n].pose =
//                        math_model::odometry_motion_sample(odo_pose,
//                                                           data.last_odo_pose,
//                                                           data.p[n].pose,
//                                                           cfg.motion_var);
//                    //check measurement
//                    vec2i grid_coor;
//                    T sense_angle = 0;
//                    int sense_idx = 0;
//                    T zp;
//
//                    if(data.map.get_grid_coordinate(data.p_tmp[n].pose.x, data.p_tmp[n].pose.y, grid_coor)) {
//                        if(data.map.ray_casting_cache[grid_coor.x][grid_coor.y].size() != 0) {
//                            data.p_tmp[n].w = 1.0;
//                            for(size_t i = 0; i < z.size(); i++) {
//                            //find nearest measurement in pre-computed ray casting
//
//                                //compute sense angle (convert from local coordinate to global coordinate)
//                                sense_angle = norm_a_rad(z[i].theta() + data.p_tmp[n].pose.a);
//
//                                //get index
//                                sense_idx = (int)(sense_angle/data.map.angle_res);
//                                if(sense_idx < 0) sense_idx += data.map.angle_step;
//
//                                //compute PDF (can speed up by lookup table)
//                                zp = math_model::beam_range_finder_measurement(z[i].size(),
//                                                                               data.map.ray_casting_cache[grid_coor.x][grid_coor.y][sense_idx],
//                                                                               cfg.z_max_range,
//                                                                               cfg.z_hit_var,
//                                                                               cfg.z_short_rate,
//                                                                               cfg.z_weight);
//                                data.p_tmp[n].w *= zp;
//                            }
//                        } else {
//                            data.p_tmp[n].w = 0;
//                        }
//
//                    } else {
//                        data.p_tmp[n].w = 0;
//                    }
//
//                    w_avg = w_avg + (data.p_tmp[n].w/data.p.size());
//                }
//
//
//                //=====================================================
//
////                std::sort(data.p_tmp.begin(), data.p_tmp.end(), pf_weight_compare<mcl_grid2::particle<T> >);
////                PRINTVEC(data.p_tmp);
//
//
//                //=====================================================
//                //                       re-sample
//                //=====================================================
//                size_t np = data.p_tmp.size();
//
//                data.w_slow = data.w_slow + cfg.a_slow*(w_avg - data.w_slow);
//                data.w_fast = data.w_fast + cfg.a_fast*(w_avg - data.w_fast);
//                T random_prob = LB_MAX(0.0, 1.0 - (cfg.v_factor*(data.w_fast / data.w_slow)));
//                int n_random = random_prob * cfg.n_particles;
//
//                PRINTVAR(data.w_slow);
//                PRINTVAR(data.w_fast);
//                PRINTVAR(random_prob);
//                PRINTVAR(n_random);
//
//
//                pf_normalize_weight(data.p_tmp);
//                double square_sum = 0;
//
//                //square sum
//                for(size_t i = 0; i < np; i++) {
//                   square_sum += LB_SQR(data.p_tmp[i].w);
//                }
//
//
//                int n_eff = (int)(1.0/square_sum);
//                PRINTVAR(n_eff);
//
//                if(n_eff > n_min) {
//                    data.p = data.p_tmp;
//                    return 0;
//                }
//
//                PRINTVALUE("Do Resample");
//
//                std::vector<double> cum_sum_w(np, 0.0);
//                std::vector<int> keep(np, 0);
//                std::vector<double> select(np, 0.0);
//
//                //cumulative sum
//                cum_sum_w[0] = data.p_tmp[0].w;
//                for(size_t i = 1; i < np; i++) {
//                    cum_sum_w[i] = cum_sum_w[i-1] + data.p_tmp[i].w;
//                }
//
//                stat_stratified_random(select, np);
//                size_t ctr = 0;
//                for(size_t i = 0; i < np; i++) {
//                    while((ctr < np) && (select[ctr] < cum_sum_w[i])) {
//                        keep[ctr] = i;
//                        ctr++;
//                    }
//                }
//
//                for(size_t i = 0; i < np; i++) {
//                    data.p[i].pose = data.p_tmp[keep[i]].pose;
//                    data.p[i].w = 1.0/np;
//                }
//
//                //update position
//                data.last_odo_pose = odo_pose;
//                return 1;
//            }
//        }
//
//        /**
//         * Namespace for grid localization in 2D space
//         */
//        namespace grid2 {
//            template<typename T>
//            struct grid2_data {
//
//            };
//
//
//            template<typename T>
//            int update(const map_grid2<T>& map) {
//                int angle_step = (int)((2.0 * M_PI) / map.angle_res);
//                for(int x = 0; x < map.x; x++) {
//                    for(int y = 0; y < map.y; y++) {
//                        for(int i = 0; i < angle_step; i++) {
//
//                        }
//                    }
//                }
//                return 0;
//            }
//
//        }
//
//    }
//

#endif //librobotics_version
