/*
 * buggy_robot.cpp
 *
 *  Created on: Nov 2, 2008
 *      Author: mahisorn
 */

#if 0
#include <iostream>

#include <CImg.h>

#define librobotics_use_cimg
#include "librobotics.h"
#include "buggy_kf.h"



using namespace std;
using namespace librobotics;
using namespace cimg_library;

ofstream gps_log;
ofstream odo_log;

const unsigned char
    red[] = { 255,0,0 }, green[] = { 0,255,0 }, blue[] = { 0,0,255 },
    cyan[] = {0, 255, 255}, yellow[] = {255, 255, 0},
    red_drak[] = { 127,0,0 }, green_drak[] = { 0,127,0 }, blue_drak[] = { 0,0,127 },
    cyan_drak[] = {0, 127, 127}, yellow_dark[] = {127, 127, 0},
    black[] = { 0,0,0 },
    gray[] = { 127, 127, 127 },
    white[] = { 255,255,255 };


int main (int argc, char* argv[]) {
    if(argc < 2) {
         cout << "Please enter log file" << endl;
         return -1;
     }

    logdata_buggy_robot<double,int> buggy_log;
    buggy_log.open(argv[1]);

    buggy_log.read_all();
    PRINTVAR(buggy_log.step);
    PRINTVAR(buggy_log.odo.size());
    PRINTVAR(buggy_log.lrf.size());
    PRINTVAR(buggy_log.gps.size());

    vector<double> fi(1081);
    vector<double> co(1081);
    vector<double> si(1081);
    std::vector<vec2d> lrf_pts0;

    lrf_init_fi_table(DEG2RAD(-135), DEG2RAD(0.25), fi);
    lrf_init_cos_sin_table(DEG2RAD(-135), DEG2RAD(0.25), co, si);

    double gx0, gy0;
    gx0 = buggy_log.gps[0].x;
    gy0 = buggy_log.gps[0].y;
    for(size_t i = 0; i < buggy_log.gps.size(); i++) {
        buggy_log.gps[i].x -= gx0;
        buggy_log.gps[i].y -= gy0;
    }


    pose2d pose;
    pose2d pose_cmp;
    int step = 0;
    odo_log.open("odo_compare.dat");
    pose.a = buggy_log.odo[step].a;
    BuggyKF kf;
    kf.init(0.0, 0.0, pose.a, 0.0, 0.0);

    while(step < (int)buggy_log.odo.size()) {
        double a = pose.a + buggy_log.odo[step].da;
        double x = pose.x + (buggy_log.odo[step].ds * cos(a));
        double y = pose.y + (buggy_log.odo[step].ds * sin(a));


        pose.x = x;
        pose.y = y;
        pose.a = a;

        pose_cmp.a = buggy_log.odo[step].a;
        x = pose_cmp.x + (buggy_log.odo[step].ds * cos(pose_cmp.a));
        y = pose_cmp.y + (buggy_log.odo[step].ds * sin(pose_cmp.a));

        pose_cmp.x = x;
        pose_cmp.y = y;


        kf.predict(0.1);

        if(step % 10 == 0) {
//            cout << "GPS update" << endl;
//            PRINTVAR(step);
            kf.update_gps(buggy_log.odo[step].ds * 10.0,
                          buggy_log.odo[step].da * 10.0,
                          buggy_log.odo[step].a,
                          buggy_log.gps[step/10 - 1].x,
                          buggy_log.gps[step/10 - 1].y,
                          buggy_log.gps[step/10 - 1].a,
                          buggy_log.gps[step/10 - 1].v);

        } else {
//            cout << "ODO update" << endl;
            kf.update_odo(buggy_log.odo[step].a,
                          buggy_log.odo[step].ds * 10.0,
                          buggy_log.odo[step].da  * 10.0);
        }


        odo_log <<  pose << " "
                <<  pose_cmp << " "
                << kf.X_k(0) << " " << kf.X_k(1) << " " << kf.X_k(2) << " "
                << kf.X_k(3) << " " << kf.X_k(4) << "\n";

        PRINTVAR(step++);
    }
    odo_log.close();



    /*
    CImgDisplay display_main(1000,1000,"Main Display",0);

    int step = 0;
    bool run = false;
    while(!display_main.is_closed && !display_main.is_keyESC && step < buggy_log.lrf.size()) {
        if(display_main.is_resized) {
              display_main.resize();
          }

        if(display_main.is_keySPACE || run) {
            CImg<unsigned char> img_lrf(display_main.dimx(),display_main.dimy(),1,3,0);
            lrf_scan_point_from_scan_range(buggy_log.lrf[step], co, si, lrf_pts0);
            lrf_draw_scan_point_to_cimg(lrf_pts0, img_lrf, red, false, 0.015);
            img_lrf.display(display_main);
            display_main.is_keySPACE = false;

            step++;
        }

        if(display_main.is_keyR) {
            run = !run;
        }


    }
    */
    double angle = 0;
//
    gps_log.open("gps_log.dat");
    gps_log.precision(20);
    for(size_t i = 0; i < buggy_log.gps.size(); i++) {
        angle = norm_a_rad(-buggy_log.gps[i].a + M_PI/2);
        if(angle < 0) angle += 2*M_PI;
        gps_log << (buggy_log.gps[i].x) << " " << (buggy_log.gps[i].y) << " "  <<  angle << " " << buggy_log.gps[i].v << "\n";
    }
    gps_log.close();
//
//    odo_log.open("odo_log.dat");
//
//    for(size_t i = 0; i < buggy_log.odo.size(); i++) {
//        angle = buggy_log.odo[i].a;
//        if(angle < 0) angle += 2*M_PI;
//        odo_log << buggy_log.odo[i].ds << " " << buggy_log.odo[i].da << " "  << angle << "\n";
//    }
//    odo_log.close();



}


#endif
