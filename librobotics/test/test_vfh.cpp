/*
 * test_vfh.cpp
 *
 *  Created on: Nov 7, 2008
 *      Author: chang
 */

#if 0
#include <iostream>

#include <CImg.h>

#define librobotics_use_cimg
#include "../librobotics.h"
using namespace std;
using namespace librobotics;
using namespace cimg_library;

#define DISP_SCALE      0.1
#define ANGLE_STEP      3.0
#define MIN_RANGE       500.0
#define MAX_RANGE       4000.0
#define VFH_A           10.0
#define THRESHOLD       100.0

const unsigned char
    red[] = { 255,0,0 }, green[] = { 0,255,0 }, blue[] = { 0,0,255 },
    cyan[] = {0, 255, 255}, yellow[] = {255, 255, 0},
    red_drak[] = { 127,0,0 }, green_drak[] = { 0,127,0 }, blue_drak[] = { 0,0,127 },
    cyan_drak[] = {0, 127, 127}, yellow_dark[] = {127, 127, 0},
    black[] = { 0,0,0 },
    gray[] = { 127, 127, 127 },
    white[] = { 255,255,255 };

int main(int argc, char* argv[]) {
//    vector<vec2d> vec;
//    vec.push_back(vec2d(1, -1));
//    vec.push_back(vec2d(-1, -1));
//    vec.push_back(vec2d(-1, 1));
//    vec.push_back(vec2d(1, 1));
//    PRINTVEC(vec);
//    sort(vec.begin(), vec.end(), angle_compare_xy_component<vec2d>);
//    PRINTVEC(vec);

    logdata_simple_odo_n_lrf<double, int> lrf_log;
    vector<string> lrf_label;
    lrf_label.push_back("LRF0");
    lrf_label.push_back("LRF1");
    lrf_label.push_back("LRF2");
    lrf_log.open("../log/log_1225986028.log", 3, "Odometry", lrf_label);
    lrf_log.read_all();
    PRINTVAR(lrf_log.step);

    vector<double> fi(769);
    vector<double> co(769);
    vector<double> si(769);

    lrf_init_fi_table(DEG2RAD(-135), DEG2RAD(0.3515625), fi);
    lrf_init_cos_sin_table(DEG2RAD(-135), DEG2RAD(0.3515625), co, si);

    std::vector<vec2d> lrf_pts[3];

    CImgDisplay display_main(900,900,"Main Display",0);
    vector<double> h;

    int step = 0;
    bool update_display = false;
    vec2d target, result;

    while(!display_main.is_closed && !display_main.is_keyESC && step < lrf_log.step) {
        display_main.wait();

        if(display_main.is_keySPACE) {
            lrf_scan_point_from_scan_range(lrf_log.n_lrf[0][step], co, si, lrf_pts[0]);
            lrf_scan_point_from_scan_range(lrf_log.n_lrf[1][step], co, si, lrf_pts[1]);
            lrf_scan_point_from_scan_range(lrf_log.n_lrf[2][step], co, si, lrf_pts[2]);

            lrf_scan_point_to_global_pose(lrf_pts[0], pose2d(140.0, 0.0, 0.0), pose2d());
            lrf_scan_point_to_global_pose(lrf_pts[1], pose2d(200.0, 0.0, DEG2RAD(-2.0)), pose2d());
            lrf_scan_point_to_global_pose(lrf_pts[2], pose2d(-473.0, 0.0, M_PI), pose2d());





            h.clear();

            double t = utils_get_current_time();

            path_planning::vfh::compute_histogram(lrf_pts[0], VFH_A, MIN_RANGE, MAX_RANGE, ANGLE_STEP, h);
//            path_planning::vfh::vfh_draw_histogram_to_cimg(h, img, red, true);
//            PRINTVEC(h);
            path_planning::vfh::compute_histogram(lrf_pts[1], VFH_A, MIN_RANGE, MAX_RANGE, ANGLE_STEP, h);
//            path_planning::vfh::vfh_draw_histogram_to_cimg(h, img, green, true);
//            PRINTVEC(h);
            path_planning::vfh::compute_histogram(lrf_pts[2], VFH_A, MIN_RANGE, MAX_RANGE, ANGLE_STEP, h);
//            path_planning::vfh::vfh_draw_histogram_to_cimg(h, img, yellow, true);
//            PRINTVEC(h);

            path_planning::vfh::smooth_histogram(h, THRESHOLD, 3);
            PRINTVAR(utils_get_current_time() - t);




            update_display = true;
            step++;
            display_main.is_keySPACE = false;
        }




        int mx = display_main.mouse_x;
        int my = display_main.mouse_y;

        if(mx >= 0 && my >= 0) {
            int dimx = display_main.dimx();
            int dimy = display_main.dimy();
            int xoffset = dimx/2;
            int yoffset = dimy/2;
            my = dimy - my;

            mx -= xoffset;
            my -= yoffset;

            target.x = mx;
            target.y = my;
            target *= 1/DISP_SCALE;
//            PRINTVAR(target);

            double t = utils_get_current_time();
            path_planning::vfh::find_open_angle(target, h, THRESHOLD, 10, ANGLE_STEP, result);
            PRINTVAR(utils_get_current_time() - t);

//            PRINTVAR(result);
            update_display = true;
        }

        if(update_display) {
            int dimx = display_main.dimx();
            int dimy = display_main.dimy();
            int xoffset = dimx/2;
            int yoffset = dimy/2;

            CImg<unsigned char> img(display_main.dimx(),display_main.dimy(),1,3,0);
            lrf_draw_scan_point_to_cimg(lrf_pts[0], img, red, true, DISP_SCALE);
            lrf_draw_scan_point_to_cimg(lrf_pts[1], img, green, true, DISP_SCALE);
            lrf_draw_scan_point_to_cimg(lrf_pts[2], img, yellow, true, DISP_SCALE);
            path_planning::vfh::draw_histogram_to_cimg(h, img, cyan, true);

            result *= target.size() * DISP_SCALE;
            int rx = (int)(result.x + xoffset);
            int ry = (int)(-result.y + yoffset);

            img.draw_arrow(xoffset, yoffset, rx, ry, white);


            img.display(display_main);
            update_display = false;
        }

    }


    return 0;
}


#endif
