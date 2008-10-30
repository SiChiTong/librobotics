/*
 * test_psm_all.cpp
 *
 *  Created on: Oct 30, 2008
 *      Author: chang
 */

#include <iostream>

#include <CImg.h>

#define librobotics_use_cimg
#include "../librobotics.h"
using namespace std;
using namespace librobotics;
using namespace cimg_library;

#define NUM_LRF         3
#define MEDIAN_WND      2
#define SEG_THRES       100
#define MAX_RANGE       4000
#define MIN_RANGE       400

#define LRF_START_IDX   128
#define LRF_END_IDX     640
#define LRF_NUM_RANGE   (LRF_END_IDX - LRF_START_IDX)

const unsigned char
    red[] = { 255,0,0 }, green[] = { 0,255,0 }, blue[] = { 0,0,255 },
    cyan[] = {0, 255, 255},
    red_drak[] = { 127,0,0 }, green_drak[] = { 0,127,0 }, blue_drak[] = { 0,0,127 },
    cyan_drak[] = {0, 127, 127},
    black[] = { 0,0,0 },
    gray[] = { 127, 127, 127 },
    white[] = { 255,255,255 };


int main(int argc, char* argv[]) {
//    if(argc < 2) {
//        cout << "Please enter log file" << endl;
//        return -1;
//    }

    vector<double> fi_769(769);
    vector<double> co_769(769);
    vector<double> si_769(769);
    vector<double> fi_512(512);
    vector<double> co_512(512);
    vector<double> si_512(512);

    lrf_init_fi_table(DEG2RAD(-135), DEG2RAD(0.3515625), fi_769);
    lrf_init_cos_sin_table(DEG2RAD(-135), DEG2RAD(0.3515625), co_769, si_769);
    lrf_init_fi_table(DEG2RAD(-90), DEG2RAD(0.3515625), fi_512);
    lrf_init_cos_sin_table(DEG2RAD(-90), DEG2RAD(0.3515625), co_512, si_512);

    logdata_simple_odo_n_lrf<double, int> ref_log;
    vector<string> lrf_label;
    lrf_label.push_back("LRF0");
    lrf_label.push_back("LRF1");
    lrf_label.push_back("LRF2");
    ref_log.open("../log/psm_test_all_lrf_open_door/log_all_lrf_open_door.log", NUM_LRF, "Odometry", lrf_label);


    ref_log.read_all();
    PRINTVAR(ref_log.step);

    std::vector<int> ref_lrf(LRF_NUM_RANGE);
    std::vector<int> ref_lrf2(LRF_NUM_RANGE);
    std::vector<vec2d> ref_lrf_pts;

    for(size_t i = LRF_START_IDX, j = 0; i < LRF_END_IDX; i++, j++) {
        ref_lrf[j] = ref_log.n_lrf[0][0][i];
    }

    lrf_range_median_filter(ref_lrf, MEDIAN_WND);
//    lrf_range_check(ref_lrf, 0, 4000, 3000, 19, 0);

    lrf_scan_point_from_scan_range(ref_lrf, co_512, si_512, ref_lrf_pts);
//    lrf_scan_pose_to_global_pose(ref_lrf_pts, pose2d(140, 0, 0), pose2d());


    lrf_save_to_file("ref_lrf_pts.csv", ref_lrf_pts);

    CImgDisplay display_main(800,800,"Main Display",0);
    CImg<unsigned char> img(800,800,1,3,0);

    while(!display_main.is_closed && !display_main.is_keyESC) {
        display_main.wait(10);
        lrf_draw_scan_point_to_cimg(ref_lrf_pts,img, red, 0.1, true);
        img.display(display_main);


    }



}
