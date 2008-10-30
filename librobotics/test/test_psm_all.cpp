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

#define DISP_SIZE       900
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
    yellow[] = {255, 255, 0}, cyan[] = {0, 255, 255},
    red_drak[] = { 127,0,0 }, green_drak[] = { 0,127,0 }, blue_drak[] = { 0,0,127 },
    cyan_drak[] = {0, 127, 127},
    black[] = { 0,0,0 },
    gray[] = { 127, 127, 127 },
    white[] = { 255,255,255 };


int main(int argc, char* argv[]) {
    if(argc < 2) {
        cout << "Please enter log file" << endl;
        return -1;
    }

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
    logdata_simple_odo_n_lrf<double, int> act_log;
    vector<string> lrf_label;
    lrf_label.push_back("LRF0");
    lrf_label.push_back("LRF1");
    lrf_label.push_back("LRF2");
    ref_log.open("../log/psm_test_all_lrf_open_door/log_all_lrf_open_door.log", NUM_LRF, "Odometry", lrf_label);
    act_log.open(argv[1], NUM_LRF, "Odometry", lrf_label);


    ref_log.read_all();
    act_log.read_all();
    PRINTVAR(ref_log.step);
    PRINTVAR(act_log.step);

    std::vector<int> ref_lrf0(LRF_NUM_RANGE);
    std::vector<int> ref_lrf1(LRF_NUM_RANGE);
    std::vector<int> ref_lrf2(LRF_NUM_RANGE);
    std::vector<vec2d> ref_lrf_pts0;
    std::vector<vec2d> ref_lrf_pts1;
    std::vector<vec2d> ref_lrf_pts2;


    std::vector<int> act_lrf0(LRF_NUM_RANGE);
    std::vector<int> act_lrf1(LRF_NUM_RANGE);
    std::vector<int> act_lrf2(LRF_NUM_RANGE);
    std::vector<vec2d> act_lrf_pts0;
    std::vector<vec2d> act_lrf_pts1;
    std::vector<vec2d> act_lrf_pts2;

    vector<unsigned int> refbad0(LRF_NUM_RANGE);
    vector<unsigned int> refbad1(LRF_NUM_RANGE);
    vector<unsigned int> refbad2(LRF_NUM_RANGE);
    vector<unsigned int> actbad0(LRF_NUM_RANGE);
    vector<unsigned int> actbad1(LRF_NUM_RANGE);
    vector<unsigned int> actbad2(LRF_NUM_RANGE);
    vector<int> refseg0(LRF_NUM_RANGE);
    vector<int> refseg1(LRF_NUM_RANGE);
    vector<int> refseg2(LRF_NUM_RANGE);
    vector<int> actseg0(LRF_NUM_RANGE);
    vector<int> actseg1(LRF_NUM_RANGE);
    vector<int> actseg2(LRF_NUM_RANGE);

    for(size_t i = LRF_START_IDX, j = 0; i < LRF_END_IDX; i++, j++) {
        ref_lrf0[j] = ref_log.n_lrf[0][0][i];
        ref_lrf1[j] = ref_log.n_lrf[1][0][i];
        ref_lrf2[j] = ref_log.n_lrf[2][0][i];
    }


    lrf_range_median_filter(ref_lrf0, MEDIAN_WND);
    lrf_range_median_filter(ref_lrf1, MEDIAN_WND);
    lrf_range_median_filter(ref_lrf2, MEDIAN_WND);
    lrf_range_threshold(ref_lrf0, refbad0, MIN_RANGE, 0, MAX_RANGE, 0);
    lrf_range_threshold(ref_lrf1, refbad1, MIN_RANGE, 0, MAX_RANGE, 0);
    lrf_range_threshold(ref_lrf2, refbad2, MIN_RANGE, 0, MAX_RANGE, 0);
    lrf_range_segment(ref_lrf0, refseg0, SEG_THRES);
    lrf_range_segment(ref_lrf1, refseg1, SEG_THRES);
    lrf_range_segment(ref_lrf2, refseg2, SEG_THRES);

    lrf_scan_point_from_scan_range(ref_lrf0, co_512, si_512, ref_lrf_pts0);
    lrf_scan_point_from_scan_range(ref_lrf1, co_512, si_512, ref_lrf_pts1);
    lrf_scan_point_from_scan_range(ref_lrf2, co_512, si_512, ref_lrf_pts2);


    lrf_scan_point_to_global_pose(ref_lrf_pts0, pose2d(140.0, 0.0, 0.0), pose2d());
    lrf_scan_point_to_global_pose(ref_lrf_pts1, pose2d(200.0, 0.0, 0.0), pose2d());
    lrf_scan_point_to_global_pose(ref_lrf_pts2, pose2d(-473.0, 0.0, M_PI), pose2d());


//    lrf_save_to_file("ref_lrf_pts0.csv", ref_lrf_pts0);

    CImgDisplay display_main(DISP_SIZE,DISP_SIZE,"Main Display",0);
    CImg<unsigned char> img(display_main.dimx(),display_main.dimy(),1,3,0);

//    lrf_draw_scan_point_to_cimg(ref_lrf_pts0,img, red, true);
//    lrf_draw_scan_point_to_cimg(ref_lrf_pts1,img, green, true);
//    lrf_draw_scan_point_to_cimg(ref_lrf_pts2,img, yellow, true);
//    lrf_draw_scan_point_to_cimg(ref_lrf_pts,img, green, true, 0.1, pose2d(100, 0, 0));
//    lrf_draw_scan_point_to_cimg(ref_lrf_pts,img, cyan_drak, true, 0.1, pose2d(-100, 0, 0));
//    lrf_draw_scan_point_to_cimg(ref_lrf_pts,img, blue, true, 0.1, pose2d(100.0, 0.0, M_PI/6));
//    lrf_draw_scan_point_to_cimg(ref_lrf_pts,img, green_drak, true, 0.1, pose2d(100.0, 0.0, -M_PI/6));
    img.display(display_main);
//    img.save("test.png");

    bool run = false;
    int step = 0;
    pose2d relLaserPose0, relRobotPose0;
    pose2d relLaserPose1, relRobotPose1;
    pose2d relLaserPose2, relRobotPose2;
    while(!display_main.is_closed && !display_main.is_keyESC) {
        display_main.wait(10);
        if(display_main.is_resized) {
            display_main.resize();
        }

        if(display_main.is_keySPACE || run) {
            cout << "========== start ==========" << endl;
            PRINTVAR(step);




            cout << "=========== end ===========" << endl;
            PRINTVAR(step);
            if(!run)
                cout << "Press enter to continue..." << endl;
            step++;
            display_main.is_keySPACE = false;
        }

        if(display_main.is_keyR) {
            run = !run;
            display_main.is_keyR = false;
        }

    }



}
