/*
 * test_psm_all.cpp
 *
 *  Created on: Oct 30, 2008
 *      Author: chang
 */

#if 1
#include <iostream>

#include <CImg.h>

#define librobotics_use_cimg
#include "../librobotics.h"
using namespace std;
using namespace librobotics;
using namespace cimg_library;


#define NUM_LRF         3
#define MEDIAN_WND      3
#define SEG_THRES       100
#define MAX_RANGE       4000
#define MIN_RANGE       400

#define DISP_MAX_RANGE  4500
#define DISP_SCALE      0.05
#define DISP_SIZE       ((int)(DISP_MAX_RANGE * DISP_SCALE * 2))

#define LRF_START_IDX   128
#define LRF_END_IDX     640
#define LRF_NUM_RANGE   ((LRF_END_IDX - LRF_START_IDX) + 1)


const unsigned char
    red[] = { 255,0,0 }, green[] = { 0,255,0 }, blue[] = { 0,0,255 },
    cyan[] = {0, 255, 255}, yellow[] = {255, 255, 0},
    red_drak[] = { 127,0,0 }, green_drak[] = { 0,127,0 }, blue_drak[] = { 0,0,127 },
    cyan_drak[] = {0, 127, 127}, yellow_dark[] = {127, 127, 0},
    black[] = { 0,0,0 },
    gray[] = { 127, 127, 127 },
    white[] = { 255,255,255 };


int main(int argc, char* argv[]) {
    if(argc < 2) {
        cout << "Please enter log file" << endl;
        return -1;
    }

    vector<double> fi(LRF_NUM_RANGE);
    vector<double> co(LRF_NUM_RANGE);
    vector<double> si(LRF_NUM_RANGE);

    cout << LRF_NUM_RANGE << endl;


    lrf_init_fi_table(DEG2RAD(-90), DEG2RAD(0.3515625), fi);
    lrf_init_cos_sin_table(DEG2RAD(-90), DEG2RAD(0.3515625), co, si);

    logdata_simple_odo_n_lrf<double, int> ref_log;
    logdata_simple_odo_n_lrf<double, int> act_log;
    vector<string> lrf_label;
    lrf_label.push_back("LRF0");
    lrf_label.push_back("LRF1");
    lrf_label.push_back("LRF2");
    ref_log.open("../log/psm_test_all_lrf_open_door/ref_all_lrf_open_door.log", NUM_LRF, "Odometry", lrf_label);
    act_log.open(argv[1], NUM_LRF, "Odometry", lrf_label);


    try {
        ref_log.read_all();
        act_log.read_all();
    } catch (...) {
        return -1;
    }
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

    std::vector<vec2d> act_lrf_psm_pts0;
    std::vector<vec2d> act_lrf_psm_pts1;
    std::vector<vec2d> act_lrf_psm_pts2;

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

//    for(size_t i = LRF_START_IDX, j = 0; i < LRF_END_IDX; i++, j++) {
//        ref_lrf0[j] = ref_log.n_lrf[0][0][i];
//        ref_lrf1[j] = ref_log.n_lrf[1][0][i];
//        ref_lrf2[j] = ref_log.n_lrf[2][0][i];
//    }

    ref_lrf0 = ref_log.n_lrf[0][0];
    ref_lrf1 = ref_log.n_lrf[1][0];
    ref_lrf2 = ref_log.n_lrf[2][0];


    lrf_range_median_filter(ref_lrf0, MEDIAN_WND);
    lrf_range_median_filter(ref_lrf1, MEDIAN_WND);
    lrf_range_median_filter(ref_lrf2, MEDIAN_WND);
    lrf_range_threshold(ref_lrf0, refbad0, MIN_RANGE, 0, MAX_RANGE, 0);
    lrf_range_threshold(ref_lrf1, refbad1, MIN_RANGE, 0, MAX_RANGE, 0);
    lrf_range_threshold(ref_lrf2, refbad2, MIN_RANGE, 0, MAX_RANGE, 0);
    lrf_range_segment(ref_lrf0, refseg0, SEG_THRES);
    lrf_range_segment(ref_lrf1, refseg1, SEG_THRES);
    lrf_range_segment(ref_lrf2, refseg2, SEG_THRES);


//    lrf_save_to_file("ref_lrf0.csv", ref_lrf0);
//    lrf_save_to_file("ref_lrf1.csv", ref_lrf1);
//    lrf_save_to_file("ref_lrf2.csv", ref_lrf2);

    lrf_scan_point_from_scan_range(ref_lrf0, co, si, ref_lrf_pts0);
    lrf_scan_point_from_scan_range(ref_lrf1, co, si, ref_lrf_pts1);
    lrf_scan_point_from_scan_range(ref_lrf2, co, si, ref_lrf_pts2);

    //lrf_scan_point_to_global_pose(ref_lrf_pts0, pose2d(140.0, 0.0, 0.0), pose2d());
    //lrf_scan_point_to_global_pose(ref_lrf_pts1, pose2d(200.0, 0.0, 0.0), pose2d());
    //lrf_scan_point_to_global_pose(ref_lrf_pts2, pose2d(-473.0, 0.0, M_PI), pose2d());


//    lrf_save_to_file("ref_lrf_pts0.csv", ref_lrf_pts0);
    CImgDisplay display_main(DISP_SIZE,DISP_SIZE,"Main Display",0);
    CImgDisplay display_lrf0(DISP_SIZE,DISP_SIZE,"LRF0 Display",0);
    CImgDisplay display_lrf1(DISP_SIZE,DISP_SIZE,"LRF1 Display",0);
    CImgDisplay display_lrf2(DISP_SIZE,DISP_SIZE,"LRF2 Display",0);
//    CImg<unsigned char> img(display_main.dimx(),display_main.dimy(),1,3,0);

//    lrf_draw_scan_point_to_cimg(ref_lrf_pts0,img, red, true);
//    lrf_draw_scan_point_to_cimg(ref_lrf_pts1,img, green, true);
//    lrf_draw_scan_point_to_cimg(ref_lrf_pts2,img, yellow, true);
//    lrf_draw_scan_point_to_cimg(ref_lrf_pts,img, green, true, 0.1, pose2d(100, 0, 0));
//    lrf_draw_scan_point_to_cimg(ref_lrf_pts,img, cyan_drak, true, 0.1, pose2d(-100, 0, 0));
//    lrf_draw_scan_point_to_cimg(ref_lrf_pts,img, blue, true, 0.1, pose2d(100.0, 0.0, M_PI/6));
//    lrf_draw_scan_point_to_cimg(ref_lrf_pts,img, green_drak, true, 0.1, pose2d(100.0, 0.0, -M_PI/6));
//    img.display(display_main);
//    img.save("test.png");

    bool run = false;
    bool display_act = false;
    int step = 0;

    lrf_psm_cfg psmcfg;

    psmcfg.maxError = 1200;
//    psmcfg.maxDriftError = 700;
    psmcfg.searchWndAngle = DEG2RAD(30);
    psmcfg.lrfMaxRange = MAX_RANGE + 1000;
//    psmcfg.lrfMinRange;
    psmcfg.minValidPts = 30;
    psmcfg.maxIter = 40;
    psmcfg.smallCorrCnt = 10;

    pose2d relLaserPose0, relRobotPose0;
    pose2d relLaserPose1, relRobotPose1;
    pose2d relLaserPose2, relRobotPose2;
    bool ok0, ok1, ok2;
    while(!display_main.is_closed && !display_main.is_keyESC && step < act_log.step) {
        if(run) {
            display_main.wait(10);
        } else {
            display_main.wait();
        }

        if(display_main.is_resized) {
            display_main.resize();
        }

        if(display_lrf0.is_resized) {
            display_lrf0.resize();
        }

        if(display_lrf1.is_resized) {
            display_lrf1.resize();
        }

        if(display_lrf2.is_resized) {
            display_lrf2.resize();
        }


        if(display_main.is_keySPACE || run) {
            cout << "========== start ==========" << endl;
            PRINTVAR(step);

            if(LRF_NUM_RANGE < 769) {
                for(size_t i = LRF_START_IDX, j = 0; i < LRF_END_IDX; i++, j++) {
                    act_lrf0[j] = act_log.n_lrf[0][step][i];
                    act_lrf1[j] = act_log.n_lrf[1][step][i];
                    act_lrf2[j] = act_log.n_lrf[2][step][i];
                }
            } else {

                act_lrf0 = act_log.n_lrf[0][step];
                act_lrf1 = act_log.n_lrf[1][step];
                act_lrf2 = act_log.n_lrf[2][step];
            }

            //filter and check
            lrf_range_median_filter(act_lrf0, MEDIAN_WND);
            lrf_range_median_filter(act_lrf1, MEDIAN_WND);
            lrf_range_median_filter(act_lrf2, MEDIAN_WND);

            lrf_range_threshold(act_lrf0, actbad0, MIN_RANGE, 0, MAX_RANGE, 0);
            lrf_range_threshold(act_lrf1, actbad1, MIN_RANGE, 0, MAX_RANGE, 0);
            lrf_range_threshold(act_lrf2, actbad2, MIN_RANGE, 0, MAX_RANGE, 0);

            lrf_range_segment(act_lrf0, actseg0, SEG_THRES);
            lrf_range_segment(act_lrf1, actseg1, SEG_THRES);
            lrf_range_segment(act_lrf2, actseg2, SEG_THRES);


            if(!display_act) {
                lrf_scan_point_from_scan_range(act_lrf0, co, si, act_lrf_pts0);
                lrf_scan_point_from_scan_range(act_lrf1, co, si, act_lrf_pts1);
                lrf_scan_point_from_scan_range(act_lrf2, co, si, act_lrf_pts2);
            }

            CImg<unsigned char> img_act0(display_lrf0.dimx(),display_lrf0.dimy(),1,3,0);
            CImg<unsigned char> img_act1(display_lrf1.dimx(),display_lrf1.dimy(),1,3,0);
            CImg<unsigned char> img_act2(display_lrf2.dimx(),display_lrf2.dimy(),1,3,0);

            lrf_draw_scan_point_to_cimg(ref_lrf_pts0, img_act0, red, true, DISP_SCALE);
            lrf_draw_scan_point_to_cimg(ref_lrf_pts1, img_act1, green, true, DISP_SCALE);
            lrf_draw_scan_point_to_cimg(ref_lrf_pts2, img_act2, yellow, true, DISP_SCALE);

            //            img_act.save("img_act.png", step);


//            CImg<unsigned char> img1(display_main.dimx(),display_main.dimy(),1,3,0);
//            CImg<unsigned char> img2(display_main.dimx(),display_main.dimy(),1,3,0);




            ok0 = lrf_psm(pose2d(0.0, 0.0, 0.0),
                         pose2d(0.0, 0.0, 0.0),
                         ref_lrf0,
                         refbad0,
                         refseg0,
                         pose2d(0.0, 0.0, 0.0),
                         pose2d(0.0, 0.0, 0.0),
                         act_lrf0,
                         actbad0,
                         actseg0,
                         fi, co, si,
                         psmcfg,
                         relLaserPose0,
                         relRobotPose0,
                         true);    //force check
//             if(ok) {
//                 PRINTVAR(relLaserPose0);
//             }


             ok1 = lrf_psm(pose2d(0.0, 0.0, 0.0),
                          pose2d(0.0, 0.0, 0.0),
                          ref_lrf1,
                          refbad1,
                          refseg1,
                          pose2d(0.0, 0.0, 0.0),
                          pose2d(0.0, 0.0, 0.0),
                          act_lrf1,
                          actbad1,
                          actseg1,
                          fi, co, si,
                          psmcfg,
                          relLaserPose1,
                          relRobotPose1,
                          true);    //force check
//             if(ok) {
//                 PRINTVAR(relLaserPose1);
//             }

             ok2 = lrf_psm(pose2d(0.0, 0.0, 0.0),
                          pose2d(0.0, 0.0, 0.0),
                          ref_lrf2,
                          refbad2,
                          refseg2,
                          pose2d(0.0, 0.0, 0.0),
                          pose2d(0.0, 0.0, 0.0),
                          act_lrf2,
                          actbad2,
                          actseg2,
                          fi, co, si,
                          psmcfg,
                          relLaserPose2,
                          relRobotPose2,
                          true);    //force check
//             if(ok) {
//                 PRINTVAR(relLaserPose2);
//             }

             lrf_scan_point_to_global_pose(act_lrf_pts0, pose2d(0.0, 0.0, 0.0), relLaserPose0);
             lrf_scan_point_to_global_pose(act_lrf_pts1, pose2d(0.0, 0.0, 0.0), relLaserPose1);
             lrf_scan_point_to_global_pose(act_lrf_pts2, pose2d(0.0, 0.0, 0.0), relLaserPose2);

             lrf_draw_scan_point_to_cimg(act_lrf_pts0, img_act0, green, true, DISP_SCALE);
             lrf_draw_scan_point_to_cimg(act_lrf_pts1, img_act1, red, true, DISP_SCALE);
             lrf_draw_scan_point_to_cimg(act_lrf_pts2, img_act2, cyan, true, DISP_SCALE);

             PRINTVAR(relLaserPose0);
             PRINTVAR(relLaserPose1);
             PRINTVAR(relLaserPose2);

             CImg<unsigned char> img(display_main.dimx(),display_main.dimy(),1,3,0);
             if(MIN2_BOOL(fabs(relLaserPose0.a), fabs(relLaserPose1.a))) {
                 if(MIN2_BOOL(fabs(relLaserPose0.a), fabs(relLaserPose2.a))) {
                     cout << "0 min" << endl;
                     lrf_draw_scan_point_to_cimg(ref_lrf_pts0, img, red, true, DISP_SCALE);
                     lrf_draw_scan_point_to_cimg(act_lrf_pts0, img, green, true, DISP_SCALE);

                 } else {
                     cout << "2 min" << endl;
                     lrf_draw_scan_point_to_cimg(ref_lrf_pts2, img, red, true, DISP_SCALE);
                     lrf_draw_scan_point_to_cimg(act_lrf_pts2, img, cyan, true, DISP_SCALE);
                 }
             } else {
                 if(MIN2_BOOL(fabs(relLaserPose1.a), fabs(relLaserPose2.a))) {
                     cout << "1 min" << endl;
                     lrf_draw_scan_point_to_cimg(ref_lrf_pts1, img, green, true, DISP_SCALE);
                     lrf_draw_scan_point_to_cimg(act_lrf_pts1, img, red, true, DISP_SCALE);
                 } else {
                     cout << "2 min" << endl;
                     lrf_draw_scan_point_to_cimg(ref_lrf_pts2, img, yellow, true, DISP_SCALE);
                     lrf_draw_scan_point_to_cimg(act_lrf_pts2, img, cyan, true, DISP_SCALE);
                 }
             }



             img_act0.display(display_lrf0);
             img_act1.display(display_lrf1);
             img_act2.display(display_lrf2);
             img.display(display_main);



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

        if(display_main.is_keyA) {
            display_act = !display_act;
            display_main.is_keyA = false;
        }

    }

    //cout << "Press enter to exit..." << endl;
    //getchar();


}

#endif
