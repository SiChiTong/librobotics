/*
 * test_psm.cpp
 *
 *  Created on: Oct 28, 2008
 *      Author: chang
 */

#include <iostream>
#include "../librobotics.h"
using namespace std;
using namespace librobotics;

#define MEDIAN_WND 3
#define SEG_THRES   100

int main() {
    logdata_simple_odo_lrf<double, int> ref_log;
    logdata_simple_odo_lrf<double, int> act_log;

    vector<unsigned int> refbad(769);
    vector<unsigned int> actbad(769);
    vector<int> refseg(769);
    vector<int> actseg(769);
    lrf_psm_cfg psmcfg;

    psmcfg.maxError = 1200;
//    psmcfg.maxDriftError = 700;
    psmcfg.searchWndAngle = DEG2RAD(40);
    psmcfg.lrfMaxRange = 3000;
//    psmcfg.lrfMinRange;
    psmcfg.minValidPts = 30;
    psmcfg.maxIter = 40;
    psmcfg.smallCorrCnt = 10;

    pose2d ref_odo;
    pose2d act_odo;
    std::vector<int> ref_lrf;
    std::vector<int> act_lrf;

    vector<double> fi(769);
    vector<double> co(769);
    vector<double> si(769);

    vector<vec2d> ref_lrf_pose(769);
    vector<vec2d> act_lrf_pose(769);
    vector<vec2d> psm_lrf_pose(769);

    lrf_init_fi_table(DEG2RAD(-135), DEG2RAD(0.3515625), fi);
    lrf_init_cos_sin_table(DEG2RAD(-135), DEG2RAD(0.3515625), co, si);


    try {
        ref_log.open("../log/log_door_open_lower_lrf.log");
        act_log.open("../log/log_lower_center_line_0_+30cm.log");
        act_log.read_all();
        ref_log.read_all();
    } catch (LibRoboticsException& e) {
        cout << e.message << endl;
        return -1;
    }

    PRINTVAR(ref_log.step);
    PRINTVAR(act_log.step);



//    lrf_save_to_file("door_open_lower_lrf_raw.csv", ref_log.lrf[0]);
    ref_lrf = ref_log.lrf[0];
    lrf_range_median_filter(ref_lrf, MEDIAN_WND);
    lrf_range_threshold(ref_lrf, refbad, 500, 0, 3000, 0);
    lrf_range_segment(ref_lrf, refseg, SEG_THRES);
//    lrf_save_to_file("door_open_lower_lrf_filtered.csv", ref_lrf);


    int step = 0;
    char fn[256];
    pose2d relLaserPose, relRobotPose;
    while(step < act_log.step) {
        cout << "========== start ==========" << endl;
        PRINTVAR(step);
        PRINTVAR(act_log.odo[step]);



        act_lrf = act_log.lrf[step];
        lrf_range_median_filter(act_lrf, MEDIAN_WND);
        lrf_range_threshold(act_lrf, actbad, 500, 0, 3000, 0);
        lrf_range_segment(act_lrf, actseg, SEG_THRES);

        bool ok = lrf_psm(pose2d(0.0, 0.0, 0.0),
                          pose2d(140.0, 0.0, 0.0),
                          ref_lrf,
                          refbad,
                          refseg,
                          pose2d(0.0, 0.0, 0.0),
                          //act_log.odo[step] * 1000.0,
                          pose2d(140.0, 0.0, 0.0),
                          act_lrf,
                          actbad,
                          actseg,
                          fi, co, si,
                          psmcfg,
                          relLaserPose,
                          relRobotPose,
                          true);    //force check

        if(ok) {
            PRINTVAR(relLaserPose);
        }

        //ref pose
        lrf_scan_pose_from_scan_range(ref_lrf, co, si, ref_lrf_pose);
        lrf_save_to_file("ref_lrf_pose.csv", ref_lrf_pose);
        lrf_scan_pose_to_global_pose(ref_lrf_pose, pose2d(140.0, 0.0, 0.0), pose2d());
        lrf_save_to_file("ref_lrf_pose_move.dat", ref_lrf_pose, " ");

        //act pose
        lrf_scan_pose_from_scan_range(act_lrf, co, si, act_lrf_pose);
        lrf_save_to_file("act_lrf_pose.csv", act_lrf_pose);
        //lrf_scan_pose_to_global_pose(act_lrf_pose, pose2d(140.0, 0.0, 0.0), act_log.odo[step] * 1000);
        lrf_scan_pose_to_global_pose(act_lrf_pose, pose2d(140.0, 0.0, 0.0), pose2d());
        lrf_save_to_file("act_lrf_pose_move.dat", act_lrf_pose, " ");

        //psm pose
        lrf_scan_pose_from_scan_range(act_lrf, co, si, psm_lrf_pose);
        lrf_save_to_file("psm_lrf_pose.csv", psm_lrf_pose);
        lrf_scan_pose_to_global_pose(psm_lrf_pose, pose2d(140.0, 0.0, 0.0), relLaserPose);
        lrf_save_to_file("psm_lrf_pose_move.dat", psm_lrf_pose, " ");



        cout << "=========== end ===========" << endl;
        PRINTVAR(step);
        cout << "Press enter to continue..." << endl;
        getchar();

        step++;
    }




    return 0;

}
