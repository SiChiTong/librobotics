/*
 * test_main.cpp
 *
 *  Created on: Oct 24, 2008
 *      Author: chang
 */

#include <iostream>
#include "../librobotics.h"
using namespace std;
using namespace librobotics;

#if 0

int main() {
    librobotics::info();
//    cout << librobotics::exception_mode() << endl;

//    lrf_data<int, double> lrfdat;


//    try {
//        lrf_init_lrf_data_angle(lrfdat, 0.0, 0.01);
//    } catch(LibRoboticsException& e) {
//        cout << "Hello" << endl;
//    }


//    PRINTVAR(2.0 * stat_pdf_1d(0.1, 1.0, 1.0));
//    PRINTVAR(model::measurement::range_finder_beam_model(1.0, 1.0, 2.0, 0.1, 0.1));
//    PRINTVAR(model::measurement::range_finder_beam_model(1.1, 1.0, 2.0, 0.1, 0.1));
//    PRINTVAR(model::measurement::range_finder_beam_model(0.9, 1.0, 2.0, 0.1, 0.1));
//    PRINTVAR(model::measurement::range_finder_beam_model(2.0, 1.0, 2.0, 0.1, 0.1));

    logdata_simple_odo_lrf<double, int> log;

    try {
        log.open("../log/log_1224772417.log");
        log.read_all();
    } catch (LibRoboticsException& e) {
        cout << e.message << endl;
        return -1;
    }

    PRINTVAR(log.step);

    vector<unsigned int> refbad(769);
    vector<unsigned int> actbad(769);
    vector<int> refseg(769);
    vector<int> actseg(769);
    lrf_psm_cfg psmcfg;


    pose2d last_odo = log.odo[0];
    pose2d current_odo = log.odo[0];
    std::vector<int> last_lrf = log.lrf[0];
    std::vector<int> current_lrf = log.lrf[0];

    vector<double> fi(769);
    vector<double> co(769);
    vector<double> si(769);

    vector<vec2d> last_lrt_pose(769);
    vector<vec2d> curr_lrt_pose(769);
    vector<vec2d> psm_lrt_pose(769);

    lrf_init_fi_table(DEG2RAD(-135), DEG2RAD(0.3515625), fi);
    lrf_init_cos_sin_table(DEG2RAD(-135), DEG2RAD(0.3515625), co, si);


    int step = 0;
    int lastStep = 0;
    char fn[256];
    while(step < log.step) {
        current_odo = log.odo[step];
        if(last_odo.dist_to(current_odo) > 0.3 ||
           fabs(last_odo.angle_to(current_odo)) > DEG2RAD(10))
        {
            cout << "========== start ==========" << endl;

            current_lrf = log.lrf[step];

            PRINTVAR(step);
            PRINTVAR(last_odo);
            PRINTVAR(log.odo[step]);


//            sprintf(fn, "lrf_lastStep_%d_currentStep_%d.csv", lastStep, step);
//            lrf_save_to_file(fn, log.lrf[step], last_lrf);

            //filter laser range
            lrf_range_median_filter(current_lrf);
            lrf_range_threshold(current_lrf, actbad, 500, 0);
            lrf_range_segment(current_lrf, actseg, 100);



//            sprintf(fn, "lrf_before_after.csv");
//            lrf_save_to_file(fn, current_lrf, refbad);

            lrf_range_median_filter(last_lrf);
            lrf_range_threshold(last_lrf, refbad, 500, 0);
            lrf_range_segment(last_lrf, refseg, 100);





            pose2d relLaserPose, relRobotPose;
            lrf_psm(last_odo * 1000,
                    pose2d(140.0, 0.0, 0.0),
                    last_lrf,
                    refbad,
                    refseg,
                    current_odo * 1000,
                    pose2d(140.0, 0.0, 0.0),
                    current_lrf,
                    actbad,
                    actseg,
                    fi, co, si,
                    psmcfg,
                    relLaserPose,
                    relRobotPose);

            PRINTVAR(relLaserPose);
            PRINTVAR(relRobotPose);

            //last pose
            lrf_scan_pose_from_scan_range(last_lrf, co, si, last_lrt_pose);
            lrf_save_to_file("last_lrt_pose.csv", last_lrt_pose);
            lrf_scan_pose_to_global_pose(last_lrt_pose, pose2d(140.0, 0.0, 0.0), last_odo * 1000);
            lrf_save_to_file("last_lrt_pose_move.csv", last_lrt_pose);

            //current pose
            lrf_scan_pose_from_scan_range(current_lrf, co, si, curr_lrt_pose);
            lrf_save_to_file("curr_lrt_pose.csv", curr_lrt_pose);
            lrf_scan_pose_to_global_pose(curr_lrt_pose, pose2d(140.0, 0.0, 0.0), current_odo * 1000);
            lrf_save_to_file("curr_lrt_pose_move.csv", curr_lrt_pose);

            //psm pose
            lrf_scan_pose_from_scan_range(current_lrf, co, si, psm_lrt_pose);
            lrf_save_to_file("psm_lrt_pose.csv", psm_lrt_pose);
            lrf_scan_pose_to_global_pose(psm_lrt_pose, pose2d(140.0, 0.0, 0.0), relLaserPose);
            lrf_save_to_file("psm_lrt_pose_move.csv", psm_lrt_pose);

            return 0;















            cout << "=========== end ===========" << endl;
            last_odo = log.odo[step];
            last_lrf = log.lrf[step];
            lastStep = step;
        }
        step++;
    }









    return 0;
}

#endif

