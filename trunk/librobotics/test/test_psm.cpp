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

int main() {
    logdata_simple_odo_lrf<double, int> ref_log;
    logdata_simple_odo_lrf<double, int> act_log;

    vector<unsigned int> refbad(769);
    vector<unsigned int> actbad(769);
    vector<int> refseg(769);
    vector<int> actseg(769);
    lrf_psm_cfg psmcfg;

    pose2d ref_odo;
    pose2d act_odo;
    std::vector<int> ref_lrf;
    std::vector<int> act_lrf;

    vector<double> fi(769);
    vector<double> co(769);
    vector<double> si(769);

    vector<vec2d> last_lrt_pose(769);
    vector<vec2d> curr_lrt_pose(769);
    vector<vec2d> psm_lrt_pose(769);

    lrf_init_fi_table(DEG2RAD(-135), DEG2RAD(0.3515625), fi);
    lrf_init_cos_sin_table(DEG2RAD(-135), DEG2RAD(0.3515625), co, si);


    try {
        ref_log.open("../log/log_door_open_lower_lrf.log");
        ref_log.read_all();
    } catch (LibRoboticsException& e) {
        cout << e.message << endl;
        return -1;
    }

    PRINTVAR(ref_log.step);



    lrf_save_to_file("door_open_lower_lrf_raw.csv", ref_log.lrf[0]);
    ref_lrf = ref_log.lrf[0];
    lrf_range_median_filter(ref_lrf);
    lrf_range_threshold(ref_lrf, refbad, 500, 0);
    lrf_range_segment(ref_lrf, refseg, 100);
    lrf_save_to_file("door_open_lower_lrf_filtered.csv", ref_lrf);




    return 0;

}
