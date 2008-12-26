/*
 * test_device.cpp
 *
 *  Created on: Dec 25, 2008
 *      Author: mahisorn
 */

#if 1



#include <boost/thread/thread.hpp>
#include <iostream>
#include "../librobotics.h"
#include "device/hokuyo_urg.h"

using namespace std;

#define TIME 10000

hokuyo_urg urg[3];

void tr(hokuyo_urg& urg, int id) {
    vector<unsigned int> ranges;
    unsigned int time;
    for(int i = 0; i < TIME; i++) {
        urg.get_data_2char_scip20(0, 768, 1, ranges, time);
        cout << "urg[" << id << "] update: " << time << "\n";
    }
}

using namespace std;

int main(int argc, char* argv[]) {
    PRINTVAR(urg[0].open("/dev/ttyACM0", 115200));
    PRINTVAR(urg[1].open("/dev/ttyACM1", 115200));
    PRINTVAR(urg[2].open("/dev/ttyACM2", 115200));

//    urg[0].enable_scip20();
//    urg[1].enable_scip20();
//    urg[2].enable_scip20();

    urg[0].laser_on_scip20();
    urg[1].laser_on_scip20();
    urg[2].laser_on_scip20();

    boost::thread thrd0(&tr, urg[0], 0);
    boost::thread thrd1(&tr, urg[1], 1);
    boost::thread thrd2(&tr, urg[2], 2);

    thrd0.join();
    thrd1.join();
    thrd2.join();

//    unsigned long start = librobotics::utils_get_current_time();

//    for(int i = 0; i < 10000; i++) {
//
//        PRINTVAR(time);
//        urg[1].get_data_2char_scip20(0, 768, 1, ranges, time);
//        PRINTVAR(time);
//        urg[2].get_data_2char_scip20(0, 768, 1, ranges, time);
//        PRINTVAR(time);
//        PRINTVAR((librobotics::utils_get_current_time() - start));
//    }

    urg[0].laser_off_scip20();
    urg[1].laser_off_scip20();
    urg[2].laser_off_scip20();

    urg[0].close();
    urg[1].close();
    urg[2].close();



    return 0;
}


#endif

