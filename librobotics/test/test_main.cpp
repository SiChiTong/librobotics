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
    PRINTVAR(model::measurement::range_finder_beam_model(1.0, 1.0, 2.0, 0.1, 0.1));
    PRINTVAR(model::measurement::range_finder_beam_model(1.1, 1.0, 2.0, 0.1, 0.1));
    PRINTVAR(model::measurement::range_finder_beam_model(0.9, 1.0, 2.0, 0.1, 0.1));
    PRINTVAR(model::measurement::range_finder_beam_model(2.0, 1.0, 2.0, 0.1, 0.1));


    return 0;
}

