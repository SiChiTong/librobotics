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
    cout << librobotics::exception_mode() << endl;

    lrf_data<int, double> lrfdat;


    try {
        lrf_init_lrf_data_angle(lrfdat, 0.0, 0.01);
    } catch(LibRoboticsException& e) {
        cout << "Hello" << endl;
    }



    return 0;
}

