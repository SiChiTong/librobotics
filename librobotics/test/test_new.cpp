/*
 * test_new.cpp
 *
 *  Created on: Jan 15, 2009
 *      Author: mahisorn
 */

#include "librobotics.h"

using namespace std;
using namespace librobotics;


int main(int argc, char* argv[]) {
    grid2_data grid2dat;


    grid2dat.load_config("../test_data/grid2_cfg.txt");
    grid2dat.load_map_image("../test_data/003_01.png");
    grid2dat.show_information();
//    vec2f pts;
//    vec2i coor;
//    for(int x = 0; x < grid2dat.size.x; x++)
//        for(int y = 0; y < grid2dat.size.y; y++) {
//            grid2dat.get_grid_position(x, y, pts);
//            LB_PRINT_VAR(pts);
//            grid2dat.get_grid_coordinate(pts.x, pts.y, coor);
//            LB_PRINT_VAR(coor);
//        }

    grid2dat.save_config("../test_data/grid2_cfg_save.txt");
    grid2dat.save_map_txt("../test_data/grid2_map_save.txt");

    cimg8u map = grid2dat.get_image(0, 0, 1);

    cimg_library::CImgDisplay disp(map.dimx(), map.dimy(), "Hello", 0);
    map.display(disp);

    while(!disp.is_closed) {
        cimg_library::CImgDisplay::wait_all();

    }


}
