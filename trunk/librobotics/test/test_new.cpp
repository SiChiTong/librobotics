/*
 * test_new.cpp
 *
 *  Created on: Jan 15, 2009
 *      Author: mahisorn
 */

#include "device/Joystick.h"
#include "librobotics.h"

using namespace std;
using namespace librobotics;
using namespace cimg_library;

const unsigned char
    red[] = { 255,0,0 }, green[] = { 0,255,0 }, blue[] = { 0,0,255 },
    red_drak[] = { 127,0,0 }, green_drak[] = { 0,127,0 }, blue_drak[] = { 0,0,127 },
    cyan[] = {0, 255, 255}, yellow[] = {255, 255, 0}, margenta[] = {255, 0, 255},
    cyan_drak[] = {0, 127, 127}, yellow_dark[] = {127, 127, 0},
    black[] = { 0,0,0 },
    gray[] = { 127, 127, 127 },
    white[] = { 255,255,255 };


int main(int argc, char* argv[]) {




    simple_cimg_gui disp(800, 800, "hello");

    simple_lrf_log log("Laser");
    if(!log.open("../test_data/log_1224675697.log"))
        LB_PRINT_VAL("cannot open");

    vector<LB_FLOAT> co;
    vector<LB_FLOAT> si;
    vector<vector<int> > range_all;
    vector<vector<vec2f> > pts_all;

    build_cos_sin_table(LB_DEG2RAD(-135), LB_DEG2RAD(135), 769, co, si);

//    while(log.real_one_step()) {
//        LB_PRINT_VEC(log.ranges);
//    }

    log.real_all(range_all);

    LB_PRINT_VAR(range_all.size());
    pts_all.resize(log.step);

    for(int i = 0; i < log.step; i++) {
        pts_all[i].resize(769);
        lrf_scan_point_from_scan_range(range_all[i], co, si, pts_all[i], 0, 768, 1, 0.001);
    }



    size_t cnt = 0;
    while(!disp.is_closed()) {
        cimg_library::CImgDisplay::wait_all();

        if(cnt < pts_all.size()) {
            cimg8u img(800, 800, 1, 3, 0);
            draw_lrf_point_cimg(img, pts_all[cnt], red, 100.0);
            img.display(disp.disp);
        }



        cnt++;

    }



//    grid2_data grid2dat;
//
//
//    grid2dat.load_config("../test_data/grid2_cfg_save.txt");
//    grid2dat.load_map_txt("../test_data/grid2_map_save.txt");
//    grid2dat.show_information();
////    vec2f pts;
////    vec2i coor;
////    for(int x = 0; x < grid2dat.size.x; x++)
////        for(int y = 0; y < grid2dat.size.y; y++) {
////            grid2dat.get_grid_position(x, y, pts);
////            LB_PRINT_VAR(pts);
////            grid2dat.get_grid_coordinate(pts.x, pts.y, coor);
////            LB_PRINT_VAR(coor);
////        }
//
//    //grid2dat.save_config("../test_data/grid2_cfg_save.txt");
//    //grid2dat.save_map_txt("../test_data/grid2_map_save.txt");
//
//    simple_cimg_gui map_disp;
//    cimg8u map = grid2dat.get_image(1, 0, 0);
//
//    //cimg_library::CImgDisplay disp(map.dimx(), map.dimy(), "Hello", 0);
//    //map.display(disp);
//
//    map.display(map_disp.disp);
//
//    while(!map_disp.is_closed()) {
//        cimg_library::CImgDisplay::wait_all();
//        map_disp.process_event( );
//
//        LB_PRINT_VAR(map_disp.get_zoom_pose());
//
//    }


}
