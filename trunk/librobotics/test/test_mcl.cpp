/*
 * test_mcl.cpp
 *
 *  Created on: Feb 2, 2009
 *      Author: mahisorn
 */

#if 1

#include "librobotics.h"

using namespace std;
using namespace librobotics;
using namespace cimg_library;

#define ZOOM 2.0

simple_cimg_gui disp(800, 800, "MAP");
simple_cimg_gui lrf_disp(800, 800, "LRF");


int main(int argc, char* argv[]) {

    lb_mcl_grid2_configuration mcl_cfg;
    mcl_cfg.load("../test_data/mcl_grid2d.txt");

    lb_mcl_grid2_data mcl_data;
    mcl_data.initialize(mcl_cfg);

    lb_init_particle2(mcl_data.p, mcl_data.map, 3, pose2f(5.0, 7.0, -M_PI/3), 1.0, 0.0);

    cimg8u map = mcl_data.map.get_image().resize_doubleXY();
    disp.disp.resize(map);

    lb_draw_paticle2(map, mcl_data.p, mcl_data.map, red, 3, ZOOM, 0.0, 0, 0, true);

    lb_lrf_odo_log<LB_FLOAT> log("Odometry", "Laser");
    if(!log.open("../test_data/cart/log_1229617023.log")) {
        cout << "Cannot open file\n";
    }

    vector<LB_FLOAT> co;
    vector<LB_FLOAT> si;
    vector<LB_FLOAT> range_tmp;
    vector<vec2f> pts;

    lb_build_cos_sin_table(LB_DEG2RAD(-135), LB_DEG2RAD(135), 769, co, si);
    pts.resize(769);



    map.display(disp.disp);
    cimg8u tmp;
    while(!disp.is_closed()) {
        CImgDisplay::wait_all();
        disp.process_event();
        lrf_disp.process_event();

        tmp = map;

        vec2f map_pts;
        mcl_data.map.get_grid_position(disp.mouse.x/ZOOM, disp.mouse.y/ZOOM, map_pts);
        lb_draw_map2_grid_ray_cast(tmp, disp.mouse / 2, mcl_data.map, red, ZOOM, 0.0, 0.0, 0.0);

        tmp.display(disp.disp);

        if(log.real_one_step()) {
            lb_lrf_get_scan_point_from_scan_range(log.ranges, co, si, pts, 0, 768, 1, 0.001);
        }

        cimg8u lrf_img(800, 800, 1, 3, 0);
        lb_draw_points_cimg(lrf_img, pts, red, 100, 0.0);
        lrf_img.display(lrf_disp.disp);


    }



    return 0;
}

#endif
