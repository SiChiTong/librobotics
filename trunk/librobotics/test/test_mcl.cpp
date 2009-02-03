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

simple_cimg_gui disp(800, 800, "hello");


int main(int argc, char* argv[]) {

    lb_mcl_grid2_configuration mcl_cfg;
    mcl_cfg.load("../test_data/mcl_grid2d.txt");

    lb_mcl_grid2_data mcl_data;
    mcl_data.initialize(mcl_cfg);

    lb_init_particle2(mcl_data.p, mcl_data.map, 2, pose2f(5.0, 7.0, -M_PI/3), 1.0, 0.0);

    cimg8u map = mcl_data.map.get_image().resize_doubleXY();
    disp.disp.resize(map);

    lb_draw_paticle2(map, mcl_data.p, mcl_data.map, red, 3, ZOOM, 0.0, 0, 0, true);

    mcl_data.map.compute_ray_casting_cache(mcl_cfg.map_angle_res, 0.1);


    map.display(disp.disp);
    cimg8u map_tmp;
    while(!disp.is_closed()) {
        CImgDisplay::wait_all();
        disp.process_event();

        map_tmp = map;

        vec2f map_pts;
        mcl_data.map.get_grid_position(disp.mouse.x/ZOOM, disp.mouse.y/ZOOM, map_pts);
        lb_draw_map2_grid_ray_cast(map_tmp, disp.mouse / 2, mcl_data.map, red, ZOOM, 0.0, 0.0, 0.0);





        map_tmp.display(disp.disp);
    }



    return 0;
}

#endif
