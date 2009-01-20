/*
 * test_new.cpp
 *
 *  Created on: Jan 15, 2009
 *      Author: mahisorn
 */


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

    lb_build_cos_sin_table(LB_DEG2RAD(-135), LB_DEG2RAD(135), 769, co, si);

//    while(log.real_one_step()) {
//        LB_PRINT_VEC(log.ranges);
//    }

    log.real_all(range_all);

    LB_PRINT_VAR(range_all.size());
    pts_all.resize(log.step);

    for(int i = 0; i < log.step; i++) {
        pts_all[i].resize(769);
        lb_lrf_range_threshold_filter(range_all[i], 500);
//        lb_lrf_range_median_filter(range_all[i]);
        lb_lrf_get_scan_point_from_scan_range(range_all[i], co, si, pts_all[i], 0, 768, 1, 0.001);
        //lrf_scan_point_offset(pts_all[i], pose2f(-0.1, 0.3, -M_PI/4), pose2f(-0.5, 0.5, M_PI/2));
    }


    int n_segment = 0;
    int n_line = 0;
    bool found_arc;
    vector<vector<vec2f> > segments;
    vector<lrf_object> objects;
    size_t cnt = 800;
    vector<int> seg;
    while(!disp.is_closed()) {
        cimg_library::CImgDisplay::wait_all();

        if(disp.disp.is_keySPACE) {
            if(cnt < pts_all.size()) {
                LB_PRINT_VAL("=========================");
                LB_PRINT_VAR(cnt);
                cimg8u img(800, 800, 1, 3, 0);
                img.draw_axis(-399, 399, 399, -399, green);
                //draw_points_cimg(img, pts_all[cnt], red, 80.0, 0.0, -1, -1, true);

                //segment
                n_segment = lb_lrf_range_segment(range_all[cnt], seg, 100);
                if(n_segment > 0) {
                    //objects.clear();
                    lb_lrf_create_segment(pts_all[cnt], seg, segments);

                    LB_PRINT_VAR(n_segment);
//                    LB_PRINT_VAR(segments.size());

                    for(size_t i = 0; i <  segments.size(); i++) {
                        lb_draw_points_cimg(img, segments[i], red, 80.0, 0.0, -1, -1, true);
                        n_line = lb_lrf_recusive_line_fitting(segments[i], objects, 0.2, 0.1, i);
                        LB_PRINT_VAR(n_line);

                        if(n_line > 1) {
                            for(int lc = 0; lc < n_line; lc++) {
                                found_arc = lb_lrf_arc_fiting(objects[lc].points,
                                                              objects,
                                                              1.57,    //min_aparture
                                                              2.8,     //max_aparture
                                                              0.5,     //max_stdev
                                                              0.1,     //arc_ratio
                                                              0.1,     //is_line_error
                                                              0.2);    //is_line_stdev
                                LB_PRINT_VAR(found_arc);
                            }
                        } else {
                            found_arc = lb_lrf_arc_fiting(segments[i],
                                                          objects,
                                                          1.57,    //min_aparture
                                                          2.8,     //max_aparture
                                                          0.5,     //max_stdev
                                                          0.1,     //arc_ratio
                                                          0.1,     //is_line_error
                                                          0.2);    //is_line_stdev
                            LB_PRINT_VAR(found_arc);
                        }

                        LB_PRINT_VAR(found_arc);
                    }
                }

                for(size_t l = 0; l < objects.size(); l++) {
                    switch(objects[l].type) {
                        case LRF_OBJ_LINE:
                            lb_draw_lrf_line_object_cimg(img, objects[l], white, 80);
                            break;
                        case LRF_OBJ_ARC:
                            lb_draw_lrf_arc_object_cimg(img, objects[l], green_drak, 80);
                            break;
                        default:
                            break;
                    }
                }
                objects.clear();


                img.display(disp.disp);
            }
            cnt++;
            disp.disp.is_keySPACE = false;
        }

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
