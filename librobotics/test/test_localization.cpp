/*
 * test_localization.cpp
 *
 *  Created on: Dec 4, 2008
 *      Author: mahisorn
 */

#if 1
#include <iostream>

#include <CImg.h>

#define librobotics_use_cimg
#include "../librobotics.h"
using namespace std;
using namespace librobotics;
using namespace cimg_library;

#define ZOOM_FACTOR 3

const unsigned char
    red[] = { 255,0,0 }, green[] = { 0,255,0 }, blue[] = { 0,0,255 },
    red_drak[] = { 127,0,0 }, green_drak[] = { 0,127,0 }, blue_drak[] = { 0,0,127 },
    cyan[] = {0, 255, 255}, yellow[] = {255, 255, 0},
    cyan_drak[] = {0, 127, 127}, yellow_dark[] = {127, 127, 0},
    black[] = { 0,0,0 },
    gray[] = { 127, 127, 127 },
    white[] = { 255,255,255 };

vec2i disp_map_mouse;
vec2d disp_map_mouse_pose;
vec2i disp_map_z_mouse;
vec2d disp_map_z_mouse_pose;

#define SET_POSE_STATE_NONE     0
#define SET_POSE_STATE_DRAG     1
#define SET_POSE_STATE_ANGLE    2

int set_pose_state = SET_POSE_STATE_NONE;
pose2d set_pose_pose;



localization::mcl_grid2::configuration<double> mcl_cfg;
localization::mcl_grid2::data<double> mcl_data;







void draw_robot(CImg<unsigned char>& img, const pose2d& pose, double size, double zoom) {
    vec2i grid_pose, grid_vec;
    mcl_data.map.get_grid_coordinate(pose.x, pose.y, grid_pose);
    grid_pose *= zoom;
    grid_pose.y = img.dimy() - grid_pose.y;
    img.draw_circle(grid_pose.x, grid_pose.y, (size/mcl_data.map.resolution)*zoom, red);

    vec2d v = vec2d(size,0.0).rot(pose.a) + pose.vec();
    mcl_data.map.get_grid_coordinate(v.x, v.y, grid_vec);
    grid_vec *= zoom;
    grid_vec.y = img.dimy() - grid_vec.y;
    img.draw_arrow(grid_pose.x, grid_pose.y, grid_vec.x, grid_vec.y, black);
}

int main(int argc, char* argv[]) {
    mcl_cfg.load("../test_data/mcl_grid2d.txt");
    mcl_data.initialize(mcl_cfg);
    mcl_data.map.compute_ray_casting_cache(mcl_cfg.map_angle_res);


    CImg<unsigned char> map_image = mcl_data.map.get_image();
    CImg<unsigned char> map_image_z =
        (+map_image).resize(map_image.dimx() * ZOOM_FACTOR, map_image.dimy() * ZOOM_FACTOR);
    CImg<unsigned char> map_image_tmp;

    CImgDisplay disp_map(map_image.dimx(), map_image.dimy(),"Map",0);
    CImgDisplay disp_map_z(map_image_z.dimx(), map_image_z.dimy(),"Map-Zoom",0);

    map_image.display(disp_map);
    map_image_z.display(disp_map_z);

    while(!disp_map.is_closed  && !disp_map.is_keyESC &&
          !disp_map_z.is_closed  && !disp_map_z.is_keyESC)
    {
        CImgDisplay::wait_all();

        //copy tmp image
        map_image_tmp = map_image_z;

        if(disp_map.mouse_x >= 0 && disp_map.mouse_y >= 0) {
            disp_map_mouse.x = disp_map.mouse_x;
            disp_map_mouse.y = disp_map.dimy() - disp_map.mouse_y;
//            PRINTVAR(disp_map_mouse);
            disp_map_mouse_pose = (disp_map_mouse - mcl_data.map.center) * mcl_data.map.resolution;
//            PRINTVAR(disp_map_mouse_pose);
            disp_map.set_title("Map: %6.3f %6.3f", disp_map_mouse_pose.x, disp_map_mouse_pose.y);
        }

        if(disp_map_z.mouse_x >= 0 && disp_map_z.mouse_y >= 0) {
            disp_map_z_mouse.x = disp_map_z.mouse_x;
            disp_map_z_mouse.y = disp_map_z.dimy() - disp_map_z.mouse_y;
//            PRINTVAR(disp_map_z_mouse);
            disp_map_z_mouse_pose = ((disp_map_z_mouse * (1.0/ZOOM_FACTOR)) - mcl_data.map.center) * mcl_data.map.resolution;
//            PRINTVAR(disp_map_z_mouse_pose);
            disp_map_z.set_title("Map-Zoom: %6.3f %6.3f", disp_map_z_mouse_pose.x, disp_map_z_mouse_pose.y);

//            PRINTVAR(disp_map_z.button);
//            PRINTVAR(disp_map_z.key);
//            PRINTVAR(disp_map_z.released_key);
//            PRINTVAR(disp_map_z.buttons[0]);
//            PRINTVAR(disp_map_z.buttons[1]);
//            PRINTVAR(disp_map_z.buttons[2]);

            switch(disp_map_z.button) {
                case 0: //release all btn
                    if(set_pose_state == SET_POSE_STATE_DRAG) {
                        PRINTVAR(set_pose_pose);
                        set_pose_state = SET_POSE_STATE_NONE;
                    }
                    break;
                case 1: //left btn
                    switch(set_pose_state) {
                        case SET_POSE_STATE_NONE:
                            //set position
                            set_pose_pose.x = disp_map_z_mouse_pose.x;
                            set_pose_pose.y = disp_map_z_mouse_pose.y;
                            set_pose_state = SET_POSE_STATE_DRAG;
                            cout << "Set position at: " << set_pose_pose << "\n";
                            draw_robot(map_image_tmp, set_pose_pose, 0.25, ZOOM_FACTOR);
                            map_image_tmp.display(disp_map_z);
                            break;
                        case SET_POSE_STATE_DRAG:
                            vec2d v = disp_map_z_mouse_pose - set_pose_pose.vec();
                            set_pose_pose.a = v.theta();
                            set_pose_state = SET_POSE_STATE_DRAG;
                            cout << "With angle : " << set_pose_pose << "\n";
                            draw_robot(map_image_tmp, set_pose_pose, 0.25, ZOOM_FACTOR);
                            map_image_tmp.display(disp_map_z);
                            break;
                    }
                    break;
                case 2: //right btn
                    {
                        //get measurement from current grid
                        int img_x, img_y;
                        double r;
                        double rad;
                        vec2i grid;
                        if(!mcl_data.map.get_grid_coordinate(disp_map_z_mouse_pose.x, disp_map_z_mouse_pose.y, grid))
                            break;
                        if(mcl_data.map.ray_casting_cache[grid.x][grid.y].size() != 0) {
                            for(int i = 0; i < mcl_data.map.angle_step; i++) {
                                r = mcl_data.map.ray_casting_cache[grid.x][grid.y][i];
                                //if(r > mcl_cfg.z_max_range) continue;

                                r /= mcl_data.map.resolution;
                                rad = mcl_data.map.angle_res * i;
                                img_x = r * cos(rad) * ZOOM_FACTOR;
                                img_y = r * sin(rad) * ZOOM_FACTOR;


                                map_image_tmp.draw_line(disp_map_z.mouse_x,
                                                        disp_map_z.mouse_y,
                                                        disp_map_z_mouse.x + img_x,
                                                        map_image_z.dimy() - (disp_map_z_mouse.y + img_y), red);
                            }
                        }
                        map_image_tmp.draw_circle(disp_map_z.mouse_x, disp_map_z.mouse_y, (int)(mcl_cfg.z_max_range/mcl_cfg.map_res)*ZOOM_FACTOR, green, 1.0, 5);

                        map_image_tmp.display(disp_map_z);
                    }
                    break;
                case 4: //mid btn
                case 3: //left-right btn
                case 5: //left-mid btn
                case 6: //mid-right btn
                case 7: //all btn
                default:
                    break;
            }
        }
    }


    return 0;
}



//#define UNIT 1.0
//#define MEAN_Z (2.5*UNIT)
//#define MAX_Z (5.0*UNIT)
//#define SD_MAP (0.168 * UNIT)
//#define SD_US (0.16 * UNIT)
//#define SD_LRF (0.048 * UNIT)
//#define HIT_MAP_COV SQR(SD_MAP)
//#define HIT_US_COV SQR(sqrt(SQR(SD_US) + SQR(SD_MAP)))
//#define HIT_LRF_COV SQR(sqrt(SQR(SD_LRF) + SQR(SD_MAP)))
//#define SHORT_RATE (0.5 / UNIT)
//
//double motion_cov[6] = { 0.01, 0.001,
//                         0.01, 0.001,
//                         0.01, 0.001 };
//double odo_cov[4] = { 0.05, 0.2,
//                      0.01, 0.01};
//
//
//#define TARGET_X 1.0
//#define TARGET_Y 0.0
//#define TARGET_A 0.0
//#define STARTX -5.0
//#define ENDX 5.0
//#define STARTY -5.0
//#define ENDY 5.0
//#define RES 0.4
//#define U 3.0
//#define W 0.0

//int main(int argc, char** argv) {
//    ofstream log;
//    log.open("log.dat");










//    pose2d u_pt(-1.0, 0.5, 0.0);
//    pose2d u_p(0.0, 0.0, 0.0);
//    pose2d pstart(0.0, 0.0, 0.0);
//
//    pose2d pose;
//    double p;

//    for(int i = 0; i < 100; i++) {
//        pose  = math_model::odometry_motion_sample(u_pt, u_p, pstart, odo_cov);
//        p = math_model::odometry_motion(pose, u_pt, u_p, pstart, odo_cov);
//
//        log << pose << " " << p << "\n";
//    }


//    for(int t = 1; t <= 10; t++) {
//        for(int i = 0; i < 10000; i++) {
//            pose = math_model::velocity_model_sample(vec2d(U, W), pstart, 1.0, motion_cov);
//            p = math_model::velocity_motion(pose,
//                                            vec2d(U, W),
//                                            pstart,
//                                            1.0,
//                                            motion_cov);
//            log << pose << " " << p <<"\n";
//        }
//    }







//    double x, y, a;
//    for(int i = (int)(STARTX / RES); i <= (int)(ENDX / RES); i++) {
//        for(int j = (int)(STARTY / RES); j <= (int)(ENDY / RES); j++) {
//            for(int k = -90; k < 90; k++) {
//                x = i*RES;
//                y = j*RES;
//                a = DEG2RAD(2.0*k);
//                p = math_model::velocity_motion(pose2d(x, y, a),
//                                                vec2d(-2.0, 0.5),
//                                                pose2d(0.0, 0.0, 0.0),
//                                                1.0,
//                                                motion_cov);
//                if(p > 0.1)
//                log << x << " " << y << " " << a << " " << p << "\n";
//            }
//            log << "\n";
//
//        }
//
//    }


//    double x, p_hit_map,  p_hit_us, p_hit_lrf, p_short, p;
//
//
//    cout << exp(1) << endl;
//
//    log.precision(20);
//    double lastp = 0.0;
//    double sum = 0.0;
//    for(int i = 0; i < 1100; i++) {
//        x = i * (MAX_Z/1000.0);
//        //p = stat_pdf_triangular_dist(HIT_MAP_COV, MEAN_Z, x);//stat_pdf_exponential_dist(1.0, x);//stat_pdf_normal_dist(HIT_MAP_COV, MEAN_Z, x);
//        p = math_model::beam_range_finder_measurement(x, MEAN_Z, MAX_Z, HIT_LRF_COV, SHORT_RATE, 1.0, 1.0, 1.0, 1.0);
//        sum += (p + lastp) * 0.5 * (MAX_Z/1000.0);
//        lastp = p;
//        log << x << " " << p << " " << sum << "\n";
//    }


//    for(int i = 0; i < 1100; i++) {
//        x = i * (MAX_Z/1000.0);
//        p_hit_map = stat_pdf_normal_dist(HIT_MAP_COV, MEAN_Z, x);
//        p_hit_us = stat_pdf_normal_dist(HIT_US_COV, MEAN_Z, x);
//        p_hit_lrf = stat_pdf_normal_dist(HIT_LRF_COV, MEAN_Z, x);
//        log << x << " " << p_hit_map << " " << p_hit_us << " " << p_hit_lrf << "\n";
//    }
//    log.close();


//    double p = math_model::beam_range_finder_measurement(0.5, 0.5, 4.0, 0.1, 0.1);
//    PRINTVAR(p);
//    cout << "------------\n";
//    p = math_model::beam_range_finder_measurement(0.4, 0.5, 4.0, 0.1, 0.1);
//    PRINTVAR(p);
//    cout << "------------\n";
//    p = math_model::beam_range_finder_measurement(0.6, 0.5, 4.0, 0.1, 0.1);
//    PRINTVAR(p);
//    cout << "------------\n";
//    p = math_model::beam_range_finder_measurement(3.9, 0.5, 4.0, 0.1, 0.1);
//    PRINTVAR(p);
//    cout << "------------\n";
//    p = math_model::beam_range_finder_measurement(4.0, 0.5, 4.0, 0.1, 0.1);
//    PRINTVAR(p);
//    cout << "------------\n";
//    p = math_model::beam_range_finder_measurement(4.1, 0.5, 4.0, 0.1, 0.1);
//    PRINTVAR(p);
//    cout << "------------\n";
//    p = math_model::beam_range_finder_measurement(0.1, 0.5, 4.0, 0.1, 0.1);
//    PRINTVAR(p);
//    cout << "------------\n";
//    p = math_model::beam_range_finder_measurement(0.0, 0.5, 4.0, 0.1, 0.1);
//    PRINTVAR(p);
//    cout << "------------\n";
//    p = math_model::beam_range_finder_measurement(0.1, 0.5, 4.0, 0.1, 0.1);
//    PRINTVAR(p);
//    cout << "------------\n";
//    p = math_model::beam_range_finder_measurement(0.0, 0.5, 4.0, 0.1, 0.001);
//    PRINTVAR(p);
//    cout << "------------\n";
//    p = math_model::beam_range_finder_measurement(0.04, 0.5, 4.0, 0.1, 1.0);
//    PRINTVAR(p);




//
//    return 0;
//}

/*
int main(int argc, char** argv) {
    map_grid2d map;
    map.load_image("../test_data/map_400_200.png",
                   pose2d(),
                   vec2d(20.0, 10.0),
                   0.1);
    map.print_info();
    map.compute_ray_casting_cache(DEG2RAD(2.0));

    CImgDisplay display_main(500,500,"Main Display",0);
    CImgDisplay display_map(100,100,"Map",0);

    CImg<unsigned char> map_image = map.get_image();
    CImg<unsigned char> map_image_tmp;

    cout << math_model::beam_range_finder_measurement(0.5, 0.4, 1.0, 0.1, 0.1) << endl;

    display_map.resize(map_image);
    map_image.display(display_map);
    while(!display_main.is_closed && !display_main.is_keyESC &&
          !display_map.is_closed && !display_map.is_keyESC)
    {
        CImgDisplay::wait_all();

        if(display_map.mouse_x > -1 && display_map.mouse_y > -1) {
            int map_mouse_x = display_map.mouse_x;
            int map_mouse_y = map_image.dimy() - display_map.mouse_y;
            cout << "map_mouse:" << map_mouse_x << "," << map_mouse_y << "\n";
            vec2d pts;
            if(map.get_real_pts(map_mouse_x, map_mouse_y, pts)) {
                cout << "map_pose:" << pts << " v:" << map.get_grid_value(pts.x, pts.y) << "\n";
            }

            map_image_tmp = map_image;
            int img_x, img_y;
            double r;
            double deg;
            int step = map.ray_casting_cache[map_mouse_x][map_mouse_y].size();
            if(step > 0) {
                double t = utils_get_current_time();
                for(int i = 0; i < step; i++) {
                    r = map.ray_casting_cache[map_mouse_x][map_mouse_y][i];
                    r /= map.resolution;
                    deg = DEG2RAD(2.0) * i;
                    img_x = r * cos(deg);
                    img_y = r * sin(deg);

                    map_image_tmp.draw_line(display_map.mouse_x,
                                            display_map.mouse_y,
                                            map_mouse_x + img_x,
                                            map_image.dimy() - (map_mouse_y + img_y), red);
                }
                cout << (utils_get_current_time() - t) << "\n";
            }
            map_image_tmp.display(display_map);


//            vec2i hit;
//            int result;
//            map_image_tmp = map_image;
//            static const int step = 180;
//            for(int i = 0; i < step; i++) {
//                result = map.get_ray_casting_hit_point(map_mouse_x, map_mouse_y, i * (2.0*M_PI/step), hit);
//                PRINTVAR(result);
//                if(result == 1) {
//                    PRINTVAR(hit);
//                    map_image_tmp.draw_line(display_map.mouse_x, display_map.mouse_y, hit.x, map_image.dimy() - hit.y, red);
//                }
//            }
//            map_image_tmp.display(display_map);

        }
    }
    return 0;
}
*/
/*
    map_grid2d map;
    map.load_txt("../test_data/test_map.txt");
    map.print_info();
//    map.save_txt("../test_data/test_map_save1.txt");

    vec2i v;
    if(map.get_grid_coordinate(0.0, 0.0, v)) {
        PRINTVAR(v);
        PRINTVAR(map.get_grid_value(0.0, 0.0));
        PRINTVAR(map.get_grid_value(1.0, 0.0));
        PRINTVAR(map.get_grid_value(1.0, 1.0));
        PRINTVAR(map.get_grid_value(0.0, 1.0));
        PRINTVAR(map.get_grid_value(-1.0, 1.0));
        PRINTVAR(map.get_grid_value(-1.0, 0.0));
        PRINTVAR(map.get_grid_value(-1.0, -1.0));
        PRINTVAR(map.get_grid_value(0.0, -1.0));
        PRINTVAR(map.get_grid_value(1.0, -1.0));

    } else {
        cout << "outside\n";
    }

    map_grid2d map2;
    map2.load_image("../test_data/map2.bmp",
                 pose2d(),
                 vec2d(5, 5),
                 1.0/10);
    map2.print_info();
    map2.save_txt("../test_data/map2.txt");
    map2.get_image(false, false).save("../test_data/map2_00.jpg");
    map2.get_image(true, false).save("../test_data/map2_10.jpg");
    map2.get_image(true, true).save("../test_data/map2_11.jpg");
    map2.get_image(false, true).save("../test_data/map2_01.jpg");
 */


#endif
