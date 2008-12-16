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

#define ZOOM_FACTOR 2

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
bool update_map_z_display = false;

#define MEDIAN_WND      3
#define SEG_THRES       100
#define MAX_RANGE       4000
#define MIN_RANGE       400
#define SET_POSE_STATE_NONE     0
#define SET_POSE_STATE_DRAG     1
#define SET_POSE_STATE_ANGLE    2

int set_pose_state = SET_POSE_STATE_NONE;
pose2d set_pose_pose;

#define LRF_START_IDX   0
#define LRF_END_IDX     768
#define LRF_NUM_RANGE   ((LRF_END_IDX - LRF_START_IDX) + 1)
vector<double> fi(LRF_NUM_RANGE);
vector<double> co(LRF_NUM_RANGE);
vector<double> si(LRF_NUM_RANGE);
logdata_simple_odo_lrf<double, int> log_data;
vector<int> lrf(LRF_NUM_RANGE);
vector<vec2d> lrf_pts;
int log_step = 0;
int last_log_step = 0;

bool run = false;
bool single_step = false;
localization::mcl_grid2::configuration<double> mcl_cfg;
localization::mcl_grid2::data<double> mcl_data;
vector<vec2d> measurement_z;




void draw_robot(CImg<unsigned char>& img, const pose2d& pose, double size, double zoom) {
    vec2i grid_pose, grid_vec;
    mcl_data.map.get_grid_coordinate(pose.x, pose.y, grid_pose);
    grid_pose *= zoom;
    grid_pose.y = img.dimy() - grid_pose.y;
    img.draw_circle(grid_pose.x, grid_pose.y, (size/mcl_data.map.resolution)*zoom, red, 0.1);

    vec2d v = vec2d(size*1.2,0.0).rot(pose.a) + pose.vec();
    mcl_data.map.get_grid_coordinate(v.x, v.y, grid_vec);
    grid_vec *= zoom;
    grid_vec.y = img.dimy() - grid_vec.y;
    img.draw_arrow(grid_pose.x, grid_pose.y, grid_vec.x, grid_vec.y, black);
}

template<typename T>
void draw_particle(CImg<unsigned char>& img,
                   const vector<localization::mcl_grid2::particle<T> >& particles,
                   double size, double zoom)
{
    vec2i grid_pose, grid_vec;
    for(size_t i = 0; i < particles.size(); i++) {
        mcl_data.map.get_grid_coordinate(particles[i].pose.x, particles[i].pose.y, grid_pose);
        grid_pose *= zoom;
        grid_pose.y = img.dimy() - grid_pose.y;

        unsigned char color[3] = {0, 0, 0};
        color[0] = (unsigned char)(particles[i].w * 255.0);
        //color[1] = (unsigned char)((1.0 - particles[i].w) * 255.0);


        img.draw_circle(grid_pose.x, grid_pose.y, (size/mcl_data.map.resolution)*zoom, color);

//        vec2d v = vec2d(size*2.0,0.0).rot(particles[i].pose.a) + particles[i].pose.vec();
//        mcl_data.map.get_grid_coordinate(v.x, v.y, grid_vec);
//        grid_vec *= zoom;
//        grid_vec.y = img.dimy() - grid_vec.y;
//        img.draw_arrow(grid_pose.x, grid_pose.y, grid_vec.x, grid_vec.y, black);
    }

}

int main(int argc, char* argv[]) {
    ofstream log;
    log.open("log.dat");
    log_data.open("../test_data/log_1224598716.log", "Odometry", "Laser");
    log_data.read_all();
    lrf_init_fi_table(DEG2RAD(-135), DEG2RAD(0.3515625), fi);
    lrf_init_cos_sin_table(DEG2RAD(-135), DEG2RAD(0.3515625), co, si);
    //threshold and filter
    for(int i = 0; i < log_data.step; i++) {
        lrf_range_median_filter(log_data.lrf[i]);
        lrf_range_threshold(log_data.lrf[i], 400, 0, 4000, 0);
    }


    mcl_cfg.load("../test_data/mcl_grid2d.txt");
    mcl_data.initialize(mcl_cfg);
    mcl_data.map.compute_ray_casting_cache(mcl_cfg.map_angle_res);
    mcl_data.initialize_particle(4, pose2d(2.43, 3.3, 0.0), 0.25, 0.1);


    CImg<unsigned char> map_image = mcl_data.map.get_image();
    CImg<unsigned char> map_image_z =
        (+map_image).resize(map_image.dimx() * ZOOM_FACTOR, map_image.dimy() * ZOOM_FACTOR);
    CImg<unsigned char> map_image_tmp;

    CImgDisplay disp_sensor(400, 400,"Sensor",0);
    CImgDisplay disp_map(map_image.dimx(), map_image.dimy(),"Map",0);
    CImgDisplay disp_map_z(map_image_z.dimx(), map_image_z.dimy(),"Map-Zoom",0);

    map_image.display(disp_map);
    map_image_z.display(disp_map_z);


    while(!disp_map.is_closed  && !disp_map.is_keyESC &&
          !disp_map_z.is_closed  && !disp_map_z.is_keyESC &&
          !disp_sensor.is_closed && !disp_sensor.is_keyESC)
    {
        if(!run) {
            CImgDisplay::wait_all();
        } else {
            disp_map_z.wait(10);
        }

        //copy tmp image
        map_image_tmp = map_image_z;


        if(disp_sensor.mouse_x >= 0 && disp_sensor.mouse_y >= 0) {
            if(disp_sensor.is_keyPADADD) {
                log_step++;
                if(log_step >= log_data.step) {
                    log_step = log_data.step-1;
                }
                disp_sensor.is_keyPADADD = false;
            }

            if(disp_sensor.is_keyPAGEUP) {
                log_step+=10;
                if(log_step >= log_data.step) {
                    log_step = log_data.step-1;
                }
                disp_sensor.is_keyPAGEUP = false;
            }

            if(disp_sensor.is_keyPADSUB) {
                log_step--;
                if(log_step < 0) {
                    log_step = 0;
                }
                disp_sensor.is_keyPADSUB = false;
            }

            if(disp_sensor.is_keyPAGEDOWN) {
                log_step-=10;
                if(log_step < 0) {
                    log_step = 0;
                }
                disp_sensor.is_keyPAGEDOWN = false;
            }
        }//if disp_sensor


        if(disp_map.mouse_x >= 0 && disp_map.mouse_y >= 0) {
            disp_map_mouse.x = disp_map.mouse_x;
            disp_map_mouse.y = disp_map.dimy() - disp_map.mouse_y;
//            PRINTVAR(disp_map_mouse);
            disp_map_mouse_pose = (disp_map_mouse - mcl_data.map.center) * mcl_data.map.resolution;
//            PRINTVAR(disp_map_mouse_pose);
            disp_map.set_title("Map: %6.3f %6.3f", disp_map_mouse_pose.x, disp_map_mouse_pose.y);
        }//if disp_map

        if(disp_map_z.mouse_x >= 0 && disp_map_z.mouse_y >= 0) {
            disp_map_z_mouse.x = disp_map_z.mouse_x;
            disp_map_z_mouse.y = disp_map_z.dimy() - disp_map_z.mouse_y;
//            PRINTVAR(disp_map_z_mouse);
            disp_map_z_mouse_pose = ((disp_map_z_mouse * (1.0/ZOOM_FACTOR)) - mcl_data.map.center) * mcl_data.map.resolution;
//            PRINTVAR(disp_map_z_mouse_pose);
            disp_map_z.set_title("Map-Zoom: %6.3f %6.3f %d %d",
                                  disp_map_z_mouse_pose.x, disp_map_z_mouse_pose.y,
                                  disp_map_z_mouse.x, disp_map_z_mouse.y);


            if(disp_map_z.is_keyR) {
                run = true;
                disp_map_z.is_keyR = false;
            }

            if(disp_map_z.is_keyS) {
                single_step = true;
                run = false;
                disp_map_z.is_keyS = false;
            }

            if(disp_map_z.is_keySPACE) {
                run = false;
                disp_map_z.is_keySPACE = false;
            }

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
                    if(set_pose_state == SET_POSE_STATE_NONE) {
                        //set position
                        set_pose_pose.x = disp_map_z_mouse_pose.x;
                        set_pose_pose.y = disp_map_z_mouse_pose.y;
                        set_pose_state = SET_POSE_STATE_DRAG;
                        cout << "Set position at: " << set_pose_pose << "\n";
                        draw_robot(map_image_tmp, set_pose_pose, 0.25, ZOOM_FACTOR);
                        update_map_z_display = true;
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

                        double q = 1.0;
                        double q1 = 1.0;
                        double p;
                        double p1;
                        if(mcl_data.map.ray_casting_cache[grid.x][grid.y].size() != 0) {
                            for(int i = 0; i < mcl_data.map.angle_step; i++) {
                                r = mcl_data.map.ray_casting_cache[grid.x][grid.y][i];
                                if(r > mcl_cfg.z_max_range) continue;

                                //compute pdf
                                p = math_model::beam_range_finder_measurement(r,
                                                                              r,
                                                                              mcl_cfg.z_max_range,
                                                                              mcl_cfg.z_hit_var,
                                                                              mcl_cfg.z_short_rate,
                                                                              mcl_cfg.z_weight);
                                p1 = math_model::beam_range_finder_measurement(r*0.7,
                                                                               r,
                                                                               mcl_cfg.z_max_range,
                                                                               mcl_cfg.z_hit_var,
                                                                               mcl_cfg.z_short_rate,
                                                                               mcl_cfg.z_weight);
                                q *= p;
                                q1 *= p1;


                                r /= mcl_data.map.resolution;
                                rad = mcl_data.map.angle_res * i;
                                img_x = r * cos(rad) * ZOOM_FACTOR;
                                img_y = r * sin(rad) * ZOOM_FACTOR;


                                map_image_tmp.draw_line(disp_map_z.mouse_x,
                                                        disp_map_z.mouse_y,
                                                        disp_map_z_mouse.x + img_x,
                                                        map_image_z.dimy() - (disp_map_z_mouse.y + img_y), red);
                            }
                            map_image_tmp.draw_circle(disp_map_z.mouse_x, disp_map_z.mouse_y, (int)(mcl_cfg.z_max_range/mcl_cfg.map_res)*ZOOM_FACTOR, green, 1.0, 5);
                            map_image_tmp.draw_circle(disp_map_z.mouse_x, disp_map_z.mouse_y, (int)(0.25/mcl_cfg.map_res)*ZOOM_FACTOR, green, 1.0, 5);
                            update_map_z_display = true;
                            PRINTVAR(q);
                            PRINTVAR(q1);
                        }
                    }
                    break;
                case 4: //mid btn
                    {
                        int img_x, img_y;
                        double r;
                        double r_grid;
                        double rad;
                        vec2i grid;
                        int angle_idx = 0;
                        if(!mcl_data.map.get_grid_coordinate(disp_map_z_mouse_pose.x, disp_map_z_mouse_pose.y, grid))
                            break;
                        if(mcl_data.map.ray_casting_cache[grid.x][grid.y].size() != 0) {
                            measurement_z.clear();
                            lrf_scan_point_from_scan_range(log_data.lrf[log_step], co, si, lrf_pts, 0.001);
                            PRINTVAR(lrf_pts.size());
                            for(size_t i = 0; i < lrf_pts.size(); i+=10) {
                                if(lrf_pts[i].size() > 0.1) {
                                    measurement_z.push_back(lrf_pts[i]);
                                }
                            }
//                            PRINTVAR(measurement_z.size());
//                            PRINTVAR(log_data.odo[log_step]);

                            double p = 1.0;
                            double zp = 1.0;
                            for(size_t i = 0; i < measurement_z.size(); i++) {
                                r = measurement_z[i].size() / mcl_data.map.resolution;
                                rad = measurement_z[i].theta();
                                angle_idx = (int)(rad/mcl_data.map.angle_res);
                                if(angle_idx < 0) angle_idx += mcl_data.map.angle_step;

                                img_x = r * cos(rad) * ZOOM_FACTOR;
                                img_y = r * sin(rad) * ZOOM_FACTOR;
                                map_image_tmp.draw_line(disp_map_z.mouse_x,
                                                        disp_map_z.mouse_y,
                                                        disp_map_z_mouse.x + img_x,
                                                        map_image_z.dimy() - (disp_map_z_mouse.y + img_y), red);

                                r_grid = mcl_data.map.ray_casting_cache[grid.x][grid.y][angle_idx] / mcl_data.map.resolution;
                                img_x = r_grid * cos(angle_idx * mcl_data.map.angle_res) * ZOOM_FACTOR;
                                img_y = r_grid * sin(angle_idx * mcl_data.map.angle_res) * ZOOM_FACTOR;

//                                measurement_z[i] += vec2d(0.22, 0.0);
//                                r = measurement_z[i].size() / mcl_data.map.resolution;
//                                rad = measurement_z[i].theta();
//                                img_x = r * cos(rad) * ZOOM_FACTOR;
//                                img_y = r * sin(rad) * ZOOM_FACTOR;
                                map_image_tmp.draw_line(disp_map_z.mouse_x,
                                                        disp_map_z.mouse_y,
                                                        disp_map_z_mouse.x + img_x,
                                                        map_image_z.dimy() - (disp_map_z_mouse.y + img_y), green);


                                zp = math_model::beam_range_finder_measurement(measurement_z[i].size(),
                                                                               mcl_data.map.ray_casting_cache[grid.x][grid.y][angle_idx],
                                                                               mcl_cfg.z_max_range,
                                                                               mcl_cfg.z_hit_var,
                                                                               mcl_cfg.z_short_rate,
                                                                               mcl_cfg.z_weight);
                                p *= zp;
                            }

                            PRINTVAR(p);




                            update_map_z_display = true;
                        }


                    }
                    break;
                case 3: //left-right btn
                    if(set_pose_state == SET_POSE_STATE_DRAG) {
                        vec2d v = disp_map_z_mouse_pose - set_pose_pose.vec();
                        set_pose_pose.a = v.theta();
                        set_pose_state = SET_POSE_STATE_DRAG;
                        cout << "With angle : " << set_pose_pose << "\n";
                        draw_robot(map_image_tmp, set_pose_pose, 0.25, ZOOM_FACTOR);
                        update_map_z_display = true;
                    }
                    break;
                case 5: //left-mid btn
                case 6: //mid-right btn
                case 7: //all btn
                default:
                    break;
            }//switch
        }//if disp_map_z


        if(run || single_step) {
            if(single_step) {
                single_step = false;
            }

            cout << "====== Step: " << log_step << "======\n";


            //get sensor data
            cout << "====== get sensor data ======\n";
            measurement_z.clear();
            lrf_scan_point_from_scan_range(log_data.lrf[log_step], co, si, lrf_pts, 0.001);
            for(size_t i = 0; i < lrf_pts.size(); i+=20) {
                if(lrf_pts[i].size() > 0.1) {
                    //measurement_z.push_back(lrf_pts[i] + vec2d(0.22, 0.0));
                    measurement_z.push_back(lrf_pts[i]);
                }
            }
            PRINTVAR(measurement_z.size());
            PRINTVAR(log_data.odo[log_step]);


            //update
            cout << "====== update ======\n";
            localization::mcl_grid2::update_with_odomety(mcl_cfg,
                                                         mcl_data,
                                                         measurement_z,
                                                         log_data.odo[log_step]);

//            PRINTVEC(mcl_data.p);

            //find best particle
          double max_w = -1e100;
          int max_idx = 0;
          for(size_t i = 0; i < mcl_data.p.size(); i++) {
              if(mcl_data.p[i].w >= max_w) {
                  max_w = mcl_data.p[i].w;
                  max_idx = i;
              }
          }

          log << log_data.odo[log_step] << " " << mcl_data.p[max_idx].pose << "\n";



            //resample
            cout << "====== resample ======\n";
            if(mcl_data.stratified_resample((int)(0.1 *  mcl_cfg.n_particles))) {
                cout << "do resample\n";
            } else {
                cout << "not resample\n";
            }

            //draw particle
             draw_particle(map_image_tmp, mcl_data.p, 0.03, ZOOM_FACTOR);
             update_map_z_display = true;

             PRINTVAR(mcl_data.p[max_idx]);
             draw_robot(map_image_tmp, mcl_data.p[max_idx].pose, 0.25, ZOOM_FACTOR);



            //draw particle
         //   draw_particle(map_image_tmp, mcl_data.p, 0.05, ZOOM_FACTOR);
           // update_map_z_display = true;
            //draw lrf
            int img_x, img_y;
            double r;
            double rad;
            vec2i grid;
            if(mcl_data.map.get_grid_coordinate(mcl_data.p[max_idx].pose.x,
                                                mcl_data.p[max_idx].pose.y, grid))
            {

                grid *= ZOOM_FACTOR;
                for(size_t i = 0; i < measurement_z.size(); i++) {
                    r = measurement_z[i].size() / mcl_data.map.resolution;
                    rad = norm_a_rad(measurement_z[i].theta() + mcl_data.p[max_idx].pose.a);
                    img_x = r * cos(rad) * ZOOM_FACTOR;
                    img_y = r * sin(rad) * ZOOM_FACTOR;
                    map_image_tmp.draw_line(grid.x,
                                            map_image_z.dimy() - grid.y,
                                            grid.x + img_x,
                                            map_image_z.dimy() - (grid.y + img_y), green);

                }
            }






            cout << "====== end ======\n";




            log_step++;
            if(log_step >= log_data.step) {
                log_step = log_data.step-1;
                std::cout << "Finish log file" << endl;
                run = !run;
                continue;
            }
        }


        //update sensor display
        if(log_step != last_log_step) {
            CImg<unsigned char> img(400, 400, 1, 3, 0);
            lrf_scan_point_from_scan_range(log_data.lrf[last_log_step], co, si, lrf_pts, 0.001);
            lrf_draw_scan_point_to_cimg(lrf_pts, img, red, true, 40);
            lrf_draw_scan_point_to_cimg(measurement_z, img, green, false, 40);
            disp_sensor.set_title("Odo: %6.3f %6.3f %6.3f",
                    log_data.odo[log_step].x,
                    log_data.odo[log_step].y,
                    log_data.odo[log_step].a);
            img.display(disp_sensor);
            last_log_step = log_step;
        }

        //update map display
        if(update_map_z_display) {
            map_image_tmp.display(disp_map_z);
            update_map_z_display = false;
        }


    }
    log.close();

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
