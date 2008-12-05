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

const unsigned char
    red[] = { 255,0,0 }, green[] = { 0,255,0 }, blue[] = { 0,0,255 },
    red_drak[] = { 127,0,0 }, green_drak[] = { 0,127,0 }, blue_drak[] = { 0,0,127 },
    cyan[] = {0, 255, 255}, yellow[] = {255, 255, 0},
    cyan_drak[] = {0, 127, 127}, yellow_dark[] = {127, 127, 0},
    black[] = { 0,0,0 },
    gray[] = { 127, 127, 127 },
    white[] = { 255,255,255 };

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

            /*
            vec2i hit;
            int result;
            map_image_tmp = map_image;
            static const int step = 180;
            for(int i = 0; i < step; i++) {
                result = map.get_ray_casting_hit_point(map_mouse_x, map_mouse_y, i * (2.0*M_PI/step), hit);
                PRINTVAR(result);
                if(result == 1) {
                    PRINTVAR(hit);
                    map_image_tmp.draw_line(display_map.mouse_x, display_map.mouse_y, hit.x, map_image.dimy() - hit.y, red);
                }
            }
            map_image_tmp.display(display_map);
            */
        }
    }
    return 0;
}

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
