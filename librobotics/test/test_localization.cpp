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
    map.load_image("../test_data/map_400_200_2.png",
                   pose2d(),
                   vec2d(20.0, 10.0),
                   1.0/10);
    map.print_info();

    CImgDisplay display_main(500,500,"Main Display",0);
    CImgDisplay display_map(100,100,"Map",0);

    CImg<unsigned char> map_image = map.get_image();

    display_map.resize(map_image);
    map_image.display(display_map);
    while(!display_main.is_closed && !display_main.is_keyESC)
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
