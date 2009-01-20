/*
 * lb_simple_gui.h
 *
 *  Created on: Jan 16, 2009
 *      Author: mahisorn
 *
 *  Copyright (c) <2009> <Mahisorn Wongphati>
 *  Permission is hereby granted, free of charge, to any person
 *  obtaining a copy of this software and associated documentation
 *  files (the "Software"), to deal in the Software without
 *  restriction, including without limitation the rights to use,
 *  copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the
 *  Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be
 *  included in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *  OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef LB_SIMPLE_GUI_H_
#define LB_SIMPLE_GUI_H_

#include "lb_common.h"
#include "lb_data_type.h"



namespace librobotics {



#if (librobotics_use_cimg == 1)
struct simple_cimg_gui {
    simple_cimg_gui() :
        disp(200, 200, "Simple GUI", 0),
        flip_x(false), flip_y(true),
        wheel(0), diff_wheel(0), last_wheel(0), wheel_zoom(1),
        button(0), last_button(0)
    { }

    simple_cimg_gui(int w, int h,
                    const std::string& title,
                    bool flip_mouse_x = false,
                    bool flip_mouse_y = true)
        :  disp(w, h, title.c_str(), 0),
           flip_x(false), flip_y(true),
           wheel(0), diff_wheel(0), last_wheel(0),
           button(0), last_button(0)
    { }

    bool is_closed() { return disp.is_closed; }

    void process_event( ) {
        end_process();
        if(disp.mouse_x > 0 && disp.mouse_y > 0) {
            mouse.x = disp.mouse_x;
            if(flip_x) mouse.x = disp.dimx() - mouse.x;
            mouse.y = disp.mouse_y;
            if(flip_y) mouse.y = disp.dimy() - mouse.y;

            diff_mouse = mouse - last_mouse;

            wheel = disp.wheel;
            diff_wheel = wheel - last_wheel;
            wheel_zoom -= diff_wheel;
            if(wheel_zoom < 1) wheel_zoom = 1;

            button = disp.button;
            LB_PRINT_VAR(button);
            switch(button) {
                case 0: //release all btn

                break;
                case 1: //left btn

                break;
                case 2: //right btn

                break;
                case 3: //left-right btn

                break;
                case 4: //mid btn
                {
                    wheel_zoom = 1;
                }
                break;
                case 5: //left-mid btn

                break;
                case 6: //mid-right btn

                break;
                case 7: //all btn

                break;


            }

//            LB_PRINT_VAR(mouse);
//            LB_PRINT_VAR(diff_mouse);
//            LB_PRINT_VAR(diff_wheel);
//            LB_PRINT_VAR(button);
//            LB_PRINT_VAR(last_button);
            LB_PRINT_VAR(wheel_zoom);
        }
    }

    void end_process( ) {
        last_mouse = mouse;
        last_wheel = wheel;
        last_button = button;
    }

    vec2f get_zoom_pose(LB_FLOAT scale = 1.0) {
        LB_FLOAT factor = wheel_zoom * scale;
        return vec2f(mouse.x * factor, mouse.y * factor);
    }


    cimg_library::CImgDisplay disp;
    bool flip_x, flip_y;
    vec2i mouse;
    vec2i diff_mouse;
    vec2i last_mouse;

    int wheel;
    int diff_wheel;
    int last_wheel;
    int wheel_zoom;

    unsigned int button;
    unsigned int last_button;

};

#endif


}




#endif /* LB_SIMPLE_GUI_H_ */
