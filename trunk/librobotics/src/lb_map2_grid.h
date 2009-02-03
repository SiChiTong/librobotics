/*
 * lb_map2_grid.h
 *
 *  Created on: Jan 15, 2009
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

#ifndef LB_MAP2_GRID_H_
#define LB_MAP2_GRID_H_

#include "lb_common.h"
#include "lb_exception.h"
#include "lb_data_type.h"
#include "lb_statistic_function.h"

namespace librobotics {

    /**
     * Data structure for 2D grid map.
     */
    struct lb_grid2_data {
        vec2i       size;           //!< Size of the map
        vec2i       center;         //!< Center of the map
        pose2f      offset;         //!< Map offset in real world unit (m, mm, cm...)
        LB_FLOAT    resolution;     //!< Map resolution real world unit/map size unit
        LB_FLOAT    angle_res;
        int         angle_step;
        std::vector<std::vector<LB_FLOAT> > mapprob;           //!< Value of each grid cell
        std::vector<std::vector<LB_FLOAT> > dyn_mapprob;       //!< dynamic value of each grid cell

        std::vector<std::vector<int> > gradient_map;
        std::vector<std::vector<int> > gradient_intr;

        std::vector<std::vector<std::vector<LB_FLOAT> > > ray_casting_cache;

        void show_information() {
            LB_PRINT_VAR(size);
            LB_PRINT_VAR(center);
            LB_PRINT_VAR(offset);
        }


        /**
         * Check grid position with map size.
         * @param x grid coordinate
         * @param y grid coordinate
         * @return true if (x,y) is inside the map
         */
        inline bool is_inside(int x, int y) const {
            return (x >= 0) && (x < size.x) && (y >= 0) && (y < size.y);
        }

        /**
         * Get real world unit position of given grid coordinate.
         * @param x grid coordinate
         * @param y grid coordinate
         * @param pts
         * @return true if (x,y) is inside the map
         */
        inline bool get_grid_position(int x, int y, vec2f& pts) const {
            if(is_inside(x, y)) {
                pts.x = ((x - center.x) * resolution) + offset.x;
                pts.y = ((y - center.y) * resolution) + offset.y;
                return true;
            } else {
                return false;
            }
        }

        /**
         * Get grid coordinate from real world unit position.
         * @param x position
         * @param y position
         * @param v result in grid coordinate
         * @return true if (x,y) is inside the map
         */
        inline bool get_grid_coordinate(LB_FLOAT x, LB_FLOAT y, vec2i& v) const {
            v.x = center.x + (int)LB_ROUND((x-offset.x)/resolution);
            v.y = center.y + (int)LB_ROUND((y-offset.y)/resolution);
            if(is_inside(v.x, v.y))
                return true;
            else {
                return false;
            }
        }

        inline bool get_random_pts(vec2f& pts, LB_FLOAT max_mapprob = 0.0, int retry = 100) const {
            int x, y;
            bool pass = false;
            do {
                x = (int)(lb_rand() * size.x);
                y = (int)(lb_rand() * size.y);
                pass = get_grid_position(x, y, pts);
                if(pass) {
                    if(mapprob[x][y] > max_mapprob) {
                        pass = false;
                    }
                }
            } while(!pass && (retry-- > 0));
            return pass;
        }

        /**
         * Compute hit point on grid coordinate from the (x,y) grid position from given direction.
         * Adapt from http://student.kuleuven.be/~m0216922/CG/raycasting.html
         * @param x grid position
         * @param y grid position
         * @param dir ray casting direction
         * @param hit_grid result of the function
         * @return -1 if (x,y) not in the map \n
         *          0 if (x,y) inside occupied grid\n
         *          1 if hit \n
         *          2 if not hit
         */
        inline int get_ray_casting_hit_point(int x, int y, LB_FLOAT dir, vec2i& hit_grid) {
            //outside the map
            if(!is_inside(x, y))
                return -1;

            //hit itself
            if(mapprob[x][y] != 0) {
                hit_grid = vec2i(x,y);
                return 0;
            }

            int mapX = x;
            int mapY = y;
            LB_FLOAT posX = mapX;
            LB_FLOAT posY = mapY;
            LB_FLOAT rayPosX = posX;
            LB_FLOAT rayPosY = posY;
            LB_FLOAT rayDirX = cos(dir);
            LB_FLOAT rayDirY = sin(dir);
            LB_FLOAT deltaDistX = sqrt(1 + LB_SQR(rayDirY) / LB_SQR(rayDirX));
            LB_FLOAT deltaDistY = sqrt(1 + LB_SQR(rayDirX) / LB_SQR(rayDirY));

            //length of ray from current position to next x or y-side
            LB_FLOAT sideDistX;
            LB_FLOAT sideDistY;

            //what direction to step in x or y-direction (either +1 or -1)
            int stepX;
            int stepY;

            int hit = 0; //was there a wall hit?
            int side; //was a NS or a EW wall hit?

            //calculate step and initial sideDist
            if (rayDirX < 0) {
                stepX = -1;
                sideDistX = (rayPosX - mapX) * deltaDistX;
            } else {
                stepX = 1;
                sideDistX = (mapX + 1.0 - rayPosX) * deltaDistX;
            }
            if (rayDirY < 0) {
                stepY = -1;
                sideDistY = (rayPosY - mapY) * deltaDistY;
            } else {
                stepY = 1;
                sideDistY = (mapY + 1.0 - rayPosY) * deltaDistY;
            }
            //perform DDA
            while (hit == 0) {
                //jump to next map square, OR in x-direction, OR in y-direction
                if (sideDistX < sideDistY) {
                    sideDistX += deltaDistX;
                    mapX += stepX;
                    side = 0;
                } else {
                    sideDistY += deltaDistY;
                    mapY += stepY;
                    side = 1;
                }

                //Check if ray is out side map boundary
                if(!is_inside(mapX, mapY)) return 2; //dose not hit any cell

                //Check if ray has hit
                if (mapprob[mapX][mapY]> 0) hit = 1;
            }

            //hit
            hit_grid.x = mapX;
            hit_grid.y = mapY;
            return 1;
        }

        /**
         * Pre-compute ray casting result of all unoccupied gird.
         * @param angle_res ray casting angle resolution in radian
         */
        inline void compute_ray_casting_cache(LB_FLOAT _angle_res, LB_FLOAT threshold = 0) {
            if(_angle_res <= 0) {
                throw LibRoboticsRuntimeException("angle resolution must > 0");
            }
            angle_res = _angle_res;
            angle_step = (int)((2*M_PI) / angle_res + 1);
            int result;
            vec2i hit;
            int cnt = 0;

            LB_PRINT_STREAM << "Start ray casting compute...";
            ray_casting_cache.resize(size.x);
            for(int x = 0; x < size.x; x++) {
                ray_casting_cache[x].resize(size.y);
                if((x % 10) == 0) {
                    LB_PRINT_STREAM << ".";
                }
                for(int y = 0; y < size.y; y++) {
                    if(mapprob[x][y] > threshold) {
                        //occupied grid
                        continue;
                    }
                    ray_casting_cache[x][y].resize(angle_step);
                    for(int i = 0; i < angle_step; i++) {
                        result = get_ray_casting_hit_point(x, y, i * angle_res, hit);
                        if(result == 1) {
                            ray_casting_cache[x][y][i] = LB_SIZE((LB_FLOAT)(x-hit.x), (LB_FLOAT)(y-hit.y)) * resolution;
                        } else {
                            ray_casting_cache[x][y][i] = -1;    //no measurement on that direction
                        }
                        cnt++;
                    }
                }
            }
            LB_PRINT_STREAM << "done! with " << cnt << " ray casting operations\n";
        }

        inline bool get_gradient_path(const vec2f& start,
                                     const vec2f& goal,
                                     std::vector<vec2i>& path,
                                     LB_FLOAT clearance)
        {
            vec2i grid_start;
            vec2i grid_goal;

            //check start point
            if(get_grid_coordinate(start.x, start.y, grid_start)) {
                if(dyn_mapprob[grid_start.x][grid_start.y] > 0.0) {
                    std::cerr << "start position is inside the obstacle\n";
                    return false;
                }
            } else {
                LB_PRINT_STREAM << "start position is outside the map\n";
                return false;
            }

            //check goal point
            if(get_grid_coordinate(goal.x, goal.y, grid_goal)) {
                if(dyn_mapprob[grid_goal.x][grid_goal.y] > 0) {
                    LB_PRINT_STREAM << "goal position is inside the obstacle\n";
                    return false;
                }
            } else {
                LB_PRINT_STREAM << "goal position is outside the map\n";
                return false;
            }

            LB_PRINT_STREAM << "Move from: " << start << " (" << grid_start
                            <<  ") to: " << goal << "  (" << grid_goal << ")\n";

#warning "not implement"

            return true;
        }


        inline void load_config(const std::string& filename) {
            std::ifstream file;
            file.open(filename.c_str());
            if(!file.is_open())
                throw LibRoboticsIOException("Cannot open %s for reading", filename.c_str());
            file >> size;
            file >> center;
            file >> offset;
            file >> resolution;
            if(resolution <= 0) {
                warn("resolution must > 0 -> automatic set to 0.1");
                resolution = 0.1;
            }

            LB_RESIZE_2D_VEC(mapprob, size.x, size.y);
            LB_RESIZE_2D_VEC(gradient_map, size.x, size.y);
            LB_RESIZE_2D_VEC(gradient_intr, size.x, size.y);
            file.close();
        }

        inline void save_config(const std::string& filename) {
            std::ofstream file;
            file.open(filename.c_str());
            if(!file.is_open())
                throw LibRoboticsIOException("Cannot open %s for writing", filename.c_str());

            file << size << "\n";
            file << center << "\n";
            file << offset << "\n";
            file << resolution << "\n";

            file.close();
        }

        inline void load_map_txt(const std::string& filename) {
            std::ifstream file;
            file.open(filename.c_str());
            if(!file.is_open())
                throw LibRoboticsIOException("Cannot open %s for reading", filename.c_str());

            for(int i = 0; i < size.x && !file.eof(); i++) {
               for(int j = 0; j < size.y && !file.eof(); j++) {
                   file >> mapprob[i][j];
               }
            }
            file.close();
        }

        inline void save_map_txt(const std::string& filename) {
            std::ofstream file;
            file.open(filename.c_str());
            if(!file.is_open())
                throw LibRoboticsIOException("Cannot open %s for writing", filename.c_str());

            for(int i = 0; i < size.x; i++) {
                for(int j = 0; j < size.y; j++) {
                    file << mapprob[i][j] << " ";
                }
                file << "\n";
            }
            file << "\n";
            file.close();
        }

        /**
         * Load map data from text file.
         * Example of map with 5x5 size (do not put \\\\ comment inside the file) \n
         *      5 5             \\\\size \n
         *      0.0 0.0 0.0     \\\\offset \n
         *      3 3             \\\\center \n
         *      1.0             \\\\resolution \n
         *      0.1 0.2 0.3     \\\\grid value of (x, 0) \n
         *      0.4 0.5 0.6     \\\\grid value of (x, 1) \n
         *      0.7 0.8 0.8     \\\\grid value of (x, 2) \n
         * @param filename of the map data
         */
        inline void load_txt(const std::string& filename) {
            std::ifstream file;
            file.open(filename.c_str());
            if(!file.is_open())
                throw LibRoboticsIOException("Cannot open %s for reading", filename.c_str());

            file >> size;
            file >> center;
            file >> offset;
            file >> resolution;
            if(resolution <= 0) {
                warn("resolution must > 0 -> automatic set to 0.1");
                resolution = 0.1;
            }

            LB_RESIZE_2D_VEC(mapprob, size.x, size.y);
            LB_RESIZE_2D_VEC(gradient_map, size.x, size.y);
            LB_RESIZE_2D_VEC(gradient_intr, size.x, size.y);

            for(int i = 0; i < size.x && !file.eof(); i++) {
                for(int j = 0; j < size.y && !file.eof(); j++) {
                    file >> mapprob[i][j];
                }
            }

            //copy map
            dyn_mapprob = mapprob;
        }

        /**
         * Save map data to text file.
         * @param filename of the output map
         */
        inline void save_txt(const std::string& filename) {
            std::ofstream file;
            file.open(filename.c_str());
            if(!file.is_open())
                throw LibRoboticsIOException("Cannot open %s for writing", filename.c_str());

            file << size << "\n";
            file << center << "\n";
            file << offset << "\n";
            file << resolution << "\n";
            for(int i = 0; i < size.x; i++) {
                for(int j = 0; j < size.y; j++) {
                    file << mapprob[i][j] << " ";
                }
                file << "\n";
            }
            file << "\n";
            file.close();
        }



#if (librobotics_use_cimg == 1)
        /**
         * Load map data from image file. Image data should save in 8 bit color depth format.
         * This function will use only first channel as map data.
         * Map data will read directly for each pixel position to grid position.
         * @param filename of the map image
         * @param _offset map offset in real world unit (m, mm, cm...)
         * @param _center map center in real world unit (m, mm, cm...)
         * @param _resolution map resolution in real world unit (m, mm, cm...)
         */
        inline void load_map_image(const std::string& filename) {
            using namespace cimg_library;
            cimg8u img;

            try {
                img.load(filename.c_str());
            } catch (CImgException& e) {
                throw LibRoboticsIOException(e.message);
            }


            for(int i = 0; i < size.x && i < img.dimx(); i++) {
                for(int j = 0; j < size.y && j < img.dimy(); j++) {
                    mapprob[i][j] = (255 - img(i, j, 0)) / 255.0;
                }
            }

            //copy map
            dyn_mapprob = mapprob;
        }


        /**
         * Get image of the map
         * @param flip_x true to flip result image along X-axis
         * @param flip_y true to flip result image along Y-axis
         * @return image in CImg<unsigned char> format.
         */
        inline cimg8u get_image(bool flip_x = false,
                                bool flip_y = true,
                                bool invert = true)
        {
            using namespace cimg_library;
            cimg8u img(size.x, size.y, 1, 3, 0);
            unsigned char v = 0;
            int x, y;
            for(int i = 0; i < size.x; i++) {
                for(int j = 0; j < size.y; j++) {
                    v = (unsigned char)(mapprob[i][j] * 255);
                    x = i;
                    y = j;

                    if(invert) v = 255 - v;
                    if(flip_x) x = (size.x - 1) - x;
                    if(flip_y) y = (size.y - 1) - y;

                    img(x, y, 0) = v;
                    img(x, y, 1) = v;
                    img(x, y, 2) = v;
                }
            }
            return img;
        }
#endif //(librobotics_use_cimg == 1)
    };
}


#endif /* LB_MAP2_GRID_H_ */
