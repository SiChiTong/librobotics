/*
 * lb_mcl.h
 *
 *  Created on: Jan 29, 2009
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

#ifndef LB_MCL2_H_
#define LB_MCL2_H_

#include "lb_common.h"
#include "lb_exception.h"
#include "lb_data_type.h"
#include "lb_map2_grid.h"

namespace librobotics {


struct lb_mcl2_particle {
    pose2f p;                //!< robot position
    LB_FLOAT w;                 //!< weight
    lb_mcl2_particle() : w(0) { }

    ///Support for output stream operator
    friend std::ostream& operator << (std::ostream& os, const lb_mcl2_particle& p) {
        return os << p.p << " " << p.w;
    }

    ///Support for input stream operator
    friend std::istream& operator >> (std::istream& is, lb_mcl2_particle& p) {
        is >> p.p >> p.w;
        return is;
    }
};


/**
 * Configuration for MCL on grid2 map
 */
struct lb_mcl_grid2_configuration {
    std::string map_config_file;         //!< map configuration file
    std::string map_image_file;          //!< map image files
    LB_FLOAT map_angle_res;     //!< pre-compute ray casting angle resolution

    int n_particles;            //!< number of particles
    LB_FLOAT min_particels;     //!< in percentage of n_particles \f$(0.0, 1.0)\f$
    LB_FLOAT a_slow, a_fast;    //!< decay rate for augmented MCL
    LB_FLOAT v_factor;
    LB_FLOAT motion_var[6];     //!< \f$(\sigma_0...\sigma_3)\f$ in odometry mode \n \f$(\sigma_0...\sigma_5)\f$ in velocity mode
    LB_FLOAT map_var;           //!< compute directly from map resolution
    LB_FLOAT z_max_range;       //!< max measurement range
    LB_FLOAT z_hit_var;         //!< measurement hit target variance (normal distribution)
    LB_FLOAT z_short_rate;      //!< measurement too short rate (exponential distribution)
    LB_FLOAT z_weight[4];       //!< normalized weight for all possible measurement outcome


    /**
     * Simple load a configuration from text file
     * @param filename
     */
    void load(const std::string& filename) {
        try {
            ConfigFile file(filename);
            LB_PRINT_VAL("============== MCL Configuration ==============");
            LOAD_N_SHOW_CFG(map_config_file, std::string);
            LOAD_N_SHOW_CFG(map_image_file, std::string);
            LOAD_N_SHOW_CFG(map_angle_res, LB_FLOAT);

            LOAD_N_SHOW_CFG(n_particles, int);
            LOAD_N_SHOW_CFG(min_particels, LB_FLOAT);
            LOAD_N_SHOW_CFG(a_slow, LB_FLOAT);
            LOAD_N_SHOW_CFG(a_fast, LB_FLOAT);
            LOAD_N_SHOW_CFG(v_factor, LB_FLOAT);
            LOAD_N_SHOW_CFG(motion_var[0], LB_FLOAT);
            LOAD_N_SHOW_CFG(motion_var[1], LB_FLOAT);
            LOAD_N_SHOW_CFG(motion_var[2], LB_FLOAT);
            LOAD_N_SHOW_CFG(motion_var[3], LB_FLOAT);
            LOAD_N_SHOW_CFG(motion_var[4], LB_FLOAT);
            LOAD_N_SHOW_CFG(motion_var[5], LB_FLOAT);

            LOAD_N_SHOW_CFG(map_var, LB_FLOAT);
            LOAD_N_SHOW_CFG(z_max_range, LB_FLOAT);
            LOAD_N_SHOW_CFG(z_hit_var, LB_FLOAT);
            LOAD_N_SHOW_CFG(z_short_rate, LB_FLOAT);
            LOAD_N_SHOW_CFG(z_weight[0], LB_FLOAT);
            LOAD_N_SHOW_CFG(z_weight[1], LB_FLOAT);
            LOAD_N_SHOW_CFG(z_weight[2], LB_FLOAT);
            LOAD_N_SHOW_CFG(z_weight[3], LB_FLOAT);
            LB_PRINT_VAL("===============================================");
        } catch (std::string& e) {
            LB_PRINT_STREAM << e;
        }
    }
};


/**
 * Data for MCL on grid2 map
 */
struct lb_mcl_grid2_data {
    std::vector<lb_mcl2_particle> p;         //!< current particle set
    std::vector<lb_mcl2_particle> p_tmp;     //!< temporary particle set
    lb_grid2_data map;                       //!< gird map
    pose2f last_odo_pose;                    //!< last odometry position

    void initialize(const lb_mcl_grid2_configuration& cfg) {
        p.resize(cfg.n_particles);
        p_tmp.resize(cfg.n_particles);
        map.load_config(cfg.map_config_file);
        map.load_map_image(cfg.map_image_file);

        //compute ray_cast cache
        map.compute_ray_casting_cache(cfg.map_angle_res);

    }
};


inline int lb_mcl_grid2_update_with_odomety(const lb_mcl_grid2_configuration& cfg,
                                            const std::vector<vec2f>& z,
                                            const pose2f& odo_pose,
                                            lb_mcl_grid2_data& data)
{
    for(int n = 0; n < cfg.n_particles; n++) {
        //predict position
        data.p_tmp[n].p =
            lb_odometry_motion_model_sample(odo_pose,
                                            data.last_odo_pose,
                                            data.p[n].p,
                                            cfg.motion_var);
        //check measurement
//        vec2i grid_coor;
//        LB_FLOAT sense_angle = 0;
//        int sense_idx = 0;
//        LB_FLOAT zp;
//        if(data.map.get_grid_coordinate(data.p_tmp[n].p.x, data.p_tmp[n].p.y, grid_coor)) {
//            if(data.map.ray_casting_cache[grid_coor.x][grid_coor.y].size() != 0) {
//                data.p_tmp[n].w = 1.0;
//                for(size_t i = 0; i < z.size(); i++) {
//                //find nearest measurement in pre-computed ray casting
//
//                    //compute sense angle (convert from local coordinate to global coordinate)
//                    sense_angle = lb_normalize_angle(z[i].theta() + data.p_tmp[n].p.a);
//
//                    //get index
//                    sense_idx = (int)(sense_angle/data.map.angle_res);
//                    if(sense_idx < 0) sense_idx += data.map.angle_step;
//
//                    //compute PDF (can speed up by lookup table)
//                    zp = lb_beam_range_finder_measurement_model(z[i].size(),
//                                                                data.map.ray_casting_cache[grid_coor.x][grid_coor.y][sense_idx],
//                                                                cfg.z_max_range,
//                                                                cfg.z_hit_var,
//                                                                cfg.z_short_rate,
//                                                                cfg.z_weight);
//                    data.p_tmp[n].w *= zp;
//                }
//            } else {
//                data.p_tmp[n].w = 0;
//            }
//
//        } else {
//            data.p_tmp[n].w = 0;
//        }
        data.p[n].p = data.p_tmp[n].p;
        data.p[n].w = data.p_tmp[n].w;
    }
    data.last_odo_pose = odo_pose;
    return 0;
}




}


#endif /* LB_MCL2_H_ */
