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
    std::string mapfile;    //!< map file name
    pose2f map_offset;    //!< map offset
    vec2f map_center;     //!< map center
    LB_FLOAT map_res;              //!< map resolution
    LB_FLOAT map_angle_res;        //!< pre-compute ray casting angle resolution

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
//        std::ifstream file;
//        file.open(filename.c_str());
//        if(!file.is_open()) {
//            throw LibRoboticsIOException("Cannot load file %s in %s", filename.c_str(), __FUNCTION__);
//        }
//
//        std::string tmp;    //dummy data for load_cfg_from_text_file macro
//        std::cout << "======= mcl_grid2::configuration =======\n";
//        load_cfg_from_text_file(mapfile, file);
//        load_cfg_from_text_file(map_offset, file);
//        load_cfg_from_text_file(map_center, file);
//        load_cfg_from_text_file(map_res, file);
//        load_cfg_from_text_file(map_angle_res, file);
//        map_angle_res = LB_DEG2RAD(map_angle_res);
//
//        //particles settings
//        load_cfg_from_text_file(n_particles, file);
//        load_cfg_from_text_file(min_particels, file);
//        load_cfg_from_text_file(a_slow, file);
//        load_cfg_from_text_file(a_fast, file);
//        load_cfg_from_text_file(v_factor, file);
//
//        //motion
//        load_cfg_from_text_file(motion_var[0], file);
//        load_cfg_from_text_file(motion_var[1], file);
//        load_cfg_from_text_file(motion_var[2], file);
//        load_cfg_from_text_file(motion_var[3], file);
//        load_cfg_from_text_file(motion_var[4], file);
//        load_cfg_from_text_file(motion_var[5], file);
//
//        //measurement
//        load_cfg_from_text_file(z_max_range, file);
//        load_cfg_from_text_file(z_hit_var, file);
//        load_cfg_from_text_file(z_short_rate, file);
//        load_cfg_from_text_file(z_weight[0], file);
//        load_cfg_from_text_file(z_weight[1], file);
//        load_cfg_from_text_file(z_weight[2], file);
//        load_cfg_from_text_file(z_weight[3], file);
//        std::cout << "========================================\n";
//
//
//        file.close();
    }
};





}


#endif /* LB_MCL2_H_ */
