/*
 * lb_log_file.h
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

#ifndef LB_LOG_FILE_H_
#define LB_LOG_FILE_H_

#include "lb_common.h"

namespace librobotics {
struct log_file {
    std::fstream file;
    int step;
    std::vector<std::string> labels;
    std::string tmp;
    size_t count;
    size_t idx;

    log_file() :
        step(0), count(0), idx(0)
    {
        tmp.reserve(4096);
    }

    virtual ~log_file() {
        close();
    }

    bool open(const std::string& filename) {
        file.open(filename.c_str());
        return file.is_open();
    }

    void close() {
        file.close();
    }


    virtual bool real_one_step() = 0;

};

struct simple_lrf_log : public log_file {
    std::vector<int> ranges;

    simple_lrf_log(const std::string& label)
        : log_file()
    {
        labels.push_back(label);
    }

    virtual bool real_one_step( ) {
        file >> tmp;

        if(file.eof()) {
            return false;
        }

        if(tmp.compare(labels[0]) == 0) {
            file >> count;
            idx = 0;
            if(ranges.size() < count) ranges.resize(count);
            while(!file.eof() && (idx < count)) {
                file >> ranges[idx++];
            }

            if(idx != count) {
                return false;
            }
            step++;
        } else {
            //ignore whole line
            file.ignore(std::numeric_limits<int>::max(), '\n');
            return false;
        }
        return true;
    }

    int real_all(std::vector<std::vector<int> >& ranges_all) {
        while(!file.eof()) {
            if(real_one_step())
                ranges_all.push_back(ranges);
        }
        return step;
    }

};


}

#endif /* LB_LOG_FILE_H_ */
