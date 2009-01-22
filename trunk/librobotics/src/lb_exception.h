/*
 * lb_exception.h
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

#ifndef LB_EXCEPTION_H_
#define LB_EXCEPTION_H_

#include "lb_common.h"

namespace librobotics {

    struct LibRoboticsException;

#if (librobotics_use_vt100 == 1)
    const char t_normal[] = { 0x1b,'[','0',';','0',';','0','m','\0' };
    const char t_red[]    = { 0x1b,'[','4',';','3','1',';','5','9','m','\0' };
    const char t_bold[]   = { 0x1b,'[','1','m','\0' };
    const char t_purple[] = { 0x1b,'[','0',';','3','5',';','5','9','m','\0' };
    const char t_green[]  = { 0x1b,'[','0',';','3','2',';','5','9','m','\0' };
#else
    const char t_normal[] = { '\0' };
    const char *const t_red = librobotics::t_normal,
               *const t_bold = librobotics::t_normal,
               *const t_purple = librobotics::t_normal,
               *const t_green = librobotics::t_normal;
#endif

    void inline info() { fprintf(stderr, "LibRobotics V %6.2f\n", librobotics_version); }

    /**
     * Get/set the current LibRobotics exception mode.
     * The way error messages are handled by LibRobotics can be changed dynamically, using this function.
     * Possible values are :
     *      - 0 to hide debug messages (quiet mode, but exceptions are still thrown).
     *      - 1 to display debug messages on standard error (stderr).
     *      - 2 to display debug messages in modal windows (default behavior).
     */
    inline unsigned int& exception_mode() { static unsigned int mode = librobotics_debug; return mode; }


    /*
     * Definition of the LibRobotics: Exception structures
     */
    struct LibRoboticsException {
    #define _librobotics_exception_err(etype, disp_flag) \
        std::va_list ap; va_start(ap,format); std::vsprintf(message,format,ap); va_end(ap); \
        switch (librobotics::exception_mode()) { \
            case 0: break; \
            case 1: \
            case 2: \
                std::fprintf(stderr,"\n%s# %s%s : %s\n\n",librobotics::t_red,etype,librobotics::t_normal,message); \
                break; \
            default: std::fprintf(stderr,"\n%s# %s%s : %s\n\n",librobotics::t_red,etype,librobotics::t_normal,message); \
        } \
        if (librobotics::exception_mode()>=2) librobotics::info();

        char message[1024]; //!< Message associated with the error that thrown the exception.
        LibRoboticsException() {
            message[0] = '\0';
        }
        LibRoboticsException(const char *format, ...) {
            _librobotics_exception_err("LibRoboticsException",true);
        }
    };

    // The LibRoboticsInstanceException class is used to throw an exception related
    // to a non suitable instance encountered in a library function call.
    struct LibRoboticsInstanceException: public LibRoboticsException {
        LibRoboticsInstanceException(const char *format, ...) { _librobotics_exception_err("LibRoboticsInstanceException",true); }
    };

    // The LibRoboticsArgumentException class is used to throw an exception related
    // to invalid arguments encountered in a library function call.
    struct LibRoboticsArgumentException: public LibRoboticsException {
        LibRoboticsArgumentException(const char *format, ...) { _librobotics_exception_err("LibRoboticsArgumentException",true); }
    };

    // The LibRoboticsIOException class is used to throw an exception related
    // to Input/Output file problems encountered in a library function call.
    struct LibRoboticsIOException: public LibRoboticsException {
        LibRoboticsIOException(const char *format, ...) { _librobotics_exception_err("LibRoboticsIOException",true); }
    };

    // The LibRoboticsWarningException class is used to throw an exception for warnings
    // encountered in a library function call.
    struct LibRoboticsWarningException: public LibRoboticsException {
        LibRoboticsWarningException(const char *format, ...) { _librobotics_exception_err("LibRoboticsWarningException",false); }
    };

    // The LibRoboticsRuntimeException class is used to throw an exception for warnings
    // encountered in a library function call.
    struct LibRoboticsRuntimeException: public LibRoboticsException {
        LibRoboticsRuntimeException(const char *format, ...) { _librobotics_exception_err("LibRoboticsRuntimeException",false); }
    };

    /**
     * Display a warning message.
     * \param format is a C-string describing the format of the message, as in <tt>std::printf()</tt>.
     */
    inline void warn(const char *format, ...) {
        if (librobotics::exception_mode() >= 1) {
            char message[8192];
            std::va_list ap;
            va_start(ap,format);
            std::vsprintf(message,format,ap);
            va_end(ap);
            #ifdef librobotics_strict_warnings
                throw LibRoboticsWarningException(message);
            #else
                std::fprintf(stderr,"%s# LibRobotics Warning%s : %s\n",librobotics::t_red,librobotics::t_normal,message);
            #endif
        }
    }
}


#endif /* LB_EXCEPTION_H_ */
