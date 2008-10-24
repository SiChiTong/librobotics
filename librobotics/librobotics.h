/*
 *  File(s)     : librobotics.h (C++ header)
 *
 *  Description : The C++ Robotics Library.
 *                This file is the main part of the LibRobotics project.
 *                ( http://code.google.com/p/librobotics/ )
 *
 *  Created on  : Oct 24, 2008
 *      Author  : Mahisorn Wongphati
 *
 *  Copyright (c) <2008> <Mahisorn Wongphati>
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

// Define version number of the current file.
//

#ifndef librobotics_version
#define librobotics_version 0.1


// Include required standard C++ headers.
//
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <cstring>
#include <cmath>
#include <ctime>
#include <vector>
#include <list>
#include <algorithm>

/*-----------------------------------------------------------
 #
 # Test/auto-set LibRobotics configuration variables
 # and include required headers.
 #
 # If you find that default configuration variables are
 # not adapted, you can override their values before including
 # the header file "librobotics.h" (using the #define directive).
 #
 ------------------------------------------------------------*/

// Operating system configuration.
//
// Define 'librobotics_OS' to : 0 for an unknown OS (will try to minize library dependancies).
//                              1 for a Unix-like OS (Linux, Solaris, BSD, MacOSX, Irix, ...).
//                              2 for Microsoft Windows.
//
#ifndef librobotics_OS
#if defined(unix)        || defined(__unix)      || defined(__unix__) \
 || defined(linux)       || defined(__linux)     || defined(__linux__) \
 || defined(sun)         || defined(__sun) \
 || defined(BSD)         || defined(__OpenBSD__) || defined(__NetBSD__) \
 || defined(__FreeBSD__) || defined __DragonFly__ \
 || defined(sgi)         || defined(__sgi) \
 || defined(__MACOSX__)  || defined(__APPLE__) \
 || defined(__CYGWIN__)
#define librobotics_OS 1
#elif defined(_MSC_VER) || defined(WIN32)  || defined(_WIN32) || defined(__WIN32__) \
   || defined(WIN64)    || defined(_WIN64) || defined(__WIN64__)
#define librobotics_OS 2
#else
#define librobotics_OS 0
#endif
#elif !(librobotics_OS==0 || librobotics_OS==1 || librobotics_OS==2)
#error LibRobotics Library : Configuration variable 'librobotics_OS' is badly defined.
#error (valid values are '0=unknown OS', '1=Unix-like OS', '2=Microsoft Windows').
#endif

// Compiler configuration.
//
// Try to detect Microsoft VC++ compilers.
// (lot of workarounds are needed afterwards to
//  make LibRobotics working, particularly with VC++ 6.0).
//
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4311)
#pragma warning(disable:4312)
#pragma warning(disable:4800)
#pragma warning(disable:4804)
#pragma warning(disable:4996)
#define _CRT_SECURE_NO_DEPRECATE 1
#define _CRT_NONSTDC_NO_DEPRECATE 1
#if _MSC_VER<1300
#define librobotics_use_visualcpp6
#define _WIN32_WINNT 0x0500
#endif
#endif

// Include OS-specific headers.
//
#if librobotics_OS==1
#include <sys/time.h>
#include <unistd.h>
#elif librobotics_OS==2
#include <windows.h>
#ifndef _WIN32_IE
#define _WIN32_IE 0x0400
#endif
#include <shlobj.h>
#ifdef librobotics_use_visualcpp6
#define std
#endif
#endif

// Output messages configuration.
//
// Define 'librobotics_debug' to : 0 to hide debug messages (quiet mode, but exceptions are still thrown).
//                          1 to display debug messages on standard error (stderr).
//                          2 to display debug messages with dialog windows (default behavior).
//                          3 to do as 1 + add extra warnings (may slow down the code !).
//                          4 to do as 2 + add extra warnings (may slow down the code !).
//
// Define 'librobotics_strict_warnings' to replace warning messages by exception throwns.
//
// Define 'librobotics_use_vt100' to allow output of color messages (require VT100-compatible terminal).
//

//#define librobotics_strict_warnings

#ifndef librobotics_debug
#define librobotics_debug 2
#elif !(librobotics_debug==0 || librobotics_debug==1 || librobotics_debug==2 || librobotics_debug==3 || librobotics_debug==4)
#error LibRobotics Library : Configuration variable 'librobotics_debug' is badly defined.
#error (valid values are '0=quiet', '1=stderr', '2=dialog', '3=stderr+warnings', '4=dialog+warnings').
#endif

// CImg configuration.
// (http://cimg.sourceforge.net)
//
// Define 'librobotics_use_cimg' to enable CImg support.
//
// Using CImg is not mandatory.
//
#ifdef librobotics_use_cimg
#include <CImg.h>
#endif

// OpenGL configuration.
// (www.opengl.org)
//
// Define 'librobotics_use_opengl' to enable OpenGL support.
//
// Using OpenGL is not mandatory.
//
#ifdef librobotics_use_opengl
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif


/*------------------------------------------------------------------------------
 *
 * Define user-friendly macros.
 *
 * User macros are prefixed by 'librobotics_' and can be used in your own code.
 *
 ------------------------------------------------------------------------------*/
#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

//etc
#define SIGN(A)             ((A) >= 0.0 ? (1.0) : (-1.0))
#define IS_ZERO             (1e-8)
#define SQR(x)              ((x)*(x))
#define SQUARE(x)           ((x)*(x))
#define DEG2RAD(x)          ((x)/180.0 * M_PI)
#define RAD2DEG(x)          ((x)/M_PI * 180.0)
#define SIZE(x, y)          (sqrt(SQR(x) + SQR(y)))

//compare
#define MIN2(a, b)          (((a) <= (b)) ? (a) : (b))
#define MIN(a, b)           (((a) <= (b)) ? (a) : (b))
#define MIN3(a, b, c)       (MIN2(MIN2(a,b), c))
#define MAX2(a, b)          (((a) >= (b)) ? (a) : (b))
#define MAX(a, b)           (((a) >= (b)) ? (a) : (b))
#define MAX3(a, b, c)       (MAX2(MAX2(a,b), c))

template<typename T>
T inline norm_a_rad(T a) {
    int m = (int)(a / (2.0*M_PI));
    a = a - (m * M_PI * 2.0);
    if (a < (-M_PI))
        a += (2.0 * M_PI);
    if (a >= M_PI)
        a -= (2.0 * M_PI);
    return a;
};

template<typename T>
T inline norm_a_deg(T a) {
    return RAD2DEG(norm_a_rad(DEG2RAD(a)));
}



/*------------------------------------------------
 *
 * Definition of the librobotics:: namespace
 *
 -------------------------------------------------*/

namespace librobotics {

    struct LibRoboticsException;

#ifdef librobotics_use_vt100
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

    void inline info() { fprintf(stderr, "LibRobotics V 0.1\n"); }

    //! Get/set the current LibRobotics exception mode.
    /**
         The way error messages are handled by LibRobotics can be changed dynamically, using this function.
         Possible values are :
             - 0 to hide debug messages (quiet mode, but exceptions are still thrown).
             - 1 to display debug messages on standard error (stderr).
             - 2 to display debug messages in modal windows (default behavior).
             - 3 to do as 1 + add extra warnings (may slow down the code !).
             - 4 to do as 2 + add extra warnings (may slow down the code !).
     **/
    inline unsigned int& exception_mode() { static unsigned int mode = librobotics_debug; return mode; }


    /*----------------------------------------------
     *
     * Definition of the LibRobotics Exception structures
     *
     ----------------------------------------------*/
    struct LibRoboticsException {
    #define _librobotics_exception_err(etype, disp_flag) \
        std::va_list ap; va_start(ap,format); std::vsprintf(message,format,ap); va_end(ap); \
        switch (librobotics::exception_mode()) { \
            case 0: break; \
            case 2: \
            case 4: \
                std::fprintf(stderr,"\n%s# %s%s :\n%s\n\n",librobotics::t_red,etype,librobotics::t_normal,message); \
                break; \
            default: std::fprintf(stderr,"\n%s# %s%s :\n%s\n\n",librobotics::t_red,etype,librobotics::t_normal,message); \
        } \
        if (librobotics::exception_mode()>=3) librobotics::info();

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

    //! Display a warning message.
    /**
     *   \param format is a C-string describing the format of the message, as in <tt>std::printf()</tt>.
     **/
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
                std::fprintf(stderr,"\n%s# LibRobotics Warning%s :\n%s\n",librobotics::t_red,librobotics::t_normal,message);
            #endif
        }
    }


    /*-------------------------------------------------------------------------
     *
     * Definition of the LibRobotics: 2D robot configuration structures and functions
     *
     -------------------------------------------------------------------------*/

    template<typename T> struct vec2 {
        T x, y;

        vec2() : x(0), y(0) { }

        vec2(T xx, T yy) : x(xx), y(yy) { }

        template<typename T1>
        vec2(const vec2<T1>& v) : x((T)v.x), y((T)v.y) { }

        vec2 operator - ( ) const { return vec2(-x, -y); }

        template<typename T1>
        vec2 operator + (const vec2<T1>& v) const { return vec2(x + (T)v.x, y + (T)v.y); }

        template<typename T1>
        vec2 operator - (const vec2<T1>& v) const { return vec2(x - (T)v.x, y - (T)v.y); }

        template<typename T1>
        vec2& operator = (const vec2<T1>& v) {
            x = v.x;
            y = v.y;
            return *this;
        }

        template<typename T1>
        vec2& operator += (const vec2<T1>& v) {
            x += v.x;
            y += v.y;
            return *this;
        }

        template<typename T1>
        vec2& operator -= (const vec2<T1>& v) {
            x -= v.x;
            y -= v.y;
            return *this;
        }

        template<typename T1>
        T operator ^ (const vec2<T1>& v) const {   // dot product
            return (T)((x*v.x) + (y*v.y));
        }

        template<typename T1>
        T operator * (const vec2<T1>& v) const {    // cross product
            return (T)((x*v.y) - (y*v.x));
        }

        template<typename T1>
        vec2 operator * (const T1& s) const {
            return vec2(x*s, y*s);
        }

        template<typename T1>
        vec2 operator / (const T1& s) const {
            return vec2(x/s, y/s);
        }

        template<typename T1>
        vec2& operator *= (const T1& s) {
            x *= s;
            y *= s;
            return (*this);
        }

        template<typename T1>
        vec2& operator /= (const T1& s) {
            x /= s;
            y /= s;
            return (*this);
        }

        T size( ) const {
            return (T)sqrt(x*x + y*y);
        }

        T sqrSize( ) const {
            return (T)(x*x + y*y);
        }

        T theta() const {
            return (T)atan2(y,x);
        }

        T degree() const {
            return (T)RAD2DEG(atan2(y,x));
        }

        bool isZero() const {
            return (this->size() <= IS_ZERO);
        }

        vec2 norm() const {
            if (isZero()) {
                return vec2();
            } else {
                return vec2((*this)/size());
            }
        }

        vec2& normalize() {
            if (isZero()) {
                x = 0;
                y = 0;
                return *this;
            } else {
                return (*this)/size();
            }
        }

        template<typename T1>
        vec2 rot(T1 angle, bool rad = true) const {
            if(!rad) angle = DEG2RAD(angle);
            T1 c = cos(angle);
            T1 s = sin(angle);
            return vec2((T)(x*c - y*s), (T)(x*s + y*c));
        }

        template<typename T1>
        vec2& rotate(T1 angle, bool rad = true) {
            vec2<T> tmp = this->rot(angle, rad);
            x = tmp.x;
            y = tmp.y;
            return *this;
        }

        void print() {
            std::cout << *this << std::endl;
        }

        /// support for output stream
        friend std::ostream& operator << (std::ostream& os, const vec2<T>& v) {
            return os << v.x << " " << v.y;
        }

        /// support for input stream
        friend std::istream& operator >> (std::istream& is, vec2<T>& v) {
            is >> v.x >> v.y;
            return is;
        }
    };
    typedef vec2<int> vec2i;
    typedef vec2<long> vec2l;
    typedef vec2<float> vec2f;
    typedef vec2<double> vec2d;




    template<typename T> struct pose2 {
        T x, y, a;

        pose2() : x(0), y(0), a(0)
        { }

        template<typename T1>
        pose2(T1 xx, T1 yy, T1 aa) :
            x(xx), y(yy), a(aa)
        { }

        template<typename T1>
        pose2(vec2<T1> p, T1 aa) :
            x(p.x), y(p.y), a(aa)
        { }

        template<typename T1>
        pose2(const pose2<T1>& p2) :
            x(p2.x), y(p2.y), a(p2.a)
        { }

        template<typename T1>
        pose2 operator + (const pose2<T1>& pose) const {
            return pose2(pose.x + x, pose.y + y, pose.a + a);
        }

        template<typename T1>
        pose2 operator - (const pose2<T1>& pose) const {
            return pose2(x + pose.x, y + pose.y, a + pose.a);
        }

        void print() {
            std::cout << *this << std::endl;
        }

        /// support for output stream
        friend std::ostream& operator << (std::ostream& os, const pose2<T>& p2) {
            return os << p2.x << " " << p2.y << " " << p2.a;
        }

        /// support for input stream
        friend std::istream& operator >> (std::istream& is, pose2<T>& p2) {
            is >> p2.x >> p2.y >> p2.a;
            return is;
        }

    };
    typedef pose2<float> pose2f;
    typedef pose2<double> pose2d;

    template<typename T> struct bbox2 {
        vec2<T> min, max;
    };
    typedef bbox2<int> bbox2i;
    typedef bbox2<long> bbox2l;
    typedef bbox2<float> bbox2f;
    typedef bbox2<double> bbox2d;


    /*-------------------------------------------------------------------------
     *
     * Definition of the LibRobotics: 3D robot configuration structures and functions
     *
     -------------------------------------------------------------------------*/

    template<typename T> struct vec3 {
        T x, y, z;
        vec3() : x(0), y(0), z(0) { }
    };
    typedef vec3<int> vec3i;
    typedef vec3<long> vec3l;
    typedef vec3<float> vec3f;
    typedef vec3<double> vec3d;

    template<typename T> struct pose3 {
        T x, y, z, ax, ay, az;
        pose3() : x(0), y(0), z(0), ax(0), ay(0), az(0) { }
    };
    typedef pose3<float> pose3f;
    typedef pose3<double> pose3d;

    template<typename T> struct bbox3 {
        vec3<T> min, max;
    };
    typedef bbox3<int> bbox3i;
    typedef bbox3<long> bbox3l;
    typedef bbox3<float> bbox3f;
    typedef bbox3<double> bbox3d;

    /*--------------------------------------------------------------------------------
     *
     * Definition of the LibRobotics: 2D Laser Range Finder (LRF) data structures and functions
     *
     --------------------------------------------------------------------------------*/

    template<typename T1, typename T2> struct lrf_data {
        std::vector<T1> ranges;
        std::vector<T2> angles;
    };
    typedef lrf_data<int, float> lrf_data_if;
    typedef lrf_data<int, double> lrf_data_id;

    template<typename T1, typename T2>
    void lrf_init_lrf_data_angle(lrf_data<T1, T2>& lrf, T2 startTheta, T2 thetaStep) {
        if(lrf.angles.size() == 0) {
            librobotics::warn("size of angles is 0");
        }

        T2 tmpTheta;
        for(size_t i = 0; i < lrf.angles.size(); i++) {
            tmpTheta = startTheta + (i * thetaStep);
            lrf.angles[i] = tmpTheta;
        }
    }




    /*-------------------------------------------------
     *
     * Definition of the LibRobotics: DPSLAM Algorithm
     * (http://www.openslam.org/dpslam.html)
     *
     -------------------------------------------------*/

    namespace dpslam {

    }

    /*----------------------------------------------------
     *
     * Definition of the LibRobotics: GridSLAM Algorithm
     * (http://www.openslam.org/gridslam.html)
     *
     ----------------------------------------------------*/

    namespace gridslam {
        typedef struct QUAD_TREE {
            struct QUAD_TREE*   elem[4];
            vec2i               center;
            unsigned char       level;
            bool                inuse;
        } QUAD_TREE;


        template<typename T> struct gridmap2 {
            QUAD_TREE                           qtree;
            pose2<T>                            offset;
            T                                   resolution;
            std::vector<std::vector<bool> >     updated;
            std::vector<std::vector<T> >        maphit;
            std::vector<std::vector<int> >      mapsum;
            std::vector<std::vector<T> >        mapprob;
            std::vector<std::vector<T> >        calc;
            vec2i                               mapsize;
            vec2<T>                             center;

            bool inline isInside(int x, int y) {
                return (x >= 0) && (x < mapsize.x) && (y >= 0) && (y < mapsize.y);
            }
        };
    }

    /*----------------------------------------------------
     *
     * Definition of the LibRobotics: EKF SLAM in 2D space
     *
     ----------------------------------------------------*/

    namespace ekfslam2 {

    }

    /*-------------------------------------------------------------
     *
     * Definition of the LibRobotics: EKF Localization in 2D space
     *
     -------------------------------------------------------------*/

    namespace ekflocalization2 {

    }


}

#endif //#ifndef librobotics_version
