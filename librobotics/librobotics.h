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
#define librobotics_use_vt100

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
#define VERY_SMALL          (1e-6)
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

//angle normalize function
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

//simple print function
#define PRINTVAR(x)         (std::cout << #x << ":" << (x) << std::endl)
#define PRINTVALUE(x)       (std::cout << (x) << std::endl)
#define PRINTVEC(x)         for(uint idx = 0; idx < x.size(); idx++) { \
                                std::cout << idx << "," << x[idx] << std::endl; \
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


    /*-------------------------------------------------------
     *
     * Definition of the LibRobotics: Log files reader (CARMEN format)
     * (http://carmen.sourceforge.net)
     *
     -------------------------------------------------------*/


    template<typename T> struct log_lrf_data2 {
        double time;
        int partial;
        int numvalue;
        std::vector<T> val;
        std::vector<T> angle;
        pose2<T> offset;
        pose2<T> estpos;



    };




    template<typename T> struct log_data {


    };


    /*-------------------------------------------------------------------------
     *
     * Definition of the LibRobotics: Statistic Functions
     *
     -------------------------------------------------------------------------*/

    template<typename T>
    T stat_pdf_normal_1d(T variance, T mean, T x) {
        if(variance < 0) {
            warn("Negative variance: %e", variance);
            variance = -variance;
        }
        if(fabs(variance) <= VERY_SMALL)
            warn("Very low variance: %e", variance);

        return (1.0/sqrt(2*M_PI*variance)) * exp(-SQR(x - mean) / (2*variance));
    }

    template<typename T>
    T stat_pdf_expo_1d(T rate, T x) {
        if(x < 0) return 0;
        return rate * exp(-rate * x);
    }


    /*--------------------------------------------------------------------------------
     *
     * Definition of the LibRobotics: 2D Laser Range Finder (LRF) data structures and functions
     *
     --------------------------------------------------------------------------------*/

    enum lrf_range_condition {
        NO_ERROR        = 0x00,
        ERR_RANGE       = 0x01,     //too far
        ERR_MOVE        = 0x02,     //moving object
        ERR_MIXED       = 0x04,     //mixed pixel
        ERR_OCCLUDED    = 0x08,     //occluded
        ERR_EMPTY       = 0x10      //no measurement
    };

    enum lrf_feature_type {
        TYPE_SEGMENT = 0,
        TYPE_LINE,
        TYPE_ARC,
        TYPE_CORNER,
        TYPE_SMALL_OBJ,
        TYPE_LEG,
        TYPE_LEG2
    };

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

    template<typename T>
    void lrf_init_fi_table(T startTheta,
                           T thetaStep,
                           std::vector<T>& fiTable)
    {
        T tmpTheta;
        for(size_t i = 0; i < fiTable.size(); i++) {
            tmpTheta = startTheta + (i * thetaStep);
            fiTable[i] = tmpTheta;
        }
    }

    template<typename T>
    void lrf_init_cos_sin_table(T startTheta,
                                T thetaStep,
                                std::vector<T>& cosTable,
                                std::vector<T>& sinTable)
    {
        T tmpTheta;
        for(size_t i = 0; (i < cosTable.size()) && (i < sinTable.size()); i++) {
            tmpTheta = startTheta + (i * thetaStep);
            cosTable[i] = cos(tmpTheta);
            sinTable[i] = sin(tmpTheta);
        }
    }

    template<typename T1, typename T2>
    void lrf_scan_range_from_scan_pose(const std::vector<vec2<T1> >& scanPose,
                             std::vector<T2>& ranges,
                             T2 convert = 1.0)
    {
        if(ranges.size() != scanPose.size()) {
            librobotics::warn("scanPose and ranges size are not match");
        }

        for(size_t i = 0; (i < scanPose.size()) && (i < ranges.size()); i++ ) {
            ranges[i] = (T2)scanPose[i].size() * convert;
        }
    }

    template<typename T1, typename T2, typename T3>
    void lrf_scan_pose_from_scan_range(const std::vector<T1>& ranges,
                                       const std::vector<T2>& cosTable,
                                       const std::vector<T2>& sinTable,
                                       std::vector<vec2<T3> >& scanPose,
                                       int step = 1,
                                       bool flip = false)
    {
        size_t nPts = ranges.size();
        if((cosTable.size() != nPts) || (sinTable.size() != nPts)) {
            throw librobotics::LibRoboticsRuntimeException("cos/sin table size are not match");
        }

        vec2<T3> tmp;
        int idx = 0;
        int j = 0;
        for(size_t i = 0; i < nPts; i+=step ) {
            idx = (!flip) ? i : ((nPts - 1) - i);
            scanPose[j].x = ranges[i] * cosTable[idx];
            scanPose[j].y = ranges[i] * sinTable[idx];
            j++;
        }
    }

    template<typename T>
    void lrf_range_glitch_filter(std::vector<T>& ranges,
                                 T value = 0)
    {
        for(size_t i = 1; i < (ranges.size() - 1); i++ ) {
            if(ranges[i-1] != value &&
               ranges[i] == value &&
               ranges[i+1] != value)
            {
                ranges[i] = (T)((ranges[i-1] + ranges[i+1]) / 2.0);
            } else
            if(ranges[i-1] == value &&
               ranges[i] != value &&
               ranges[i+1] == value)
            {
                ranges[i] = value;
            }
        }
    }

    template<typename T>
    void lrf_range_median_filter(std::vector<T>& ranges,
                                 size_t halfWndSide = 2)
    {
        size_t nRange = ranges.size();
        if(nRange < ((halfWndSide * 2) + 1))
            return;

        std::vector<T> r((halfWndSide * 2) + 1,0);
        int k = 0, l = 0;
        for(size_t i = 0; i < nRange; i++) {
            k = 0;
            for(int j = i - halfWndSide ; j <= (int)(i + halfWndSide); j++) {
                l = (j < 0) ? 0 : j;
                l = (j >= (int)nRange) ? (nRange - 1) : l;
                r[k] = ranges[l];
                k++;
            }
            std::sort(r.begin(), r.end());
            ranges[i] = r[halfWndSide];
        }
    }

    template<typename T>
    void lrf_range_threshold(std::vector<T>& ranges,
                             T minValue,
                             T newMinValue = 0,
                             T maxValue = std::numeric_limits<T>::max(),
                             T newMaxValue = 0)
    {
        for(size_t i = 0; i < ranges.size(); i++ ) {
            if(ranges[i] < minValue) {
                ranges[i] = newMinValue;
            }
            else
            if(ranges[i] > maxValue) {
                ranges[i] = newMaxValue;
            }
        }
    }

    template<typename T>
    void lrfRangeThreshold(std::vector<T>& ranges,
                           std::vector<unsigned char>& bad,
                           T minValue,
                           T newMinValue = 0,
                           T maxValue = std::numeric_limits<T>::max(),
                           T newMaxValue = 0)
    {
        if(bad.size() != ranges.size()) {
            throw librobotics::LibRoboticsRuntimeException("bad and ranges size are not match");
        }

        for(size_t i = 0; i < ranges.size(); i++ ) {
            if(ranges[i] <= minValue) {
                ranges[i] = newMinValue;
                bad[i] = ERR_EMPTY;
            }
            else
            if(ranges[i] >= maxValue) {
                ranges[i] = newMaxValue;
                bad[i] = ERR_RANGE;
            } else {
                bad[i] = NO_ERROR;
            }
        }
    }

    template<typename T>
    void lrf_range_segment(const std::vector<T>& ranges,
                           std::vector<int>& seg,
                           T rangeThreshold,
                           T minValue = 0)
    {
      if(seg.size() != ranges.size()) {
          throw librobotics::LibRoboticsRuntimeException("seg and ranges size are not match");
      }

      bool newSegment = true;
      T lastRange = 0;
      int nSegment = 0;

      for(size_t i = 0; i < ranges.size(); i++) {
          if(ranges[i] > minValue) {
              if(newSegment) {
                  //start new segment
                  seg[i] = nSegment;
                  newSegment = false;
              } else {
                  if(fabs(ranges[i] - lastRange) > rangeThreshold) {
                      //start new segment
                      seg[i] = nSegment;
                      newSegment = false;

                      //end current segment
                      nSegment++;
                  } else {
                      //continue current segment
                      seg[i] = nSegment;
                      lastRange = ranges[i];
                  }
              }
          } else {
              if(!newSegment) {
                  //end current segment
                  nSegment++;
                  newSegment = true;
              } else {
                  //ignore all value below thresholds
              }
          }
      }
    }

    /*--------------------------------------------------------------------------------
     *
     * Definition of the LibRobotics: 2D Measurement and Motion Model
     *
     --------------------------------------------------------------------------------*/

    namespace model {
        namespace measurement {
            /*---------------------------------------------------------
             *
             * From Probabilistic Robotics (Ch.6 - Robot Perception)
             * (http://robots.stanford.edu/probabilistic-robotics/)
             *
             ---------------------------------------------------------*/

            template<typename T>
            T range_finder_beam_model(T x, T x_mean, T x_max, T cov_hit, T rate_short) {
                T p_hit =   2*stat_pdf_normal_1d(cov_hit, x_mean, x);
                T p_short = ((x >= 0) && (x <= x_mean)) ? (1/(1-exp(-rate_short))) * stat_pdf_expo_1d(rate_short, x) : 0;
                T p_max =   (x == x_max ? 1 : 0);
                T p_rand =  ((x >= 0) && (x < x_max)) ? 1/x_max : 0;
                return p_hit + p_short + p_max  + p_rand;
            }
        }

        namespace motion {

        }

    }





    /*-----------------------------------------------------------
     *
     * Definition of the LibRobotics: Polar Scan Match Algorithm
     * (http://www.irrc.monash.edu.au/adiosi/downloads.html)
     *
     -----------------------------------------------------------*/


    typedef struct lrf_psm_cfg {
        lrf_psm_cfg() :
            maxError(0.05), searchWndAngle(DEG2RAD(20)),
            lrfMaxRange(4.0), lrfMinRange(0.1),
            minValidPts(50), maxIter(20), smallCorrCnt(5)
        { }

        double maxError;
        double searchWndAngle;
        double lrfMaxRange;
        double lrfMinRange;
        int minValidPts;
        int maxIter;
        int smallCorrCnt;
    };

    template <typename T>
    bool lrf_psm(const pose2<T>& refRobotPose,
                 const pose2<T>& refLaserPose,
                 const std::vector<T>& refScanRanges,
                 const std::vector<unsigned char> refbad,
                 const std::vector<int> refseg,
                 const pose2<T>& actRobotPose,
                 const pose2<T>& actLaserPose,
                 const std::vector<T>& actScanRanges,
                 const std::vector<unsigned char> actbad,
                 const std::vector<int> actseg,
                 pose2<T>& relLaserPose,           //scan match result
                 pose2<T>& relRobotPose,           //scan match result
                 const std::vector<T>& pm_fi,      //angle lookup table
                 const std::vector<T>& pm_co,      //cosine lookup table
                 const std::vector<T>& pm_si,      //sine lookup table
                 const lrf_psm_cfg& cfg = lrf_psm_cfg())
    {
        vec2<T> relRbPose = (actRobotPose.p - refRobotPose.p).rot(-refRobotPose.a);
        T relRbTheta = actRobotPose.a -refRobotPose.a;

        //transformation of actual scan laser scanner coordinates into reference
        //laser scanner coordinates
        vec2<T> actLrfPose = relRbPose + actLaserPose.p.rot(relRbTheta);
        T actLrfTheta = relRbTheta + actLaserPose.a;

        vec2<T> relLrfPose = actLrfPose - refLaserPose.p;
        T relLrfTheta = norm_a(actLrfTheta - refLaserPose.a);


        std::vector<T> refranges(refScanRanges);
        std::vector<T> actranges(actScanRanges);

        //some variables
        size_t nPts = refranges.size();
        std::vector<T> r(nPts, 0);
        std::vector<T> fi(nPts, 0);
        std::vector<T> new_r(nPts, 0);
        std::vector<unsigned char> new_bad(nPts, ERR_EMPTY);
        std::vector<int> index(nPts, 0);

        double angleStep = pm_fi[1] - pm_fi[0];
        int small_corr_cnt = 0;
        int iter = -1;
        int n = 0;//, n2 = 0;
        T dx = 0, dy = 0, dth = 0;
        T ax = relLrfPose.x,  ay = relLrfPose.y, ath = relLrfTheta;
        T delta = 0, x = 0, y = 0, xr = 0, yr = 0;
        T abs_err = 0;
        T ri = 0;
        int idx = 0;
        size_t i = 0;

        while((++iter < cfg.maxIter) && (small_corr_cnt < cfg.smallCorrCnt)) {
            T corr = fabs(dx)+fabs(dy)+fabs(dth);
            if(corr < 0.001) {
                small_corr_cnt++;
            }
            else
                small_corr_cnt = 0;

            // convert range readings into ref frame
            // this can be speeded up, by connecting it with the interpolation
            for(i = 0; i < nPts; i++)
            {
                delta   = ath + pm_fi[i];
                xr = (actScanRanges[i] * cos(delta));
                yr = (actScanRanges[i] * sin(delta));
                x       = xr + ax;
                y       = yr + ay;
                r[i]    = sqrt((x*x)+(y*y));
                fi[i]   = atan2(y,x);
                new_r[i]  = 1e6;            //initialize big interpolated r;
                new_bad[i] = ERR_EMPTY;
            }//for i

            //------------------------INTERPOLATION------------------------
            //calculate/interpolate the associations to the ref scan points

            for(i = 1; i < nPts; i++) {
                // i and i-1 has to be in the same segment, both shouldn't be bad
                // and they should be bigger than 0
                if( actseg[i] >= 0 &&                           //is a segment
                    actseg[i] == actseg[i-1] &&                 //same segment
                    actranges[i] > 0 &&                         //has a measurement
                    (actbad[i] == 0) && (actbad[i-1] == 0))     //is a good measurement
                {
                    //calculation of the "whole" parts of the angles
                    T fi0 = 0, fi1 = 0;
                    T r0 = 0, r1 = 0, a0 = 0, a1 = 0;
                    bool occluded = false;

                    //are the points visible?
                    if(fi[i] > fi[i-1]) {
                        occluded = false;
                        a0  = fi[i-1];
                        a1  = fi[i];
                        fi0 = fi[i-1];//fi0 is the meas. angle!
                        fi1 = fi[i];
                        r0  = r[i-1];
                        r1  = r[i];
                    } else {
                        //invisible - still have to calculate to filter out points which
                        occluded = true; //are covered up by these!
                        //flip the points-> easier to program
                        a0  = fi[i];
                        a1  = fi[i-1];
                        fi0 = fi[i];
                        fi1 = fi[i-1];
                        r0  = r[i];
                        r1  = r[i-1];
                    }

                    //interpolate for all the measurement bearings between fi0 and fi1

                    while(fi0 <= fi1)//if at least one measurement point difference, then ...
                    {
                        //linear interpolate by r and theta ratio
                        ri = (((r1-r0)/(a1-a0))*(fi0 - a0)) + r0;

                        //if fi0 -> falls into the measurement range and ri is shorter
                        //than the current range then overwrite it
                        idx = (int)(fi0/angleStep);
                        idx += (nPts/2);
                        if((idx < (int)nPts) && new_r[idx]>ri) {
                            new_r[idx]    = ri; //overwrite the previous reading
                            index[idx]    = i;  //store which reading was used at index fi0

                            new_bad[idx]  &= ~ERR_EMPTY;    //clear the empty flag

                            //check if it was occluded
                            if(occluded) {
                                new_bad[idx] = ERR_OCCLUDED;//set the occluded flag
                            } else {
                                new_bad[idx] = NO_ERROR;
                            }

                            //the new range reading also it has to inherit the other flags
                            new_bad[idx] |= actbad[i];
                            new_bad[idx] |= actbad[i-1];
                        }
                        fi0 += angleStep;//check the next measurement angle!
                    }//while
                }//if
            }//for

            //---------------ORIENTATION SEARCH-----------------------------------
            //search for angle correction using cross correlation
            if((iter % 2) == 0) {
                T e;
                std::vector<T> err;         // the error rating
                std::vector<T> beta;        // angle for the corresponding error
                int ii = 0;
                int min_i = 0, max_i = 0;
                int wnd = (int)(cfg.searchWndAngle / angleStep);
                T dr;
                for(int di = -wnd ; di <= wnd; di++) {
                    n = 0; e = 0;
                    min_i = MAX2(-di, 0);
                    max_i = MIN2(nPts, nPts-di);
                    for(ii = min_i; ii < max_i; ii++)//searching through the actual points
                    {
                        if((new_bad[ii] == 0) && (refbad[ii+di] == 0))
                        {
                            dr = fabs(new_r[ii]-refranges[ii+di]);
                            e += dr;
                            n++;
                        }
                    }//for i

                    if(n > 0)
                        err.push_back(e/n);
                    else
                        err.push_back(1e6); //very large error

                    beta.push_back(di*angleStep);
                }//for di

                //now search for the global minimum
                //later I can make it more robust
                //assumption: monomodal error function!
                T emin = 1e6;
                int imin = 0;
                for(i = 0; i < err.size(); i++) {
                    if(err[i] < emin) {
                        emin = err[i];
                        imin = i;
                    }
                }

                if(err[imin] >= 1e6)
                {
                    warn("Polar Match: orientation search failed: %f", err[imin]);
                    return false;
                } else {
                    dth = beta[imin];
                    if(imin >= 1 && (imin < (int)(beta.size()-1))) //is it not on the extreme?
                    {//lets try interpolation
                        T D = err[imin-1] + err[imin+1] - 2.0*err[imin];
                        T d = 1000;
                        if((fabs(D) > 0.01) &&
                           (err[imin-1] > err[imin]) &&
                           (err[imin+1] > err[imin]))
                        {
                            d = ((err[imin-1] - err[imin+1]) / D) / 2.0;
                            warn("ORIENTATION REFINEMENT: %d ", d);
                        }

                        if(fabs(d) < 1) {
                            dth += DEG2RAD(d);
                        }
                        ath += dth;
                    }
                }
                continue;
            }//if

            //-----------------translation-------------
            T C = 70*70;
            if(iter > 10) C = 100;
            // do the weighted linear regression on the linearized ...
            // include angle as well
            T hi1, hi2, hwi1, hwi2, hw1 = 0, hw2 = 0, hwh11 = 0;
            T hwh12 = 0, hwh21 = 0, hwh22 = 0, w;
            T dr;
            abs_err = 0;
            n = 0;
            for (i = 0; i < nPts; i++) {
                dr = refranges[i] - new_r[i];

                //weight calculation
                if (refbad[i] == 0 && new_bad[i] == 0 &&
                    refranges[i] > 0 &&
                    new_r[i] > 0 &&
                    new_r[i] < cfg.lrfMaxRange &&
                    new_r[i] > cfg.lrfMinRange &&
                    fabs(dr) < cfg.maxError )
                {
                    abs_err += fabs(dr);
                    n++;
                    w = C / (dr * dr + C);

                    //proper calculations of the jacobian
                    hi1 = pm_co[i];
                    hi2 = pm_si[i];

                    hwi1 = hi1 * w;
                    hwi2 = hi2 * w;

                    //par = (H^t*W*H)^-1*H^t*W*dr
                    hw1 += hwi1 * dr;//H^t*W*dr
                    hw2 += hwi2 * dr;

                    //H^t*W*H
                    hwh11 += hwi1 * hi1;
                    hwh12 += hwi1 * hi2;
                    //        hwh21 += hwi2*hi1; //should take adv. of symmetricity!!
                    hwh22 += hwi2 * hi2;
                }
            }//for i

            if(n < cfg.minValidPts) {
                dx = 10;
                dy = 10;
                warn("pm_linearized_match: ERROR not enough points: %d", n);
                return false;
            }


            //calculation of inverse
            T D;//determinant
            T inv11,inv21,inv12,inv22;//inverse matrix
            D = (hwh11*hwh22) - (hwh12*hwh21);
            if(D < 0.001)
            {
                dx = 10;
                dy = 10;
                warn("pm_linearized_match: ERROR determinant to small: %f", D);
                return false;
            }

            inv11 =  hwh22/D;
            inv12 = -hwh12/D;
            inv21 = -hwh12/D;
            inv22 =  hwh11/D;

            dx = inv11*hw1+inv12*hw2;
            dy = inv21*hw1+inv22*hw2;

            ax += dx;
            ay += dy;
        }//while

        relLaserPose.p.x = ax;
        relLaserPose.p.y = ay;
        relLaserPose.a = ath;
        return true;
    }



    namespace slam {

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

            struct grid_line_t{
              int                   numgrids;
              std::vector<vec2i>    grid;
              grid_line_t() :
                  numgrids(0)
              { }
            } ;

            struct QUAD_TREE {
                struct QUAD_TREE*   elem[4];
                vec2i               center;
                unsigned char       level;
                bool                inuse;
            };

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

        namespace ekf_slam2D {

        }

    }

    namespace localization {

        /*-------------------------------------------------------------
         *
         * Definition of the LibRobotics: EKF Localization in 2D space
         *
         -------------------------------------------------------------*/

        namespace ekf_localization2D {

        }

        /*-------------------------------------------------------------
         *
         * Definition of the LibRobotics: Monte Carlo Localization MCL in 2D space
         * (for feature map)
         *
         -------------------------------------------------------------*/

        namespace mcl_feature2D {

        }

        /*-------------------------------------------------------------
         *
         * Definition of the LibRobotics: Monte Carlo Localization MCL in 2D space
         * (for grid map)
         *
         -------------------------------------------------------------*/

        namespace mcl_grid2D {

        }
    }


}

#endif //#ifndef librobotics_version
