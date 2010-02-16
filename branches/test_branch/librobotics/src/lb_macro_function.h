/*
 * lb_macro.h
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

#ifndef LB_MACRO_FUNCTION_H_
#define LB_MACRO_H_

#include "lb_common.h"

// Some useful constants. (copy from GNU math.h)
#if !(defined __USE_BSD || defined __USE_XOPEN)
# define M_E        2.7182818284590452354   /* e */
# define M_LOG2E    1.4426950408889634074   /* log_2 e */
# define M_LOG10E   0.43429448190325182765  /* log_10 e */
# define M_LN2      0.69314718055994530942  /* log_e 2 */
# define M_LN10     2.30258509299404568402  /* log_e 10 */
# define M_PI       3.14159265358979323846  /* pi */
# define M_PI_2     1.57079632679489661923  /* pi/2 */
# define M_PI_4     0.78539816339744830962  /* pi/4 */
# define M_1_PI     0.31830988618379067154  /* 1/pi */
# define M_2_PI     0.63661977236758134308  /* 2/pi */
# define M_2_SQRTPI 1.12837916709551257390  /* 2/sqrt(pi) */
# define M_SQRT2    1.41421356237309504880  /* sqrt(2) */
# define M_SQRT1_2  0.70710678118654752440  /* 1/sqrt(2) */
#endif

/* The above constants are not adequate for computation using `long double's.
   Therefore we provide as an extension constants with similar names as a
   GNU extension.  Provide enough digits for the 128-bit IEEE quad.  */
#ifndef __USE_GNU
# define M_El       2.7182818284590452353602874713526625L  /* e */
# define M_LOG2El   1.4426950408889634073599246810018921L  /* log_2 e */
# define M_LOG10El  0.4342944819032518276511289189166051L  /* log_10 e */
# define M_LN2l     0.6931471805599453094172321214581766L  /* log_e 2 */
# define M_LN10l    2.3025850929940456840179914546843642L  /* log_e 10 */
# define M_PIl      3.1415926535897932384626433832795029L  /* pi */
# define M_PI_2l    1.5707963267948966192313216916397514L  /* pi/2 */
# define M_PI_4l    0.7853981633974483096156608458198757L  /* pi/4 */
# define M_1_PIl    0.3183098861837906715377675267450287L  /* 1/pi */
# define M_2_PIl    0.6366197723675813430755350534900574L  /* 2/pi */
# define M_2_SQRTPIl    1.1283791670955125738961589031215452L  /* 2/sqrt(pi) */
# define M_SQRT2l   1.4142135623730950488016887242096981L  /* sqrt(2) */
# define M_SQRT1_2l 0.7071067811865475244008443621048490L  /* 1/sqrt(2) */
#endif

#define LB_IS_ZERO              (1e-8)
#define LB_VERY_SMALL           (1e-6)
#define LB_SIGN(A)              ((A) >= 0.0 ? (1.0) : (-1.0))
#define LB_SIGN_BOOL(A)         ((A) >= 0.0 ? (true) : (false))
#define LB_ROUND(x)             ((x) < 0 ? ceil((x)-0.5):floor((x)+0.5))
#define LB_SQR(x)               ((x)*(x))
#define LB_SQUARE(x)            (LB_SQR(x))
#define LB_DEG2RAD(x)           (((x)/180.0) * M_PI)
#define LB_RAD2DEG(x)           (((x)/M_PI) * 180.0)
#define LB_SIZE(x,y)            (sqrt(LB_SQR(x) + LB_SQR(y)))
#define LB_SQR_SIZE(x,y)        (LB_SQR(x) + LB_SQR(y))

//compare
#define LB_MIN(a,b)             ((a) <= (b) ? (a) : (b))
#define LB_MIN_BOOL(a,b)        ((a) <= (b) ? true : false)
#define LB_MIN3(a,b,c)          (MIN2(MIN2(a,b), c))

#define LB_MAX(a, b)            (((a) >= (b)) ? (a) : (b))
#define LB_MAX_BOOL(a, b)       (((a) >= (b)) ? true : false)
#define LB_MAX3(a, b, c)        (MAX2(MAX2(a,b), c))

//simple print
#define LB_PRINT_VAR(x)         (LB_PRINT_STREAM << #x << ":" << (x) << "\n")
#define LB_PRINT_VAL(x)         (LB_PRINT_STREAM << (x) << "\n")
#define LB_PRINT_VEC(x) \
    { \
        for(size_t idx = 0; idx < x.size(); idx++) { \
            LB_PRINT_STREAM << idx << ":" << x[idx] << "\n"; \
        } \
    }

#define LB_PRINT_2D_VEC(x) \
    { \
        for(size_t idx1 = 0; idx1 < x.size(); idx1++) { \
            for(size_t idx2 = 0; idx2 < x[idx1].size(); idx2++) { \
                LB_PRINT_STREAM << "(" << x[idx1][idx2] << "),"; \
            } \
            LB_PRINT_STREAM << "\n"; \
        } \
    }

//vector resize
#define LB_RESIZE_2D_VEC(v, x, y) \
    { \
        v.clear(); \
        v.resize(x); \
        for(int idx = 0; idx < x; idx++) { \
            v[idx].resize(y); \
        } \
    }

//check size
#define LB_CHK_SIZE2(v0, v1)                  (v0.size() == v1.size())
#define LB_CHK_SIZE3(v0, v1, v2)              (LB_CHK_SIZE2(v0,v1) && LB_CHK_SIZE2(v0,v2))
#define LB_CHK_SIZE4(v0, v1, v2, v3)          (LB_CHK_SIZE3(v0, v1, v2) && LB_CHK_SIZE2(v0, v3))
#define LB_CHK_SIZE5(v0, v1, v2, v3, v4)      (LB_CHK_SIZE4(v0, v1, v2, v3) && LB_CHK_SIZE2(v0, v4))

//load data from configuration file (external/configfile.h)
#define LOAD_CFG(x, T)  (x = file.read<T>(#x))
#define LOAD_N_SHOW_CFG(x, T)  LOAD_CFG(x, T); LB_PRINT_VAR(x);

#endif /* LB_MACRO_FUNCTION_H_ */
