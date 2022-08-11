#ifndef CORDICCART2POL_H
#define CORDICCART2POL_H

#include <ap_fixed.h>
#include <ap_int.h>

#define NO_ITER 16
#define W 32
#define I 9
#define I 9

typedef int   coef_t;
typedef float data_t;
typedef float acc_t;

typedef ap_fixed<W, I, AP_RND, AP_WRAP, 1> d_t;
typedef ap_uint<2> flag_t;

void cordiccart2pol(data_t x, data_t y, data_t * r,  data_t * theta);

#endif


