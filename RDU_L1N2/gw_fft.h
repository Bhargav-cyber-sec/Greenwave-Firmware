#ifndef GW_FFT_H
#define GW_FFT_H
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
/* 400-point real FFT -> 201 complex bins. */
void gw_fft_r400(const float *in, float *out_re, float *out_im);
#ifdef __cplusplus
}
#endif
#endif
