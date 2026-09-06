/***********************************************************************
 * ESP32 FFT backend: 400-point real FFT via kissfft.
 *
 * 400 = 2^4 * 5^2 -- NOT a power of two, so esp-dsp's radix-2 FFT cannot
 * be used and zero-padding to 512 is NOT equivalent (it changes bin
 * spacing, and the mel filterbank is indexed by bin).
 *
 * Vendor kissfft into the sketch folder: kiss_fft.c/h + kiss_fftr.c/h
 * from https://github.com/mborgerding/kissfft (BSD-3). Four files.
 ***********************************************************************/
#include "gw_fft.h"
#include "kiss_fftr.h"
#include <stdlib.h>

#define NFFT 400
#define NBINS 201

static kiss_fftr_cfg s_cfg = NULL;
static kiss_fft_cpx  s_out[NBINS];

void gw_fft_r400(const float *in, float *out_re, float *out_im)
{
    if (s_cfg == NULL) {
        s_cfg = kiss_fftr_alloc(NFFT, 0, NULL, NULL);
    }
    kiss_fftr(s_cfg, (const kiss_fft_scalar *)in, s_out);
    for (int k = 0; k < NBINS; k++) {
        out_re[k] = s_out[k].r;
        out_im[k] = s_out[k].i;
    }
}
