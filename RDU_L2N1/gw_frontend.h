#ifndef GW_FRONTEND_H
#define GW_FRONTEND_H

#include <stdint.h>
#include <stddef.h>
#include "gw_frontend_tables.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Scratch memory for one inference. ~155 KB -- allocate ONCE, in PSRAM.
 * Do not put this on a FreeRTOS task stack. */
typedef struct {
    float audio[GW_N_SAMPLES];                 /* 128000 B */
    float logmel[GW_N_MELS * GW_N_FRAMES];     /*  25760 B */
    float frame[GW_N_FFT];                     /*   1600 B */
    float re[GW_N_BINS];                       /*    804 B */
    float im[GW_N_BINS];                       /*    804 B */
    float power[GW_N_BINS];                    /*    804 B */
} gw_scratch_t;

void gw_preprocess_audio(const int16_t *pcm, size_t n, float *out);
void gw_logmel(const float *sig, float *logmel_out, gw_scratch_t *s);
void gw_normalize_quantize(const float *logmel, int8_t *out);
void gw_frontend_run(const int16_t *pcm, size_t n, int8_t *out_int8, gw_scratch_t *s);

#ifdef __cplusplus
}
#endif
#endif
