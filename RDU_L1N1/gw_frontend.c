/***********************************************************************
 * GreenWave Phase 6 front end -- C port of phase5_preprocess_reference().
 *
 * Reproduces 07_Specifications/preprocessing_specification.json exactly:
 *   fix length 32000 -> DC removal -> Butterworth HPF 100 Hz (order 4)
 *   -> Butterworth LPF 7500 Hz (order 4) -> peak normalise
 *   -> STFT (n_fft 400, hop 200, hamming, center=True, ZERO padding)
 *   -> power spectrum -> 40-band Slaney mel -> power_to_db(ref=max, top_db=80)
 *   -> (x - mean)/std -> INT8 quantise
 *
 * Output: int8[40*161], mel-major (index = mel*161 + frame), matching the
 * [1,40,161,1] tensor layout the Phase 6 model expects.
 *
 * VALIDATE THIS AGAINST 08_Golden_Test_Vectors BEFORE TRUSTING IT.
 ***********************************************************************/
#include "gw_frontend.h"
#include "gw_frontend_tables.h"
#include <math.h>
#include <string.h>

/* ---- FFT backend -------------------------------------------------- */
/* 400 = 2^4 * 5^2, not a power of two, so a radix-2 FFT will not do.
 * kissfft handles mixed radix. See gw_fft.h for the shim.             */
#include "gw_fft.h"

/* Direct-form II transposed biquad cascade is what scipy.signal.lfilter
 * does NOT do -- lfilter uses direct form II transposed on the full
 * order-4 numerator/denominator. We match lfilter directly.           */
/* scipy.signal.lfilter, direct form II transposed, order 4.
 *
 * ACCUMULATES IN DOUBLE ON PURPOSE. The 100 Hz high-pass at 16 kHz has a
 * normalised cutoff of 0.0125, which puts its poles very close to z = 1:
 *   a = [1, -3.8974, 5.6974, -3.7025, 0.9025]
 * Those coefficients nearly cancel. In float32 the state diverges from the
 * Python reference by ~2e-2 over 32000 samples, which is enough to shift
 * whole mel bands by tens of dB after power_to_db. Measured, not guessed.
 * Do not "optimise" this to float.                                     */
static void lfilter_o4(const float *bf, const float *af, float *x, int n)
{
    const double b0=bf[0], b1=bf[1], b2=bf[2], b3=bf[3], b4=bf[4];
    const double a1=af[1], a2=af[2], a3=af[3], a4=af[4];
    double z1=0.0, z2=0.0, z3=0.0, z4=0.0;
    for (int i = 0; i < n; i++) {
        double xn = (double)x[i];
        double yn = b0*xn + z1;
        z1 = b1*xn - a1*yn + z2;
        z2 = b2*xn - a2*yn + z3;
        z3 = b3*xn - a3*yn + z4;
        z4 = b4*xn - a4*yn;
        x[i] = (float)yn;
    }
}

void gw_preprocess_audio(const int16_t *pcm, size_t n, float *out)
{
    size_t i;
    size_t copy = (n < GW_N_SAMPLES) ? n : GW_N_SAMPLES;

    /* int16 -> float in [-1,1), then zero-pad or truncate to 32000 */
    for (i = 0; i < copy; i++)   out[i] = (float)pcm[i] / 32768.0f;
    for (; i < GW_N_SAMPLES; i++) out[i] = 0.0f;

    /* DC removal (double accumulator: 32000 float32 adds lose ~1e-3) */
    double sum = 0.0;
    for (i = 0; i < GW_N_SAMPLES; i++) sum += (double)out[i];
    double mean = sum / (double)GW_N_SAMPLES;

    /* HPF and LPF cascaded IN ONE PASS, entirely in double.
     * Storing the intermediate as float32 between the two filters is enough
     * to shift mel bands by several dB -- measured. Only the final result
     * is narrowed to float. */
    const double hb0=GW_HPF_B[0], hb1=GW_HPF_B[1], hb2=GW_HPF_B[2], hb3=GW_HPF_B[3], hb4=GW_HPF_B[4];
    const double ha1=GW_HPF_A[1], ha2=GW_HPF_A[2], ha3=GW_HPF_A[3], ha4=GW_HPF_A[4];
    const double lb0=GW_LPF_B[0], lb1=GW_LPF_B[1], lb2=GW_LPF_B[2], lb3=GW_LPF_B[3], lb4=GW_LPF_B[4];
    const double la1=GW_LPF_A[1], la2=GW_LPF_A[2], la3=GW_LPF_A[3], la4=GW_LPF_A[4];

    double h1=0,h2=0,h3=0,h4=0, l1=0,l2=0,l3=0,l4=0;
    double peak = 0.0;

    for (i = 0; i < GW_N_SAMPLES; i++) {
        double xn = (double)out[i] - mean;

        double yh = hb0*xn + h1;
        h1 = hb1*xn - ha1*yh + h2;
        h2 = hb2*xn - ha2*yh + h3;
        h3 = hb3*xn - ha3*yh + h4;
        h4 = hb4*xn - ha4*yh;

        double yl = lb0*yh + l1;
        l1 = lb1*yh - la1*yl + l2;
        l2 = lb2*yh - la2*yl + l3;
        l3 = lb3*yh - la3*yl + l4;
        l4 = lb4*yh - la4*yl;

        out[i] = (float)yl;
        double m = yl < 0 ? -yl : yl;
        if (m > peak) peak = m;
    }

    float inv = (float)(1.0 / (peak + 1e-9));
    for (i = 0; i < GW_N_SAMPLES; i++) out[i] *= inv;
}

/* Fetch one centred frame with ZERO padding (librosa pad_mode='constant').
 * Frame f is centred at sample f*HOP, so it spans
 * [f*HOP - N_FFT/2, f*HOP + N_FFT/2). */
static void get_frame(const float *sig, int f, float *frame)
{
    int start = f * GW_HOP - (GW_N_FFT / 2);
    for (int k = 0; k < GW_N_FFT; k++) {
        int idx = start + k;
        float v = (idx < 0 || idx >= GW_N_SAMPLES) ? 0.0f : sig[idx];
        frame[k] = v * GW_WINDOW[k];
    }
}

void gw_logmel(const float *sig, float *logmel_out, gw_scratch_t *s)
{
    /* --- STFT -> power -> mel ------------------------------------- */
    for (int f = 0; f < GW_N_FRAMES; f++) {
        get_frame(sig, f, s->frame);
        gw_fft_r400(s->frame, s->re, s->im);

        for (int k = 0; k < GW_N_BINS; k++)
            s->power[k] = s->re[k] * s->re[k] + s->im[k] * s->im[k];

        const float *w = GW_MEL_WEIGHTS;
        for (int m = 0; m < GW_N_MELS; m++) {
            int start = GW_MEL_START[m];
            int len   = GW_MEL_LEN[m];
            float acc = 0.0f;
            for (int j = 0; j < len; j++) acc += w[j] * s->power[start + j];
            w += len;
            logmel_out[m * GW_N_FRAMES + f] = acc;   /* mel-major */
        }
    }

    /* --- power_to_db(S, ref=np.max, top_db=80) --------------------- */
    const int N = GW_N_MELS * GW_N_FRAMES;

    float ref = 0.0f;                    /* ref = max over the WHOLE window */
    for (int i = 0; i < N; i++) if (logmel_out[i] > ref) ref = logmel_out[i];
    if (ref < GW_AMIN) ref = GW_AMIN;
    float ref_db = 10.0f * log10f(ref);

    float maxdb = -INFINITY;
    for (int i = 0; i < N; i++) {
        float v = logmel_out[i];
        if (v < GW_AMIN) v = GW_AMIN;
        v = 10.0f * log10f(v) - ref_db;
        logmel_out[i] = v;
        if (v > maxdb) maxdb = v;
    }

    float floor_db = maxdb - GW_TOP_DB;
    for (int i = 0; i < N; i++)
        if (logmel_out[i] < floor_db) logmel_out[i] = floor_db;
}

void gw_normalize_quantize(const float *logmel, int8_t *out)
{
    const int N = GW_N_MELS * GW_N_FRAMES;
    const float inv_std   = 1.0f / GW_FEAT_STD;
    const float inv_scale = 1.0f / GW_IN_SCALE;

    for (int i = 0; i < N; i++) {
        float x = (logmel[i] - GW_FEAT_MEAN) * inv_std;
        float q = roundf(x * inv_scale) + (float)GW_IN_ZERO_POINT;
        if (q < -128.0f) q = -128.0f;
        if (q >  127.0f) q =  127.0f;
        out[i] = (int8_t)q;
    }
}

void gw_frontend_run(const int16_t *pcm, size_t n,
                     int8_t *out_int8, gw_scratch_t *s)
{
    gw_preprocess_audio(pcm, n, s->audio);
    gw_logmel(s->audio, s->logmel, s);
    gw_normalize_quantize(s->logmel, out_int8);
}
