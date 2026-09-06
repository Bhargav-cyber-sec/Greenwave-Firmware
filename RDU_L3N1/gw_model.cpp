/***********************************************************************
 * TFLite Micro wrapper for greenwave_siren_dscnn_int8.tflite.
 *
 * Operator set comes from the Phase 6 compatibility report -- 6 distinct
 * neural-network operators, 0 unresolved. If AllocateTensors() fails with
 * a missing-op error, the model changed and the resolver below must too.
 ***********************************************************************/
#include "gw_model.h"
#include "gw_frontend_tables.h"
#include "gw_model_data.h"          /* xxd -i output */

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "esp_heap_caps.h"
#include <Arduino.h>

/* MEASURED ON TARGET 2026-08-10: arena_used_bytes() = 56204.
 * This closes Open Item 4 from the Phase 6 report, which recorded the
 * tensor arena as NOT_MEASURED_ON_TARGET. 64 KiB gives ~16% headroom.
 * Record 56204 in 07_Specifications/memory_report.json.
 *
 * Note this is the figure for REFERENCE kernels. Enabling ESP-NN (Phase 7,
 * via ESP-IDF or PlatformIO) may change it -- re-measure if you switch. */
#define GW_ARENA_SIZE (64 * 1024)

static uint8_t                        *s_arena = nullptr;
static tflite::MicroInterpreter       *s_interp = nullptr;
static TfLiteTensor                   *s_in = nullptr;
static TfLiteTensor                   *s_out = nullptr;

bool gw_model_init(void)
{
    const tflite::Model *model = tflite::GetModel(gw_model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        Serial.printf("[MODEL] schema %lu != %d\n",
                      (unsigned long)model->version(), TFLITE_SCHEMA_VERSION);
        return false;
    }

    s_arena = (uint8_t *)heap_caps_aligned_alloc(16, GW_ARENA_SIZE,
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_arena) {
        s_arena = (uint8_t *)heap_caps_aligned_alloc(16, GW_ARENA_SIZE, MALLOC_CAP_8BIT);
    }
    if (!s_arena) { Serial.println("[MODEL] arena alloc failed"); return false; }

    static tflite::MicroMutableOpResolver<6> resolver;
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddMaxPool2D();
    resolver.AddMean();
    resolver.AddFullyConnected();
    resolver.AddLogistic();

    static tflite::MicroInterpreter interp(model, resolver, s_arena, GW_ARENA_SIZE);
    s_interp = &interp;

    if (s_interp->AllocateTensors() != kTfLiteOk) {
        Serial.println("[MODEL] AllocateTensors failed - raise GW_ARENA_SIZE");
        return false;
    }

    s_in  = s_interp->input(0);
    s_out = s_interp->output(0);

    Serial.printf("[MODEL] arena used: %d / %d bytes\n",
                  (int)s_interp->arena_used_bytes(), GW_ARENA_SIZE);
    Serial.printf("[MODEL] input  dims %d x %d x %d x %d  type %d\n",
                  s_in->dims->data[0], s_in->dims->data[1],
                  s_in->dims->data[2], s_in->dims->data[3], s_in->type);
    Serial.printf("[MODEL] input  scale %.17g zp %d\n",
                  s_in->params.scale, s_in->params.zero_point);
    Serial.printf("[MODEL] output scale %.17g zp %d\n",
                  s_out->params.scale, s_out->params.zero_point);

    /* Hard gate: the quantisation the front end assumes must be the
     * quantisation the model actually carries. A silent mismatch here is
     * the exact class of bug the golden vectors exist to catch. */
    if (s_in->type != kTfLiteInt8 || s_out->type != kTfLiteInt8) {
        Serial.println("[MODEL] FATAL: tensors are not int8");
        return false;
    }
    if (s_in->params.zero_point != GW_IN_ZERO_POINT) {
        Serial.printf("[MODEL] FATAL: input zp %d != expected %d\n",
                      s_in->params.zero_point, GW_IN_ZERO_POINT);
        return false;
    }
    if (fabsf(s_in->params.scale - GW_IN_SCALE) > 1e-9f) {
        Serial.println("[MODEL] FATAL: input scale mismatch vs gw_frontend_tables.h");
        return false;
    }
    if (s_in->bytes != (size_t)(GW_N_MELS * GW_N_FRAMES)) {
        Serial.printf("[MODEL] FATAL: input %u bytes, front end produces %d\n",
                      (unsigned)s_in->bytes, GW_N_MELS * GW_N_FRAMES);
        return false;
    }
    return true;
}

float gw_model_infer(const int8_t *input_int8)
{
    if (!s_interp) return -1.0f;
    memcpy(s_in->data.int8, input_int8, GW_N_MELS * GW_N_FRAMES);
    if (s_interp->Invoke() != kTfLiteOk) return -1.0f;
    int8_t raw = s_out->data.int8[0];
    return s_out->params.scale * ((float)raw - (float)s_out->params.zero_point);
}

int gw_model_arena_used(void)
{
    return s_interp ? (int)s_interp->arena_used_bytes() : -1;
}
