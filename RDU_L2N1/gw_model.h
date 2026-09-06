#ifndef GW_MODEL_H
#define GW_MODEL_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
bool  gw_model_init(void);
float gw_model_infer(const int8_t *input_int8);   /* returns probability, <0 on error */
int   gw_model_arena_used(void);
#ifdef __cplusplus
}
#endif
#endif
