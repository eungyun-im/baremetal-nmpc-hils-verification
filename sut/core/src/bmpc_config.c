#include "bmpc/bmpc_config.h"

#include <string.h>

#include "bmpc/bmpc_math.h"

void bmpc_config_default(bmpc_config_t *cfg)
{
#define BMPC_CFG_DEFAULT(idx, name, type, def) cfg->name = (BMPC_CFG_CTYPE_##type)(def);
  BMPC_CONFIG_FIELDS(BMPC_CFG_DEFAULT)
#undef BMPC_CFG_DEFAULT
}

/* DEF-6: float -> int32 is undefined behaviour (C99 6.3.1.4) when the value is NaN or outside the int32 range.
 * Such a value is rejected and the field keeps its old value; the commit hash then differs and the MCU stays
 * NOT_CONFIGURED. 2147483648.0f is exactly 2^31, the first float that no longer fits. */
static int cfg_store_I(int32_t *dst, float value)
{
  if (!((value >= -2147483648.0f) && (value < 2147483648.0f))) {
    return -1;
  }
  *dst = (int32_t)value;
  return 0;
}

static int cfg_store_R(bmpc_real *dst, float value)
{
  *dst = (bmpc_real)value;
  return 0;
}

int bmpc_config_set(bmpc_config_t *cfg, uint16_t idx, float value)
{
  switch (idx) {
#define BMPC_CFG_SET_CASE(i, name, type, def)                                              \
  case i:                                                                                  \
    return cfg_store_##type(&cfg->name, value);
    BMPC_CONFIG_FIELDS(BMPC_CFG_SET_CASE)
#undef BMPC_CFG_SET_CASE
    default:
      return -1;
  }
}

float bmpc_config_get(const bmpc_config_t *cfg, uint16_t idx)
{
  switch (idx) {
#define BMPC_CFG_GET_CASE(i, name, type, def)                                              \
  case i:                                                                                  \
    return (float)cfg->name;
    BMPC_CONFIG_FIELDS(BMPC_CFG_GET_CASE)
#undef BMPC_CFG_GET_CASE
    default:
      return 0.0f;
  }
}

const char *bmpc_config_name(uint16_t idx)
{
  switch (idx) {
#define BMPC_CFG_NAME_CASE(i, name, type, def)                                             \
  case i:                                                                                  \
    return #name;
    BMPC_CONFIG_FIELDS(BMPC_CFG_NAME_CASE)
#undef BMPC_CFG_NAME_CASE
    default:
      return "?";
  }
}

static uint32_t fnv1a_byte(uint32_t h, uint8_t b) { return (h ^ b) * 16777619u; }

uint32_t bmpc_config_hash(const bmpc_config_t *cfg)
{
  uint32_t h = 2166136261u;
  uint16_t i;
  for (i = 0; i < (uint16_t)BMPC_CONFIG_COUNT; ++i) {
    const float v = bmpc_config_get(cfg, i);
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    h = fnv1a_byte(h, (uint8_t)(i & 0xFFu));
    h = fnv1a_byte(h, (uint8_t)(i >> 8));
    h = fnv1a_byte(h, (uint8_t)(bits & 0xFFu));
    h = fnv1a_byte(h, (uint8_t)((bits >> 8) & 0xFFu));
    h = fnv1a_byte(h, (uint8_t)((bits >> 16) & 0xFFu));
    h = fnv1a_byte(h, (uint8_t)(bits >> 24));
  }
  return h;
}

void bmpc_config_sanitize(bmpc_config_t *cfg)
{
  if (cfg->N < 3) cfg->N = 3;
  if (cfg->N > BMPC_N_MAX) cfg->N = BMPC_N_MAX;
  /* mpc.cpp Solve(): steer_scale, steer_change_scale 은 0.05..5 */
  cfg->steer_scale = bm_clamp(cfg->steer_scale, (bmpc_real)0.05, (bmpc_real)5.0);
  cfg->steer_change_scale = bm_clamp(cfg->steer_change_scale, (bmpc_real)0.05, (bmpc_real)5.0);
  /* mpc_path_tracking.h initSetup() */
  cfg->st_lpf_alpha = bm_clamp(cfg->st_lpf_alpha, (bmpc_real)0.0, (bmpc_real)1.0);
  cfg->st_lpf_alpha_high = bm_clamp(cfg->st_lpf_alpha_high, (bmpc_real)0.0, (bmpc_real)1.0);
  cfg->lg_ff_taper = bm_max((bmpc_real)0.1, cfg->lg_ff_taper);
  cfg->lg_ff_steer_zero = bm_max((bmpc_real)0.01, cfg->lg_ff_steer_zero);
  cfg->lg_ff_accel_per_pedal = bm_max((bmpc_real)0.5, cfg->lg_ff_accel_per_pedal);
  cfg->lg_ff_tau = bm_max((bmpc_real)0.0, cfg->lg_ff_tau);
  cfg->dc_transport_req_share = bm_clamp(cfg->dc_transport_req_share, (bmpc_real)0.0, (bmpc_real)1.0);
  if (cfg->sqp_iters < 1) cfg->sqp_iters = 1;
  if (cfg->qp_max_iters < 1) cfg->qp_max_iters = 1;
}
