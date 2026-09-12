#include "eq.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "nvs.h"

#define EQ_SAMPLE_RATE_HZ 44100.0f
#define EQ_Q              0.70710678f
#define NVS_NAMESPACE     "airplay"
#define NVS_KEY_CUSTOM    "eq5_custom"
#define NVS_KEY_MODE      "eq5_mode"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef enum {
  EQ_FILTER_LOW_SHELF = 0,
  EQ_FILTER_PEAK,
  EQ_FILTER_HIGH_SHELF,
} eq_filter_type_t;

typedef struct {
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
  float z1_l;
  float z2_l;
  float z1_r;
  float z2_r;
} eq_biquad_t;

typedef struct {
  eq_preset_t preset;
  float custom[EQ_BAND_COUNT];
  float active[EQ_BAND_COUNT];
  float preamp_db;
  float preamp_linear;
  eq_biquad_t filters[EQ_BAND_COUNT];
  bool initialized;
  bool bypass;
  bool custom_dirty;
} eq_state_t;

static const char *TAG = "eq";

static const float s_band_hz[EQ_BAND_COUNT] = {80.0f, 250.0f, 1000.0f,
                                               4000.0f, 12000.0f};

static const eq_filter_type_t s_filter_type[EQ_BAND_COUNT] = {
    EQ_FILTER_LOW_SHELF, EQ_FILTER_PEAK, EQ_FILTER_PEAK, EQ_FILTER_PEAK,
    EQ_FILTER_HIGH_SHELF};

static const float s_factory_presets[EQ_PRESET_CUSTOM][EQ_BAND_COUNT] = {
    [EQ_PRESET_FLAT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    [EQ_PRESET_BASS_BOOST] = {4.0f, 2.0f, 0.0f, -1.0f, 0.0f},
    [EQ_PRESET_TREBLE_BOOST] = {0.0f, -1.0f, 0.0f, 2.0f, 4.0f},
    [EQ_PRESET_VOCAL] = {-1.0f, 0.0f, 3.0f, 2.0f, 0.0f},
    [EQ_PRESET_ACOUSTIC] = {1.0f, 1.0f, 2.0f, 1.0f, 2.0f},
};

static const char *const s_preset_names[EQ_PRESET_COUNT] = {
    [EQ_PRESET_FLAT] = "Flat",
    [EQ_PRESET_BASS_BOOST] = "Bass Boost",
    [EQ_PRESET_TREBLE_BOOST] = "Treble Boost",
    [EQ_PRESET_VOCAL] = "Vocal",
    [EQ_PRESET_ACOUSTIC] = "Acoustic",
    [EQ_PRESET_CUSTOM] = "Custom",
};

static eq_state_t s_eq;

static float clamp_gain(float gain_db) {
  if (gain_db < EQ_GAIN_MIN_DB) {
    return EQ_GAIN_MIN_DB;
  }
  if (gain_db > EQ_GAIN_MAX_DB) {
    return EQ_GAIN_MAX_DB;
  }
  return gain_db;
}

static int16_t clamp_i16(float sample) {
  if (sample > 32767.0f) {
    return 32767;
  }
  if (sample < -32768.0f) {
    return -32768;
  }
  return (int16_t)lrintf(sample);
}

static void reset_filter_state(eq_biquad_t *bq) {
  bq->z1_l = 0.0f;
  bq->z2_l = 0.0f;
  bq->z1_r = 0.0f;
  bq->z2_r = 0.0f;
}

static void set_coeffs(eq_biquad_t *bq, float b0, float b1, float b2,
                       float a0, float a1, float a2) {
  if (a0 == 0.0f) {
    bq->b0 = 1.0f;
    bq->b1 = 0.0f;
    bq->b2 = 0.0f;
    bq->a1 = 0.0f;
    bq->a2 = 0.0f;
    return;
  }

  bq->b0 = b0 / a0;
  bq->b1 = b1 / a0;
  bq->b2 = b2 / a0;
  bq->a1 = a1 / a0;
  bq->a2 = a2 / a0;
}

static void configure_peak(eq_biquad_t *bq, float freq_hz, float gain_db) {
  float a = powf(10.0f, gain_db / 40.0f);
  float w0 = 2.0f * (float)M_PI * freq_hz / EQ_SAMPLE_RATE_HZ;
  float sn = sinf(w0);
  float cs = cosf(w0);
  float alpha = sn / (2.0f * EQ_Q);

  set_coeffs(bq, 1.0f + alpha * a, -2.0f * cs, 1.0f - alpha * a,
             1.0f + alpha / a, -2.0f * cs, 1.0f - alpha / a);
}

static void configure_shelf(eq_biquad_t *bq, float freq_hz, float gain_db,
                            bool high_shelf) {
  float a = powf(10.0f, gain_db / 40.0f);
  float w0 = 2.0f * (float)M_PI * freq_hz / EQ_SAMPLE_RATE_HZ;
  float sn = sinf(w0);
  float cs = cosf(w0);
  float sqrt_a = sqrtf(a);
  float alpha = sn * 0.5f * sqrtf(2.0f);
  float beta = 2.0f * sqrt_a * alpha;

  if (high_shelf) {
    set_coeffs(bq, a * ((a + 1.0f) + (a - 1.0f) * cs + beta),
               -2.0f * a * ((a - 1.0f) + (a + 1.0f) * cs),
               a * ((a + 1.0f) + (a - 1.0f) * cs - beta),
               (a + 1.0f) - (a - 1.0f) * cs + beta,
               2.0f * ((a - 1.0f) - (a + 1.0f) * cs),
               (a + 1.0f) - (a - 1.0f) * cs - beta);
  } else {
    set_coeffs(bq, a * ((a + 1.0f) - (a - 1.0f) * cs + beta),
               2.0f * a * ((a - 1.0f) - (a + 1.0f) * cs),
               a * ((a + 1.0f) - (a - 1.0f) * cs - beta),
               (a + 1.0f) + (a - 1.0f) * cs + beta,
               -2.0f * ((a - 1.0f) + (a + 1.0f) * cs),
               (a + 1.0f) + (a - 1.0f) * cs - beta);
  }
}

static void rebuild_filters(void) {
  float max_boost = 0.0f;
  bool flat = true;

  for (size_t i = 0; i < EQ_BAND_COUNT; i++) {
    if (s_eq.active[i] > max_boost) {
      max_boost = s_eq.active[i];
    }
    if (fabsf(s_eq.active[i]) > 0.001f) {
      flat = false;
    }
  }

  s_eq.preamp_db = -max_boost;
  s_eq.preamp_linear = powf(10.0f, s_eq.preamp_db / 20.0f);
  s_eq.bypass = flat && max_boost == 0.0f;

  for (size_t i = 0; i < EQ_BAND_COUNT; i++) {
    switch (s_filter_type[i]) {
    case EQ_FILTER_LOW_SHELF:
      configure_shelf(&s_eq.filters[i], s_band_hz[i], s_eq.active[i], false);
      break;
    case EQ_FILTER_HIGH_SHELF:
      configure_shelf(&s_eq.filters[i], s_band_hz[i], s_eq.active[i], true);
      break;
    default:
      configure_peak(&s_eq.filters[i], s_band_hz[i], s_eq.active[i]);
      break;
    }
    reset_filter_state(&s_eq.filters[i]);
  }

  ESP_LOGI(TAG, "EQ preamp: %.1f dB", s_eq.preamp_db);
}

static void apply_preset_values(eq_preset_t preset) {
  if (preset == EQ_PRESET_CUSTOM) {
    memcpy(s_eq.active, s_eq.custom, sizeof(s_eq.active));
  } else {
    memcpy(s_eq.active, s_factory_presets[preset], sizeof(s_eq.active));
  }
  rebuild_filters();
}

static esp_err_t persist_mode(void) {
  nvs_handle_t nvs;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (err != ESP_OK) {
    return err;
  }
  err = nvs_set_u8(nvs, NVS_KEY_MODE, (uint8_t)s_eq.preset);
  if (err == ESP_OK) {
    err = nvs_commit(nvs);
  }
  nvs_close(nvs);
  return err;
}

esp_err_t eq_load_custom(void) {
  nvs_handle_t nvs;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
  if (err != ESP_OK) {
    memset(s_eq.custom, 0, sizeof(s_eq.custom));
    return err;
  }

  size_t len = sizeof(s_eq.custom);
  err = nvs_get_blob(nvs, NVS_KEY_CUSTOM, s_eq.custom, &len);
  nvs_close(nvs);
  if (err != ESP_OK || len != sizeof(s_eq.custom)) {
    memset(s_eq.custom, 0, sizeof(s_eq.custom));
    return err == ESP_OK ? ESP_ERR_INVALID_SIZE : err;
  }

  for (size_t i = 0; i < EQ_BAND_COUNT; i++) {
    s_eq.custom[i] = clamp_gain(s_eq.custom[i]);
  }
  return ESP_OK;
}

esp_err_t eq_init(void) {
  memset(&s_eq, 0, sizeof(s_eq));
  s_eq.preset = EQ_PRESET_FLAT;
  s_eq.preamp_linear = 1.0f;
  (void)eq_load_custom();

  nvs_handle_t nvs;
  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
    uint8_t mode = EQ_PRESET_FLAT;
    if (nvs_get_u8(nvs, NVS_KEY_MODE, &mode) == ESP_OK &&
        mode < EQ_PRESET_COUNT) {
      s_eq.preset = (eq_preset_t)mode;
    }
    nvs_close(nvs);
  }

  apply_preset_values(s_eq.preset);
  s_eq.initialized = true;
  ESP_LOGI(TAG, "EQ preset: %s", s_preset_names[s_eq.preset]);
  return ESP_OK;
}

esp_err_t eq_set_preset(eq_preset_t preset) {
  if (preset >= EQ_PRESET_COUNT) {
    return ESP_ERR_INVALID_ARG;
  }

  s_eq.preset = preset;
  if (preset != EQ_PRESET_CUSTOM) {
    s_eq.custom_dirty = false;
  }
  apply_preset_values(preset);
  ESP_LOGI(TAG, "EQ preset: %s", s_preset_names[preset]);
  return persist_mode();
}

eq_preset_t eq_get_preset(void) {
  return s_eq.preset;
}

const char *eq_get_preset_name(eq_preset_t preset) {
  if (preset >= EQ_PRESET_COUNT) {
    return "Unknown";
  }
  return s_preset_names[preset];
}

float eq_get_band_hz(size_t index) {
  if (index >= EQ_BAND_COUNT) {
    return 0.0f;
  }
  return s_band_hz[index];
}

esp_err_t eq_set_band(size_t index, float gain_db) {
  if (index >= EQ_BAND_COUNT) {
    return ESP_ERR_INVALID_ARG;
  }

  if (s_eq.preset != EQ_PRESET_CUSTOM) {
    memcpy(s_eq.custom, s_eq.active, sizeof(s_eq.custom));
    s_eq.preset = EQ_PRESET_CUSTOM;
    (void)persist_mode();
    ESP_LOGI(TAG, "EQ preset: Custom");
  }

  s_eq.custom[index] = clamp_gain(gain_db);
  s_eq.active[index] = s_eq.custom[index];
  s_eq.custom_dirty = true;
  rebuild_filters();
  ESP_LOGI(TAG, "EQ custom band %u: %.1f dB", (unsigned)index,
           s_eq.custom[index]);
  return ESP_OK;
}

float eq_get_band(size_t index) {
  if (index >= EQ_BAND_COUNT) {
    return 0.0f;
  }
  return s_eq.active[index];
}

bool eq_is_custom_dirty(void) {
  return s_eq.custom_dirty;
}

float eq_get_preamp_db(void) {
  return s_eq.preamp_db;
}

esp_err_t eq_save_custom(void) {
  nvs_handle_t nvs;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "EQ custom save failed: %s", esp_err_to_name(err));
    return err;
  }

  err = nvs_set_blob(nvs, NVS_KEY_CUSTOM, s_eq.custom, sizeof(s_eq.custom));
  if (err == ESP_OK) {
    err = nvs_commit(nvs);
  }
  nvs_close(nvs);

  if (err == ESP_OK) {
    s_eq.custom_dirty = false;
    ESP_LOGI(TAG, "EQ custom saved");
  } else {
    ESP_LOGE(TAG, "EQ custom save failed: %s", esp_err_to_name(err));
  }
  return err;
}

void eq_process_stereo(int16_t *samples, size_t frames) {
  if (!samples || frames == 0 || !s_eq.initialized || s_eq.bypass) {
    return;
  }

  for (size_t i = 0; i < frames; i++) {
    float l = (float)samples[i * 2] * s_eq.preamp_linear;
    float r = (float)samples[i * 2 + 1] * s_eq.preamp_linear;

    for (size_t band = 0; band < EQ_BAND_COUNT; band++) {
      eq_biquad_t *bq = &s_eq.filters[band];
      float out_l = bq->b0 * l + bq->z1_l;
      bq->z1_l = bq->b1 * l - bq->a1 * out_l + bq->z2_l;
      bq->z2_l = bq->b2 * l - bq->a2 * out_l;
      l = out_l;

      float out_r = bq->b0 * r + bq->z1_r;
      bq->z1_r = bq->b1 * r - bq->a1 * out_r + bq->z2_r;
      bq->z2_r = bq->b2 * r - bq->a2 * out_r;
      r = out_r;
    }

    samples[i * 2] = clamp_i16(l);
    samples[i * 2 + 1] = clamp_i16(r);
  }
}
