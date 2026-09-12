#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#define EQ_BAND_COUNT 5
#define EQ_GAIN_MIN_DB (-6.0f)
#define EQ_GAIN_MAX_DB 6.0f
#define EQ_GAIN_STEP_DB 0.5f

typedef enum {
  EQ_PRESET_FLAT = 0,
  EQ_PRESET_BASS_BOOST,
  EQ_PRESET_TREBLE_BOOST,
  EQ_PRESET_VOCAL,
  EQ_PRESET_ACOUSTIC,
  EQ_PRESET_CUSTOM,
  EQ_PRESET_COUNT
} eq_preset_t;

esp_err_t eq_init(void);
esp_err_t eq_set_preset(eq_preset_t preset);
eq_preset_t eq_get_preset(void);
const char *eq_get_preset_name(eq_preset_t preset);
float eq_get_band_hz(size_t index);
esp_err_t eq_set_band(size_t index, float gain_db);
float eq_get_band(size_t index);
bool eq_is_custom_dirty(void);
float eq_get_preamp_db(void);
esp_err_t eq_save_custom(void);
esp_err_t eq_load_custom(void);
void eq_process_stereo(int16_t *samples, size_t frames);
