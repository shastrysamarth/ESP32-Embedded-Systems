// Fastest receiving so far:
// String: UCSC CSE 121 ABCDEFGHIJKLM NOPQRSTUVWXYZ 12345 67890
// 25ms  52/52 passed
// 20ms  52/52 passed
// 15ms  52/52 passed
// 10ms  52/52 passed
// 5ms   52/52 passed
// 1ms   52/52 passed
// 500us 52/52 passed
// 200us 16/52 passed

// ESP32-C3 Morse IR decoder (ADC Continuous DMA, jitter-hardened)
// Features: hysteresis, glitch suppression, midpoint classification, sync gating, message timing/CPS

#include "esp_adc/adc_continuous.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define TAG "MORSE_DMA"

// ======== CONFIG: wiring & timing ========
#define ADC_UNIT            ADC_UNIT_1
#define ADC_CH              ADC_CHANNEL_0          // GPIO0 (ESP32-C3)
#define ADC_ATTEN_LEVEL     ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH       ADC_BITWIDTH_12

// Sampling rate: increase for sub-ms dots.
// 10 kHz => 100 µs/sample; 20 kHz => 50 µs/sample; 50 kHz => 20 µs/sample.
#define SAMPLE_FREQ_HZ      20000                  // 20 kHz is a solid default

// DMA buffers
#define FRAME_SIZE_BYTES    1024
#define RINGBUF_BYTES       (8*1024)

// Baseline threshold and hysteresis (tune if needed)
#define GREEN_THRESH        200
#define HYST_COUNTS         15                     // ON/OFF hysteresis margin (ADC counts)
#define TH_ON               (GREEN_THRESH + HYST_COUNTS)
#define TH_OFF              (GREEN_THRESH - HYST_COUNTS)

// Required message to time (extra credit)
static const char *EXPECTED =
  "UCSC CSE 121 ABCDEFGHIJKLM NOPQRSTUVWXYZ 12345 67890";

// ======== Dot duration (in MICROSECONDS) ========
// Set this to your sender T_ms * 1000 (e.g., 500 for 0.5 ms; 15000 for 15 ms).
#define BASE_DOT_US         500                    // example: 0.5 ms dot

// ======== Time thresholds (us) ========
#define DOT_US              ((int64_t)BASE_DOT_US)

// Symbol decision by midpoints (robust)
#define DOT_DASH_SPLIT_US   ((int64_t)(1.5 * (double)DOT_US))  // < => dot, >= => dash
#define DASH_MAX_US         ((int64_t)(3.5 * (double)DOT_US))  // sanity upper bound

// Gap decisions by midpoints
#define SYMBOL_LETTER_SPLIT_US  ((int64_t)(2.0 * (double)DOT_US)) // <2T => symbol gap
#define LETTER_WORD_SPLIT_US    ((int64_t)(5.5 * (double)DOT_US)) // 2T..5.5T => letter; >=5.5T => word
#define WORD_GAP_MIN_US         ((int64_t)(6.0 * (double)DOT_US)) // sync/word threshold

// Glitch suppression: ignore any ON or OFF blip shorter than this
#define GLITCH_MAX_US       ((int64_t)(0.6 * (double)DOT_US))  // 0.6T is a good default

// Flush entire message after long idle
#define FLUSH_GAP_US        ((int64_t)(21.0 * (double)DOT_US))

// ======== Morse map ========
typedef struct { const char *pattern; char ch; } morse_t;
static const morse_t MORSE[] = {
  {".-",'A'},{"-...",'B'},{"-.-.",'C'},{"-..",'D'},{".",'E'},
  {"..-.",'F'},{"--.",'G'},{"....",'H'},{"..",'I'},{".---",'J'},
  {"-.-",'K'},{".-..",'L'},{"--",'M'},{"-.",'N'},{"---",'O'},
  {".--.",'P'},{"--.-",'Q'},{".-.",'R'},{"...",'S'},{"-",'T'},
  {"..-",'U'},{"...-",'V'},{".--",'W'},{"-..-",'X'},{"-.--",'Y'},
  {"--..",'Z'},{"-----",'0'},{".----",'1'},{"..---",'2'},{"...--",'3'},
  {"....-",'4'},{".....",'5'},{"-....",'6'},{"--...",'7'},{"---..",'8'},{"----.",'9'},
  {NULL,0}
};

static char decode_pattern(const char *pat) {
  for (int i=0; MORSE[i].pattern; ++i)
    if (strcmp(MORSE[i].pattern, pat) == 0) return MORSE[i].ch;
  return '?';
}

static inline void append_sym(char *pattern, size_t max, char s) {
  size_t n = strlen(pattern);
  if (n + 1 < max) { pattern[n] = s; pattern[n+1] = '\0'; }
}
static inline void close_letter(char *pattern, char *message, size_t max_msg) {
  if (!pattern[0]) return;
  char ch = decode_pattern(pattern);
  size_t m = strlen(message);
  if (m + 1 < max_msg) { message[m++] = ch; message[m] = '\0'; }
  pattern[0] = '\0';
}

static inline int64_t samples_to_us(int samples) {
  return (int64_t)((1000000LL * (int64_t)samples) / (int64_t)SAMPLE_FREQ_HZ);
}

// ======== ADC Continuous setup ========
static adc_continuous_handle_t adc_setup(void) {
  adc_continuous_handle_t handle = NULL;

  adc_continuous_handle_cfg_t handle_cfg = {
    .max_store_buf_size = RINGBUF_BYTES,
    .conv_frame_size    = FRAME_SIZE_BYTES,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &handle));

  adc_digi_pattern_config_t pattern = {
    .atten     = ADC_ATTEN_LEVEL,
    .channel   = ADC_CH,
    .bit_width = ADC_BIT_WIDTH
  };

  adc_continuous_config_t cfg = {
    .sample_freq_hz = SAMPLE_FREQ_HZ,
    .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
    .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    .pattern_num    = 1,
    .adc_pattern    = &pattern
  };

  ESP_ERROR_CHECK(adc_continuous_config(handle, &cfg));
  ESP_ERROR_CHECK(adc_continuous_start(handle));
  ESP_LOGI(TAG, "ADC continuous: %d Hz, frame=%dB, ring=%dB, DOT=%lld us",
           SAMPLE_FREQ_HZ, FRAME_SIZE_BYTES, RINGBUF_BYTES, (long long)DOT_US);
  return handle;
}

void app_main(void) {
  // Decoder state
  char pattern[16]  = {0};
  char message[512] = {0};

  bool is_on        = false;     // current state (after hysteresis)
  int  on_samples   = 0;
  int  off_samples  = 0;

  // Sync / timing (extra credit)
  bool synced  = false;          // true after we see ≥ ~7T idle
  bool started = false;          // true once we start timing/decoding
  int64_t t_start_us = 0;
  int64_t t_stop_us  = 0;

  bool flushed_once = false;

  adc_continuous_handle_t h = adc_setup();

  uint8_t  frame[FRAME_SIZE_BYTES];
  uint32_t bytes = 0;

  while (1) {
    esp_err_t ret = adc_continuous_read(h, frame, sizeof(frame), &bytes, 20 /*ms timeout*/);
    if (ret == ESP_OK && bytes >= sizeof(adc_digi_output_data_t)) {
      for (uint32_t i = 0; i < bytes; i += sizeof(adc_digi_output_data_t)) {
        const adc_digi_output_data_t *d = (const adc_digi_output_data_t *)&frame[i];
        if (d->type2.channel != ADC_CH) continue;

        int sample = d->type2.data;

        // Hysteretic thresholding
        bool now_on = is_on ? (sample > TH_OFF) : (sample > TH_ON);

        // ----- Transition handling with glitch suppression -----
        if (now_on != is_on) {
          // We are about to switch state; compute the run length of the state that is ending
          if (is_on && !now_on) {
            // ON is ending -> candidate ON run
            int64_t on_us = samples_to_us(on_samples);
            if (on_us < GLITCH_MAX_US) {
              // Glitch: ignore this false OFF; keep staying ON
              // (do not flip state; just keep accumulating ON)
              // absorb this "would-be" OFF sample as ON
              on_samples++;
              continue;
            }

            // Valid ON -> OFF
            if (synced && started) {
              // Midpoint classification: dot vs dash
              if (on_us < DOT_DASH_SPLIT_US) {
                append_sym(pattern, sizeof(pattern), '.');
              } else if (on_us <= DASH_MAX_US) {
                append_sym(pattern, sizeof(pattern), '-');
              } // else: too long, ignore (sanity guard)
            }

            // start OFF run
            is_on = false;
            off_samples = 1;
            on_samples  = 0;
            flushed_once = false;

          } else if (!is_on && now_on) {
            // OFF is ending -> candidate OFF run
            int64_t off_us = samples_to_us(off_samples);
            if (off_us < GLITCH_MAX_US) {
              // Glitch: ignore this false ON; keep staying OFF
              off_samples++;
              continue;
            }

            // Valid OFF -> ON
            if (!synced) {
              if (off_us >= WORD_GAP_MIN_US) {
                // Achieved clean idle => sync arms
                synced = true;
                // clear any pre-roll
                pattern[0] = '\0';
                message[0] = '\0';
                on_samples = off_samples = 0;
              }
            }

            if (synced && !started) {
              started = true;
              t_start_us = esp_timer_get_time();
              // do not classify this initial gap; just start cleanly now
            } else if (synced && started) {
              // Gap classification by midpoints
              if (off_us < SYMBOL_LETTER_SPLIT_US) {
                // intra-symbol gap -> same letter
              } else if (off_us < LETTER_WORD_SPLIT_US) {
                // letter gap
                close_letter(pattern, message, sizeof(message));
              } else {
                // word gap
                close_letter(pattern, message, sizeof(message));
                size_t m = strlen(message);
                if (m == 0 || message[m-1] != ' ') {
                  if (m + 1 < sizeof(message)) { message[m++] = ' '; message[m] = '\0'; }
                }
              }
            }

            // start ON run
            is_on = true;
            on_samples  = 1;
            off_samples = 0;
          }
        } else {
          // No transition: extend current run
          if (now_on) {
            on_samples++;
          } else {
            off_samples++;

            // Idle flush only if we already started a run
            if (synced && started && samples_to_us(off_samples) >= FLUSH_GAP_US && !flushed_once) {
              close_letter(pattern, message, sizeof(message));
              t_stop_us = esp_timer_get_time();

              // Print final message & cps
              if (message[0]) {
                int64_t elapsed_us = t_stop_us - t_start_us;
                double elapsed_s = (double)elapsed_us / 1e6;
                size_t exp_len = strlen(EXPECTED);
                bool matches = (strcmp(message, EXPECTED) == 0);
                double cps = elapsed_s > 0.0 ? (double)exp_len / elapsed_s : 0.0;

                ESP_LOGI(TAG, "MESSAGE: %s", message);
                printf("Received in %.6f s | cps=%.2f | match=%s\n\n",
                       elapsed_s, cps, matches ? "YES" : "NO");
              } else {
                ESP_LOGI(TAG, "MESSAGE: (empty)\n");
              }

              // Reset for next try; require fresh sync (long idle) again
              message[0] = '\0'; pattern[0] = '\0';
              on_samples = off_samples = 0;
              synced = false; started = false;
              t_start_us = 0; t_stop_us = 0;
              flushed_once = true;
            }
          }
        }
      }
    } else if (ret == ESP_ERR_TIMEOUT) {
      vTaskDelay(pdMS_TO_TICKS(1));
    } else {
      ESP_LOGW(TAG, "adc_continuous_read err=%d", ret);
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
}