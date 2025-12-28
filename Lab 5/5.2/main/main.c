// Fastest this can handle is 50ms per symbol (. or -)

#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define TAG          "MORSE"

#define GREEN  "\033[1;32m"
#define RESET  "\033[0m"

// ----------------- CONFIG (match your sender) -----------------
#define ADC_CH         ADC_CHANNEL_0      // ESP32-C3 GPIO0
#define ADC_ATTEN      ADC_ATTEN_DB_12
#define SAMPLE_MS      50                 // receiver sampling period
#define GREEN_THRESH   200                // >200 = "green", else "white"
#define BASE_DOT_MS    50                 // MUST match sender T_ms

// ----------------- Derived (strict, no auto-learning) --------
#define T_PRINTS       ((BASE_DOT_MS + (SAMPLE_MS/2)) / SAMPLE_MS)  // rounded prints per dot
#if T_PRINTS < 1
  #undef T_PRINTS
  #define T_PRINTS 1
#endif

// Symbols (exact multiples of T)
#define DOT_MIN_G      (1 * T_PRINTS)
#define DOT_MAX_G      (1 * T_PRINTS)
#define DASH_MIN_G     (2 * T_PRINTS)
#define DASH_MAX_G     (2 * T_PRINTS)

// Gaps (whites)
#define SYMBOL_GAP_MIN_W  (1 * T_PRINTS)
#define SYMBOL_GAP_MAX_W  (1 * T_PRINTS)   // exactly 1T
#define LETTER_GAP_MIN_W  (3 * T_PRINTS)
#define LETTER_GAP_MAX_W  (3 * T_PRINTS)   // exactly 3T
#define WORD_GAP_MIN_W    (7 * T_PRINTS)
#define WORD_GAP_MAX_W    (20 * T_PRINTS)  // 7..20T
#define FLUSH_GAP_MIN_W   (21 * T_PRINTS)  // >=21T

// ----------------- Morse map -----------------
typedef struct { const char *p; char ch; } morse_t;
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
  for (int i=0; MORSE[i].p; ++i) if (strcmp(MORSE[i].p, pat)==0) return MORSE[i].ch;
  return '?';
}

static inline void append_sym(char *pattern, size_t max, char s){
  size_t n = strlen(pattern);
  if (n + 1 < max) { pattern[n] = s; pattern[n+1] = '\0'; }
}

static inline void close_letter(char *pattern, char *message, size_t max_msg){
  if (!pattern[0]) return;
  char ch = decode_pattern(pattern);
  size_t m = strlen(message);
  if (m + 1 < max_msg) { message[m++] = ch; message[m] = '\0'; }
  pattern[0] = '\0';
}

void app_main(void){
  // ADC init
  adc_oneshot_unit_handle_t adc;
  adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = ADC_UNIT_1};
  adc_oneshot_new_unit(&unit_cfg, &adc);
  adc_oneshot_chan_cfg_t ch_cfg = {.bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN};
  adc_oneshot_config_channel(adc, ADC_CH, &ch_cfg);

  int val = 0;
  bool is_green = false, was_green = false;
  int green_run = 0, white_run = 0;

  char pattern[16]  = {0};   // current letter ".-"
  char message[512] = {0};   // full message
  bool flushed_once = false;

  ESP_LOGI(TAG, "SAMPLE_MS=%d, BASE_DOT_MS=%d, T_PRINTS=%d",
           SAMPLE_MS, BASE_DOT_MS, T_PRINTS);

  while (1) {
    adc_oneshot_read(adc, ADC_CH, &val);
    is_green = (val > GREEN_THRESH);

    // ---- EDGE FIRST: classify with completed run, then reset ----
    if (was_green && !is_green) {
      // classify symbol by strict counts
      if (green_run >= DOT_MIN_G && green_run <= DOT_MAX_G) {
        append_sym(pattern, sizeof(pattern), '.');
        printf("."); fflush(stdout);
      } else if (green_run >= DASH_MIN_G && green_run <= DASH_MAX_G) {
        append_sym(pattern, sizeof(pattern), '-');
        printf("-"); fflush(stdout);
      }
      // start first white count
      green_run = 0;
      white_run = 1;
    }
    else if (!was_green && is_green) {
      // apply strict gap rules with the completed white_run
      if (white_run >= SYMBOL_GAP_MIN_W && white_run <= SYMBOL_GAP_MAX_W) {
        // symbol gap: same letter
      } else if (white_run >= LETTER_GAP_MIN_W && white_run <= LETTER_GAP_MAX_W) {
        close_letter(pattern, message, sizeof(message));
      } else if (white_run >= WORD_GAP_MIN_W && white_run <= WORD_GAP_MAX_W) {
        close_letter(pattern, message, sizeof(message));
        size_t m = strlen(message);
        if (m == 0 || message[m-1] != ' ') {
          if (m + 1 < sizeof(message)) { message[m++] = ' '; message[m] = '\0'; }
        }
      }
      // start first green count
      white_run = 0;
      green_run = 1;
      flushed_once = false;
    }
    else {
      // no edge: extend current run and allow flush while idle white
      if (is_green) {
        green_run++;
      } else {
        white_run++;
        if (white_run >= FLUSH_GAP_MIN_W && !flushed_once) {
          close_letter(pattern, message, sizeof(message));
          ESP_LOGI(TAG, "MESSAGE: %s", message[0] ? message : "(empty)");
          printf("\n"); fflush(stdout);
          // reset for next message
          message[0] = '\0';
          pattern[0] = '\0';
          flushed_once = true;
        }
      }
    }

    was_green = is_green;
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
  }
}