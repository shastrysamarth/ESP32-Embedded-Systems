// build: gcc -O2 -o send send.c -lgpiod
// usage: ./send 4 "hello ESP32"
// wiring: GPIO17 (pin 11) -> 330Ω -> LED anode, LED cathode -> GND

#include <gpiod.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <unistd.h>
#include <math.h>

#define CHIPNAME "gpiochip0"
#define GPIO_PIN 18
// Base unit T in milliseconds; change for speed experiments from the sender side if you want.
static double T_ms = 100.0;

typedef struct { const char *ch; const char *code; } morse_t;

static const morse_t MORSE[] = {
  {"A",".-"},{"B","-..."},{"C","-.-."},{"D","-.."},{"E","."},{"F","..-."},
  {"G","--."},{"H","...."},{"I",".."},{"J",".---"},{"K","-.-"},{"L",".-.."},
  {"M","--"},{"N","-."},{"O","---"},{"P",".--."},{"Q","--.-"},{"R",".-."},
  {"S","..."},{"T","-"},{"U","..-"},{"V","...-"},{"W",".--"},{"X","-..-"},
  {"Y","-.--"},{"Z","--.."},
  {"0","-----"},{"1",".----"},{"2","..---"},{"3","...--"},{"4","....-"},
  {"5","....."},{"6","-...."},{"7","--..."},{"8","---.."},{"9","----."},
  {"/","/"}, {NULL,NULL}
};

static void nsleep(long us) {
    struct timespec ts;
    ts.tv_sec  = us / 1000000L;
    ts.tv_nsec = (us % 1000000L) * 1000L;
    nanosleep(&ts, NULL);
}

static const char* to_morse(char c) {
  if (c==' ') return "/"; // word separator
  char u = (char)toupper((unsigned char)c);
  for (int i=0; MORSE[i].ch; i++) {
    if (MORSE[i].ch[0] == u) return MORSE[i].code;
  }
  return NULL; // unknown char -> skip
}

static void led_set(struct gpiod_line *line, int val) {
  gpiod_line_set_value(line, val);
}

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "Usage: %s <repetitions> \"message\" [T_ms]\n", argv[0]);
    return 1;
  }
  int reps = atoi(argv[1]);
  const char *msg = argv[2];
  if (argc >= 4) T_ms = atof(argv[3]);

  long T_us = (long)llround(T_ms * 1000.0);

  struct gpiod_chip *chip = gpiod_chip_open_by_name(CHIPNAME);
  if (!chip) { perror("gpiod_chip_open_by_name"); return 1; }
  struct gpiod_line *line = gpiod_chip_get_line(chip, GPIO_PIN);
  if (!line) { perror("gpiod_chip_get_line"); return 1; }
  if (gpiod_line_request_output(line, "morse", 0) < 0) { perror("gpiod_line_request_output"); return 1; }

  // Timing: dot=1T on, dash=2T on, intra-symbol gap=1T off, letter gap=3T off, word gap=7T off
  for (int r=0; r<reps; r++) {
    for (size_t i=0; i<strlen(msg); i++) {
      const char *code = to_morse(msg[i]);
      if (!code) continue;

      if (strcmp(code, "/")==0) {
        // Word gap: ensure LED off then wait 7T
        led_set(line, 0); nsleep(7*T_us);
        if (i+1 < strlen(msg)) printf("/ ");
        continue;
      }

      // Emit each symbol in the letter
      for (size_t k=0; k<strlen(code); k++) {
        char sym = code[k];
        int on_ms = (sym == '.') ? (1*T_us) : (2*T_us);

        led_set(line, 1); nsleep(on_ms);
        led_set(line, 0);
        // Intra-symbol gap (only between symbols)
        if (k+1 < strlen(code)) nsleep(1*T_us);
      }
      // Letter gap (after a letter if next char isn't a space/word slash)
      // We already placed "/" explicitly for spaces, so use 3T here.
      nsleep(3*T_us);

      // Print what we're sending for debug/grade log:
      // For ambiguity, print the dot-dash with spaces between letters and "/" between words.
      printf("%s ", code);
    }
    printf("\n");
  }

  led_set(line, 0);
  gpiod_line_release(line);
  gpiod_chip_close(chip);
  return 0;
}