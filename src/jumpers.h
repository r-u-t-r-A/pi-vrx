/* Output selection jumpers on the Pi's GPIO header. A fitted jumper connects
 * the GPIO to the ground pin next to it; open reads high (pull-up).
 *
 *   AV    GPIO4 (pin 7) - GND (pin 9)     fitted: composite AV, open: HDMI
 *   Wi-Fi GPIO5 (pin 29) - GND (pin 30)   fitted: web viewer on
 *
 * Both GPIOs are pulled up from power-on, so the boot firmware can read them
 * too (config.txt [gpio4=0]). Neither is used by the SMI capture.
 */
#pragma once

#include <stdbool.h>

#define JUMPER_GPIO_AV 4
#define JUMPER_GPIO_WIFI 5

typedef struct {
    bool av, wifi;
} jumpers_t;

typedef struct {
    jumpers_t state, cand;
    double cand_since;
    bool valid;
} jumper_filter_t;

#define JUMPER_DEBOUNCE_S 0.5

/* Debounce: true when the state changed (also for the first reading). */
bool jumper_filter(jumper_filter_t *f, jumpers_t now, double t);

typedef struct {
    int fd;
} jumper_io_t;

/* sim: test file holding any of "av" "wifi" instead of real GPIOs, or NULL. */
int jumper_open(jumper_io_t *io);
int jumper_read(const jumper_io_t *io, const char *sim, jumpers_t *out);
void jumper_close(jumper_io_t *io);
