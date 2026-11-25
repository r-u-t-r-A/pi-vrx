/* Channel display: the master receiver sends its channel over a UART line,
 * on every change and every 5 s (firmware: main/link_proto.h). Read only.
 *
 *   $C5,R3,5732*14     checksum, channel and frequency must all agree
 *
 * Master D6 -> Pi GPIO15 / RXD (pin 10), 9600 baud. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define CHAN_STALE_S 12.0       /* two messages missed: the channel is not known any more */

typedef struct {
    char line[32];
    int n;
    bool in_line;
} chan_parser_t;

/* Feed one byte: 1..8 (R1..R8) when it completes a valid message, else 0. */
int chan_feed(chan_parser_t *p, uint8_t c);

typedef struct {
    const char *path;
    int fd;
    double t_retry;
    bool warned;
    chan_parser_t parser;
    int channel;                /* last one heard, 0 none */
    double t_heard;
} chanlink_t;

/* path NULL or "none": off. The port is (re)opened as needed. */
void chanlink_init(chanlink_t *c, const char *path);
/* Read what has arrived; never blocks. True when the channel changed. */
bool chanlink_poll(chanlink_t *c, double now);
/* Current channel 1..8, 0 when unknown or stale. */
int chanlink_channel(const chanlink_t *c, double now);
void chanlink_close(chanlink_t *c);
