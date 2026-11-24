#include "jumpers.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "common.h"

static bool same(jumpers_t a, jumpers_t b) { return a.av == b.av && a.wifi == b.wifi; }

bool jumper_filter(jumper_filter_t *f, jumpers_t now, double t)
{
    if (!f->valid) {
        f->valid = true;
        f->state = f->cand = now;
        f->cand_since = t;
        return true;
    }
    if (!same(now, f->cand)) {
        f->cand = now;
        f->cand_since = t;
    }
    if (same(f->cand, f->state) || t - f->cand_since < JUMPER_DEBOUNCE_S)
        return false;
    f->state = f->cand;
    return true;
}

int jumper_open(jumper_io_t *io)
{
    io->fd = -1;
    for (int i = 0; i < 16; i++) {
        char path[32];
        snprintf(path, sizeof(path), "/dev/gpiochip%d", i);
        int fd = open(path, O_RDWR | O_CLOEXEC);
        if (fd < 0)
            continue;
        struct gpiochip_info info = {0};
        /* The header GPIOs, not the firmware's expander or anything else. */
        if (ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &info) || strncmp(info.label, "pinctrl-bcm2", 12) != 0) {
            close(fd);
            continue;
        }
        struct gpio_v2_line_request rq = {0};
        rq.offsets[0] = JUMPER_GPIO_AV;
        rq.offsets[1] = JUMPER_GPIO_WIFI;
        rq.num_lines = 2;
        rq.config.flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_BIAS_PULL_UP | GPIO_V2_LINE_FLAG_ACTIVE_LOW;
        snprintf(rq.consumer, sizeof(rq.consumer), "c5div jumpers");
        int rc = ioctl(fd, GPIO_V2_GET_LINE_IOCTL, &rq);
        int err = errno;
        close(fd);
        if (rc) {
            LOGW("jumpers: GPIO%d/%d: %s", JUMPER_GPIO_AV, JUMPER_GPIO_WIFI, strerror(err));
            return -1;
        }
        io->fd = rq.fd;
        return 0;
    }
    LOGW("jumpers: no Raspberry Pi GPIO chip found");
    return -1;
}

int jumper_read(const jumper_io_t *io, const char *sim, jumpers_t *out)
{
    if (sim) {
        char b[64] = "";
        FILE *f = fopen(sim, "r");
        if (f) {
            size_t n = fread(b, 1, sizeof(b) - 1, f);
            b[n] = 0;
            fclose(f);
        }
        out->av = strstr(b, "av") != NULL;
        out->wifi = strstr(b, "wifi") != NULL;
        return 0;
    }
    struct gpio_v2_line_values v = {.mask = 3};
    if (io->fd < 0 || ioctl(io->fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &v))
        return -1;
    out->av = v.bits & 1;       /* active low: 1 = jumper fitted */
    out->wifi = v.bits & 2;
    return 0;
}

void jumper_close(jumper_io_t *io)
{
    if (io->fd >= 0)
        close(io->fd);
    io->fd = -1;
}
