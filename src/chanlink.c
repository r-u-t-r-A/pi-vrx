#include "chanlink.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "common.h"

static const int k_mhz[8] = {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917};   /* raceband */

static int check(const char *s, int n)
{
    for (int ch = 1; ch <= 8; ch++) {
        char body[24], want[32];
        const int b = snprintf(body, sizeof(body), "C5,R%d,%d", ch, k_mhz[ch - 1]);
        unsigned x = 0;
        for (int i = 0; i < b; i++)
            x ^= (unsigned char)body[i];
        const int w = snprintf(want, sizeof(want), "%s*%02X", body, x);
        if (w == n && !memcmp(want, s, (size_t)n))
            return ch;
    }
    return 0;
}

int chan_feed(chan_parser_t *p, uint8_t c)
{
    if (c == '$') {
        p->n = 0;
        p->in_line = true;
        return 0;
    }
    if (!p->in_line)
        return 0;
    if (c == '\r' || c == '\n') {
        p->in_line = false;
        return check(p->line, p->n);
    }
    if (c < 0x20 || c > 0x7e || p->n >= (int)sizeof(p->line)) {
        p->in_line = false;
        return 0;
    }
    p->line[p->n++] = (char)c;
    return 0;
}

void chanlink_init(chanlink_t *c, const char *path)
{
    memset(c, 0, sizeof(*c));
    c->fd = -1;
    c->t_retry = -1e9;
    c->t_heard = -1e9;
    c->path = path && strcmp(path, "none") ? path : NULL;
}

static void open_port(chanlink_t *c)
{
    c->fd = open(c->path, O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    struct termios t;
    if (c->fd >= 0 && tcgetattr(c->fd, &t) == 0) {
        cfmakeraw(&t);
        cfsetspeed(&t, B9600);
        t.c_cflag |= CLOCAL | CREAD;
        t.c_cflag &= ~(tcflag_t)(CSTOPB | CRTSCTS);
        if (tcsetattr(c->fd, TCSANOW, &t) == 0) {
            tcflush(c->fd, TCIFLUSH);
            c->warned = false;
            memset(&c->parser, 0, sizeof(c->parser));
            LOGI("channel link: listening on %s", c->path);
            return;
        }
    }
    if (!c->warned)
        LOGW("channel link: %s: %s (channel not shown)", c->path, strerror(errno));
    c->warned = true;
    if (c->fd >= 0)
        close(c->fd);
    c->fd = -1;
}

bool chanlink_poll(chanlink_t *c, double now)
{
    if (!c->path)
        return false;
    if (c->fd < 0) {
        if (now - c->t_retry < 5.0)
            return false;
        c->t_retry = now;
        open_port(c);
        if (c->fd < 0)
            return false;
    }
    const int before = c->channel;
    uint8_t buf[256];
    ssize_t r;
    while ((r = read(c->fd, buf, sizeof(buf))) > 0)
        for (ssize_t i = 0; i < r; i++) {
            const int ch = chan_feed(&c->parser, buf[i]);
            if (ch) {
                c->channel = ch;
                c->t_heard = now;
            }
        }
    if (r < 0 && errno != EAGAIN && errno != EINTR) {   /* unplugged adapter, hangup */
        LOGW("channel link: %s: %s", c->path, strerror(errno));
        chanlink_close(c);
        c->t_retry = now;
    }
    return c->channel != before;
}

int chanlink_channel(const chanlink_t *c, double now)
{
    return now - c->t_heard < CHAN_STALE_S ? c->channel : 0;
}

void chanlink_close(chanlink_t *c)
{
    if (c->fd >= 0)
        close(c->fd);
    c->fd = -1;
}
