/* Raspberry Pi SMI capture through the kernel's bcm2835_smi_dev driver
 * (/dev/smi, enabled with dtoverlay=smi-dev plus a pin overlay).
 *
 * Every read() is one programmed SMI transfer, so there is a short dead
 * time between blocks. Each block is flagged DISCONT with a wall-clock
 * estimate of the gap; the decoders fill it and re-phase on the next sync.
 * (A gapless userspace DMA ring is a planned alternative backend.)
 *
 * The ioctl ABI below mirrors include/linux/broadcom/bcm2835_smi.h in the
 * Raspberry Pi kernel. `c5div probe` prints the settings read back so a
 * layout mismatch is easy to spot.
 */
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "common.h"
#include "source.h"

#define BCM2835_SMI_IOC_MAGIC 0x1
#define BCM2835_SMI_IOC_GET_SETTINGS _IO(BCM2835_SMI_IOC_MAGIC, 0)
#define BCM2835_SMI_IOC_WRITE_SETTINGS _IO(BCM2835_SMI_IOC_MAGIC, 1)
#define SMI_WIDTH_8BIT 0
#define SMI_WIDTH_16BIT 1

struct smi_settings {
    int data_width;
    bool pack_data;
    int read_setup_time;
    int read_hold_time;
    int read_pace_time;
    int read_strobe_time;
    int write_setup_time;
    int write_hold_time;
    int write_pace_time;
    int write_strobe_time;
    bool dma_enable;
    bool dma_passthrough_enable;
    int dma_read_thresh;
    int dma_write_thresh;
    int dma_panic_read_thresh;
    int dma_panic_write_thresh;
};

typedef struct {
    int fd;
    uint8_t *tmp;
} smi_priv_t;

static void log_settings(const char *what, const struct smi_settings *s)
{
    LOGI("smi %s: width=%d pack=%d read setup/strobe/hold/pace=%d/%d/%d/%d dma=%d thresh=%d panic=%d",
         what, s->data_width, s->pack_data, s->read_setup_time, s->read_strobe_time, s->read_hold_time,
         s->read_pace_time, s->dma_enable, s->dma_read_thresh, s->dma_panic_read_thresh);
}

static void *smi_thread(void *arg)
{
    source_t *s = arg;
    smi_priv_t *p = s->priv;
    const source_cfg_t *c = &s->cfg;
    const int bps = c->smi_width / 8;
    const double fs = c->fs;
    const double total = c->seconds > 0 ? c->seconds * fs : 0;

    struct sched_param sp = {.sched_priority = 50};
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        LOGW("smi: could not get SCHED_FIFO (run as root for steadier capture)");

    uint64_t virt = 0, captured = 0;
    double t_prev = 0;
    bool first = true;
    int errors = 0;
    double gap_avg = 0;
    double t_start = now_s();

    while (s->running && atomic_load(&g_running)) {
        if (total > 0 && (double)captured >= total)
            break;
        ring_block_t *b = ring_acquire(s->ring);
        const uint32_t n = s->ring->block_samples;
        void *dst = bps == 2 ? (void *)b->data : (void *)p->tmp;
        ssize_t r = read(p->fd, dst, (size_t)n * bps);
        double t = now_s();
        if (r <= 0) {
            if (r < 0 && errno == EINTR)
                continue;
            LOGE("smi read failed: %s", r < 0 ? strerror(errno) : "no data");
            if (++errors > 20)
                break;
            usleep(10000);
            continue;
        }
        errors = 0;
        uint32_t got = (uint32_t)(r / bps);
        if (got < n)
            atomic_fetch_add(&s->short_reads, 1);
        if (bps == 1)
            for (uint32_t i = 0; i < got; i++)
                b->data[i] = p->tmp[i];
        if (c->smi_swap_pairs)
            for (uint32_t i = 0; i + 1 < got; i += 2) {
                uint16_t x = b->data[i];
                b->data[i] = b->data[i + 1];
                b->data[i + 1] = x;
            }

        b->flags = 0;
        b->gap_est = 0;
        if (!first) {
            /* dt = dead time + transfer time; only the dead time is a gap. */
            double gap_s = (t - t_prev) - (double)got / fs;
            if (gap_s < 0)
                gap_s = 0;
            uint64_t g = (uint64_t)(gap_s * fs);
            virt += g;
            b->gap_est = g;
            gap_avg = gap_avg * 0.95 + gap_s * 1e6 * 0.05;
            atomic_store(&s->gap_us_avg, gap_avg);
            atomic_fetch_add(&s->gaps, 1);
        }
        b->flags = first ? 0 : RING_F_DISCONT;
        first = false;
        t_prev = t;
        b->start = virt;
        b->n = got;
        b->t_wall = t;
        virt += got;
        captured += got;
        ring_publish(s->ring, b);
        atomic_fetch_add(&s->samples, got);
        atomic_fetch_add(&s->blocks, 1);
        double el = t - t_start;
        if (el > 0.2)
            atomic_store(&s->rate_meas, (double)captured / el);
    }
    ring_set_eof(s->ring);
    return NULL;
}

static void smi_close(source_t *s)
{
    smi_priv_t *p = s->priv;
    if (p->fd >= 0)
        close(p->fd);
    free(p->tmp);
    free(p);
}

int source_smi_init(source_t *s)
{
    const source_cfg_t *c = &s->cfg;
    if (c->smi_width != 8 && c->smi_width != 16) {
        LOGE("smi width must be 8 or 16");
        return -1;
    }
    smi_priv_t *p = calloc(1, sizeof(*p));
    if (!p)
        return -1;
    p->fd = open(c->smi_dev, O_RDWR);
    if (p->fd < 0) {
        LOGE("cannot open %s: %s (is dtoverlay=smi-dev loaded? see docs/WIRING.md)", c->smi_dev,
             strerror(errno));
        free(p);
        return -1;
    }
    struct smi_settings st;
    memset(&st, 0, sizeof(st));
    if (ioctl(p->fd, BCM2835_SMI_IOC_GET_SETTINGS, &st) != 0) {
        LOGE("SMI GET_SETTINGS failed: %s", strerror(errno));
        close(p->fd);
        free(p);
        return -1;
    }
    log_settings("defaults", &st);
    st.data_width = c->smi_width == 16 ? SMI_WIDTH_16BIT : SMI_WIDTH_8BIT;
    st.pack_data = true;
    st.read_setup_time = c->smi_setup;
    st.read_strobe_time = c->smi_strobe;
    st.read_hold_time = c->smi_hold;
    st.read_pace_time = c->smi_pace;
    st.dma_enable = true;
    if (ioctl(p->fd, BCM2835_SMI_IOC_WRITE_SETTINGS, &st) != 0) {
        LOGE("SMI WRITE_SETTINGS failed: %s", strerror(errno));
        close(p->fd);
        free(p);
        return -1;
    }
    memset(&st, 0, sizeof(st));
    ioctl(p->fd, BCM2835_SMI_IOC_GET_SETTINGS, &st);
    log_settings("active", &st);
    int cycles = c->smi_setup + c->smi_strobe + c->smi_hold + c->smi_pace;
    LOGI("smi: %d SMI clock cycles per sample; at 125 MHz that is %.2f MS/s (nominal --fs %.3f MS/s)",
         cycles, 125.0 / cycles, c->fs / 1e6);
    if (c->smi_width == 8)
        p->tmp = malloc(s->cfg.block_samples);
    s->priv = p;
    s->thread_fn = smi_thread;
    s->close_fn = smi_close;
    s->realtime = true;
    s->nlanes = c->smi_width == 16 ? 2 : 1;
    return 0;
}
