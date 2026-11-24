/* Gapless SMI capture: SMI read cycles paced by the SMI DREQ into a circular
 * DMA chain, entirely from user space (Raspberry Pi Zero 2W / Pi 3, BCM2837).
 *
 * Why: the kernel /dev/smi read() path runs one programmed transfer per call
 * and then copies the block out of an uncached bounce buffer (~110 MB/s); the
 * SMI is stopped meanwhile, so 20-30 % of the signal is lost between reads
 * (measured on a Zero 2W, kernel 6.18). Gaps of ~40 lines with a jittery
 * wall-clock length make field reconstruction impossible.
 *
 * How:
 *  - /dev/smi's WRITE_SETTINGS ioctl programs width, timing and DMA
 *    thresholds (the kernel driver keeps the pins and the 125 MHz clock);
 *    the driver is never asked to read.
 *  - One DMA channel that neither Linux nor the GPU firmware uses runs a
 *    ring of control blocks forever: SMI_D -> 8 MiB ring, 32 KiB per block
 *    (a lite channel moves at most 65535 bytes per block), the last block
 *    links to the first. Nothing ever stops it but us. Default channel 7: on
 *    a Zero 2W (firmware of 2026-09) Linux uses 0/2/4/5 (5 = its SMI
 *    channel), the firmware uses 6 now and then, 7 is untouched.
 *  - Ring and control blocks are VideoCore memory from the firmware mailbox
 *    (MEM_FLAG_DIRECT, outside Linux RAM). If this process dies without
 *    cleanup, the DMA keeps writing only into that leaked buffer, never into
 *    memory Linux may reuse. The handle is kept in a file; the next start
 *    stops the channel and releases it.
 *  - The reader polls the channel's DEST_AD, copies new data once into the
 *    sample ring (uncached reads run at ~130-150 MB/s, far above the 25-50
 *    MB/s needed) and publishes contiguous blocks.
 *  - SMI_L counts transfers (32 bits: ~170 s at 25 MS/s); when the SMI
 *    reports done it is restarted, and only that block is flagged DISCONT.
 *
 * Register layout and the DMA/SMI start sequence follow Jeremy P Bentham's
 * public SMI work (iosoft.blog, github.com/jbentham/rpi, Apache-2.0); the code
 * here is a new 64-bit implementation.
 */
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <unistd.h>

#include "common.h"
#include "source.h"

/* ---- kernel SMI driver ABI (include/linux/broadcom/bcm2835_smi.h) ---- */
#define BCM2835_SMI_IOC_MAGIC 0x1
#define BCM2835_SMI_IOC_GET_SETTINGS _IO(BCM2835_SMI_IOC_MAGIC, 0)
#define BCM2835_SMI_IOC_WRITE_SETTINGS _IO(BCM2835_SMI_IOC_MAGIC, 1)
struct smi_settings {
    int data_width;
    bool pack_data;
    int read_setup_time, read_hold_time, read_pace_time, read_strobe_time;
    int write_setup_time, write_hold_time, write_pace_time, write_strobe_time;
    bool dma_enable, dma_passthrough_enable;
    int dma_read_thresh, dma_write_thresh, dma_panic_read_thresh, dma_panic_write_thresh;
};

/* ---- BCM2837 peripherals ---- */
#define PERI_PHYS_2837 0x3F000000u
#define BUS_PERI       0x7E000000u
#define SMI_OFS        0x600000u
#define DMA_OFS        0x007000u

#define SMI_CS   0x00
#define SMI_L    0x04
#define SMI_A    0x08
#define SMI_D    0x0c
#define SMI_DSR0 0x10
#define SMI_DMC  0x30
#define SMI_CS_ENABLE (1u << 0)
#define SMI_CS_DONE   (1u << 1)
#define SMI_CS_ACTIVE (1u << 2)
#define SMI_CS_START  (1u << 3)
#define SMI_CS_CLEAR  (1u << 4)
#define SMI_CS_WRITE  (1u << 5)
#define SMI_CS_SETERR (1u << 13)
#define SMI_CS_PXLDAT (1u << 14)
#define SMI_CS_AFERR  (1u << 25)
#define SMI_DMC_DMAEN (1u << 28)

#define DMA_CH(n)      ((n) * 0x100u)
#define DMA_CS         0x00
#define DMA_CONBLK_AD  0x04
#define DMA_TI         0x08
#define DMA_DEST_AD    0x10
#define DMA_DEBUG      0x20
#define DMA_ENABLE     0xff0
#define DMA_CS_ACTIVE  (1u << 0)
#define DMA_CS_END     (1u << 1)
#define DMA_CS_INT     (1u << 2)
#define DMA_CS_ERROR   (1u << 8)
#define DMA_CS_WAIT_WRITES (1u << 28)
#define DMA_CS_ABORT   (1u << 30)
#define DMA_CS_RESET   (1u << 31)
#define DMA_TI_DEST_INC (1u << 4)
#define DMA_TI_SRC_DREQ (1u << 10)
#define DMA_TI_PERMAP(p) ((uint32_t)(p) << 16)
#define DREQ_SMI       4u

/* ---- VideoCore mailbox ---- */
#define IOCTL_MBOX_PROPERTY _IOWR(100, 0, char *)
#define MEM_FLAG_DIRECT (1u << 2)
#define MEM_FLAG_ZERO   (1u << 4)

#define RING_BYTES   (8u << 20)
#define CHUNK_BYTES  (32u << 10)
#define NCB          (RING_BYTES / CHUNK_BYTES)
#define CB_BYTES     (NCB * 32u)
#define HANDLE_FILE  "/run/c5div-smidma"        /* buffers of a running capture */
#define OWNER_FILE   "/run/c5div-smidma-chan"   /* channels used since boot (stays until reboot) */
#define LOCK_FILE "/run/c5div-capture.lock"

typedef struct {
    uint32_t ti, src, dst, len, stride, next, pad[2];
} dma_cb_t;

typedef struct {
    uint32_t handle, bus, size;
    volatile uint8_t *virt;
} vcmem_t;

typedef struct {
    int smi_fd, mem_fd, mbox_fd, lock_fd;
    volatile uint32_t *smi, *dma;
    int chan;
    vcmem_t ring, cbs;
    uint8_t *tmp;          /* 8-bit mode staging */
} smidma_t;

/* For the fatal-signal handler: stop DMA and SMI before the process dies. */
static volatile uint32_t *g_smi_regs, *g_dma_regs;
static int g_chan = -1;

static inline uint32_t rd(volatile uint32_t *base, uint32_t ofs) { return base[ofs / 4]; }
static inline void wr(volatile uint32_t *base, uint32_t ofs, uint32_t v) { base[ofs / 4] = v; }

static uint32_t mbox_call(int fd, uint32_t tag, uint32_t a, uint32_t b, uint32_t c)
{
    uint32_t m[16] __attribute__((aligned(16))) = {0};
    m[0] = 9 * 4;
    m[1] = 0;
    m[2] = tag;
    m[3] = 12;
    m[4] = 12;
    m[5] = a;
    m[6] = b;
    m[7] = c;
    m[8] = 0;
    if (ioctl(fd, IOCTL_MBOX_PROPERTY, m) < 0)
        return 0;
    return m[5];
}

static void vc_free(smidma_t *p, vcmem_t *m)
{
    if (m->virt)
        munmap((void *)m->virt, m->size);
    if (m->handle) {
        mbox_call(p->mbox_fd, 0x3000e, m->handle, 0, 0);   /* unlock */
        mbox_call(p->mbox_fd, 0x3000f, m->handle, 0, 0);   /* release */
    }
    memset(m, 0, sizeof(*m));
}

static int vc_alloc(smidma_t *p, vcmem_t *m, uint32_t size)
{
    memset(m, 0, sizeof(*m));
    m->size = size;
    m->handle = mbox_call(p->mbox_fd, 0x3000c, size, 4096, MEM_FLAG_DIRECT | MEM_FLAG_ZERO);
    if (!m->handle)
        return -1;
    m->bus = mbox_call(p->mbox_fd, 0x3000d, m->handle, 0, 0);
    if (!m->bus) {
        vc_free(p, m);
        return -1;
    }
    void *v = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, p->mem_fd, m->bus & ~0xC0000000u);
    if (v == MAP_FAILED) {
        m->virt = NULL;
        vc_free(p, m);
        return -1;
    }
    m->virt = v;
    return 0;
}

static void dma_stop(volatile uint32_t *dma, int ch)
{
    volatile uint32_t *c = dma + DMA_CH(ch) / 4;
    c[DMA_CS / 4] = DMA_CS_ABORT;
    for (int i = 0; i < 1000 && (c[DMA_CS / 4] & DMA_CS_ACTIVE); i++)
        ;
    c[DMA_CS / 4] = DMA_CS_RESET;
}

static void smi_stop(volatile uint32_t *smi)
{
    smi[SMI_CS / 4] = 0;   /* disable: no more read cycles, no DREQ */
}

static void fatal_handler(int sig)
{
    if (g_smi_regs)
        smi_stop(g_smi_regs);
    if (g_dma_regs && g_chan >= 0)
        dma_stop(g_dma_regs, g_chan);
    signal(sig, SIG_DFL);
    raise(sig);
}

/* A previous run that was killed: stop its channel, release its buffers. */
/* One capture at a time: a second one would take the channel and buffers away
 * from the first. The lock goes away with its process, however it ends. */
static int capture_lock(smidma_t *p)
{
    p->lock_fd = open(LOCK_FILE, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (p->lock_fd < 0) {
        LOGE("smi: %s: %s (run as root)", LOCK_FILE, strerror(errno));
        return -1;
    }
    if (flock(p->lock_fd, LOCK_EX | LOCK_NB)) {
        char pid[16] = "";
        if (read(p->lock_fd, pid, sizeof(pid) - 1) > 0)
            pid[strcspn(pid, "\n")] = 0;
        LOGE("smi: another c5div is already capturing (pid %s). If that is the service: sudo systemctl stop c5div", pid);
        close(p->lock_fd);
        p->lock_fd = -1;
        return -1;
    }
    if (ftruncate(p->lock_fd, 0) == 0)
        dprintf(p->lock_fd, "%d\n", (int)getpid());
    return 0;
}

static void reclaim_previous(smidma_t *p)
{
    FILE *f = fopen(HANDLE_FILE, "r");
    if (!f)
        return;
    unsigned ch = 0, h1 = 0, h2 = 0;
    int n = fscanf(f, "%u %u %u", &ch, &h1, &h2);
    fclose(f);
    if (n == 3 && ch < 15) {
        smi_stop(p->smi);
        dma_stop(p->dma, (int)ch);
        usleep(1000);
        if (h1) {
            mbox_call(p->mbox_fd, 0x3000e, h1, 0, 0);
            mbox_call(p->mbox_fd, 0x3000f, h1, 0, 0);
        }
        if (h2) {
            mbox_call(p->mbox_fd, 0x3000e, h2, 0, 0);
            mbox_call(p->mbox_fd, 0x3000f, h2, 0, 0);
        }
        LOGI("smi: reclaimed DMA channel %u and buffers of a previous run", ch);
    }
    unlink(HANDLE_FILE);
}

static bool channel_owned(int ch)
{
    FILE *f = fopen(OWNER_FILE, "r");
    if (!f)
        return false;
    int c, owned = 0;
    while (fscanf(f, "%d", &c) == 1)
        if (c == ch)
            owned = 1;
    fclose(f);
    return owned;
}

static bool channel_reserved_by_linux(int ch)
{
    FILE *f = fopen("/proc/device-tree/soc/dma-controller@7e007000/brcm,dma-channel-mask", "rb");
    if (!f)
        return false;
    uint8_t b[4];
    bool reserved = false;
    if (fread(b, 1, 4, f) == 4) {
        uint32_t mask = (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3];
        reserved = (mask >> ch) & 1u;
    }
    fclose(f);
    return reserved;
}

static bool peripheral_base_is_2837(void)
{
    FILE *f = fopen("/proc/device-tree/soc/ranges", "rb");
    if (!f)
        return false;
    uint8_t b[8];
    bool ok = fread(b, 1, 8, f) == 8;
    fclose(f);
    if (!ok)
        return false;
    uint32_t child = (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3];
    uint32_t parent = (uint32_t)b[4] << 24 | (uint32_t)b[5] << 16 | (uint32_t)b[6] << 8 | b[7];
    return child == BUS_PERI && parent == PERI_PHYS_2837;
}

static int start_capture(smidma_t *p, const source_cfg_t *c)
{
    /* Control blocks: SMI_D -> ring chunk i, endless loop. */
    volatile uint32_t *cbw = (volatile uint32_t *)p->cbs.virt;
    const uint32_t ring_bus = p->ring.bus, cb_bus = p->cbs.bus;
    for (uint32_t i = 0; i < NCB; i++) {
        volatile uint32_t *cb = cbw + i * 8;
        cb[0] = DMA_TI_SRC_DREQ | DMA_TI_PERMAP(DREQ_SMI) | DMA_TI_DEST_INC;
        cb[1] = BUS_PERI + SMI_OFS + SMI_D;
        cb[2] = ring_bus + i * CHUNK_BYTES;
        cb[3] = CHUNK_BYTES;
        cb[4] = 0;
        cb[5] = cb_bus + ((i + 1) % NCB) * (uint32_t)sizeof(dma_cb_t);
        cb[6] = cb[7] = 0;
    }
    __sync_synchronize();

    /* SMI: idle, device 0, DMA requests on, packed FIFO words. */
    smi_stop(p->smi);
    usleep(100);
    uint32_t cs = rd(p->smi, SMI_CS);
    if (cs & SMI_CS_SETERR)
        wr(p->smi, SMI_CS, SMI_CS_SETERR);
    wr(p->smi, SMI_A, 0);
    wr(p->smi, SMI_DMC, rd(p->smi, SMI_DMC) | SMI_DMC_DMAEN);
    wr(p->smi, SMI_L, 0xFFFFFFF0u);
    wr(p->smi, SMI_CS, SMI_CS_ENABLE | SMI_CS_CLEAR | SMI_CS_PXLDAT);

    /* DMA channel: reset, clear errors, start the chain (it waits for DREQ). */
    volatile uint32_t *ch = p->dma + DMA_CH(p->chan) / 4;
    wr(p->dma, DMA_ENABLE, rd(p->dma, DMA_ENABLE) | (1u << p->chan));
    ch[DMA_CS / 4] = DMA_CS_RESET;
    usleep(10);
    ch[DMA_CS / 4] = DMA_CS_END | DMA_CS_INT;
    ch[DMA_DEBUG / 4] = 7;
    ch[DMA_CONBLK_AD / 4] = cb_bus;
    ch[DMA_CS / 4] = (8u << 16) | (15u << 20) | DMA_CS_WAIT_WRITES | DMA_CS_ACTIVE;

    /* Go. */
    wr(p->smi, SMI_CS, SMI_CS_ENABLE | SMI_CS_PXLDAT | SMI_CS_START);
    (void)c;
    return 0;
}

static void restart_smi(smidma_t *p)
{
    smi_stop(p->smi);
    wr(p->smi, SMI_L, 0xFFFFFFF0u);
    wr(p->smi, SMI_CS, SMI_CS_ENABLE | SMI_CS_PXLDAT);
    wr(p->smi, SMI_CS, SMI_CS_ENABLE | SMI_CS_PXLDAT | SMI_CS_START);
}

/* Copy out of the uncached ring. /dev/mem maps it as Device memory, where
 * unaligned accesses fault (glibc memcpy uses them): only 16-byte aligned
 * load pairs here. n, src and dst are multiples of 64. */
static void copy_from_device(uint8_t *dst, const volatile uint8_t *src, size_t n)
{
#if defined(__aarch64__)
    for (size_t i = 0; i < n; i += 64) {
        uint64_t a, b, c, d, e, f, g, h;
        __asm__ volatile("ldp %0, %1, [%8]\n\t"
                         "ldp %2, %3, [%8, #16]\n\t"
                         "ldp %4, %5, [%8, #32]\n\t"
                         "ldp %6, %7, [%8, #48]"
                         : "=&r"(a), "=&r"(b), "=&r"(c), "=&r"(d), "=&r"(e), "=&r"(f), "=&r"(g), "=&r"(h)
                         : "r"(src + i)
                         : "memory");
        uint64_t *o = (uint64_t *)(dst + i);
        o[0] = a; o[1] = b; o[2] = c; o[3] = d; o[4] = e; o[5] = f; o[6] = g; o[7] = h;
    }
#else
    for (size_t i = 0; i < n; i++)
        dst[i] = src[i];
#endif
}

/* Byte offset in the ring of the next DMA write, or -1 if not readable. */
static long dma_pos(smidma_t *p)
{
    volatile uint32_t *ch = p->dma + DMA_CH(p->chan) / 4;
    uint32_t d = ch[DMA_DEST_AD / 4];
    if (d < p->ring.bus || d >= p->ring.bus + RING_BYTES)
        return -1;
    return (long)(d - p->ring.bus) & ~3L;
}

static void *smidma_thread(void *arg)
{
    source_t *s = arg;
    thread_begin("capture");
    smidma_t *p = s->priv;
    const source_cfg_t *c = &s->cfg;
    const int bps = c->smi_width / 8;
    const double fs = c->fs;
    const uint64_t total = c->seconds > 0 ? (uint64_t)(c->seconds * fs) : 0;
    const uint32_t nblock = s->ring->block_samples;
    const size_t block_bytes = (size_t)nblock * bps;
    /* The ring holds RING_BYTES / (fs * bps) seconds: we must read faster. */
    const double ring_s = RING_BYTES / (fs * bps);

    struct sched_param sp = {.sched_priority = 50};
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        LOGW("smi: no SCHED_FIFO (run as root)");

    start_capture(p, c);
    long rpos = 0;                 /* ring byte offset of the next unread byte */
    uint64_t virt = 0, captured = 0;
    bool discont = false;
    double t_last = now_s(), t0 = t_last;
    ring_block_t *b = NULL;
    size_t fill = 0;               /* bytes in the current block */
    unsigned no_progress = 0;
    double t_warn = -1e9;

    while (s->running && atomic_load(&g_running)) {
        if (total && captured >= total)
            break;
        if (rd(p->smi, SMI_CS) & SMI_CS_DONE) {
            /* SMI_L exhausted (~170 s at 25 MS/s): one short gap. */
            restart_smi(p);
            discont = true;
            atomic_fetch_add(&s->gaps, 1);
            LOGD("smi: transfer count exhausted, restarted");
        }
        long w = dma_pos(p);
        double t = now_s();
        const bool lost = w < 0;        /* channel reset or pointing elsewhere */
        if (lost)
            w = rpos;
        long avail = w - rpos;
        if (avail < 0)
            avail += RING_BYTES;
        if (t - t_last > 0.8 * ring_s) {
            /* We may have been lapped: drop everything and restart here. */
            rpos = w;
            avail = 0;
            discont = true;
            atomic_fetch_add(&s->gaps, 1);
            LOGW("smi: reader stalled %.0f ms, data dropped", (t - t_last) * 1e3);
        }
        if (avail < 256) {
            if (++no_progress > 400) {   /* ~200 ms without data */
                no_progress = 0;
                uint32_t cs = rd(p->smi, SMI_CS);
                uint32_t dcs = p->dma[(DMA_CH(p->chan) + DMA_CS) / 4];
                if (t - t_warn > 10.0) {
                    t_warn = t;
                    LOGW("smi: no data for 200 ms (smi cs=0x%08x dma cs=0x%08x debug=0x%08x conblk=0x%08x dest=0x%08x)",
                         cs, dcs, p->dma[(DMA_CH(p->chan) + DMA_DEBUG) / 4],
                     p->dma[(DMA_CH(p->chan) + DMA_CONBLK_AD) / 4], p->dma[(DMA_CH(p->chan) + DMA_DEST_AD) / 4]);
                }
                if (lost || !(dcs & DMA_CS_ACTIVE)) {
                    /* Something stopped our channel: start over. */
                    start_capture(p, c);
                    rpos = 0;
                    discont = true;
                    atomic_fetch_add(&s->gaps, 1);
                    LOGI("smi: capture restarted");
                } else if (!(cs & SMI_CS_ACTIVE)) {
                    restart_smi(p);
                    discont = true;
                }
            }
            t_last = t;
            usleep(500);
            continue;
        }
        no_progress = 0;
        t_last = t;
        /* Keep a margin behind the DMA's write pointer; whole 64-byte units
         * only (aligned copies, see copy_from_device). */
        avail -= 64;
        avail &= ~63L;
        while (avail > 0) {
            if (!b) {
                b = ring_acquire(s->ring);
                fill = 0;
            }
            size_t want = block_bytes - fill;
            size_t n = (size_t)avail < want ? (size_t)avail : want;
            size_t first = RING_BYTES - (size_t)rpos;
            if (first > n)
                first = n;
            uint8_t *dst = bps == 2 ? (uint8_t *)b->data + fill : p->tmp + fill;
            copy_from_device(dst, p->ring.virt + rpos, first);
            if (n > first)
                copy_from_device(dst + first, p->ring.virt, n - first);
            rpos = (long)((rpos + n) % RING_BYTES);
            fill += n;
            avail -= (long)n;
            if (fill == block_bytes) {
                if (bps == 1)
                    for (uint32_t i = 0; i < nblock; i++)
                        b->data[i] = p->tmp[i];
                b->start = virt;
                b->n = nblock;
                b->flags = discont ? RING_F_DISCONT : 0;
                b->gap_est = 0;
                b->t_wall = now_s();
                discont = false;
                virt += nblock;
                captured += nblock;
                ring_publish(s->ring, b);
                b = NULL;
                atomic_fetch_add(&s->samples, nblock);
                atomic_fetch_add(&s->blocks, 1);
                double el = now_s() - t0;
                if (el > 0.2)
                    atomic_store(&s->rate_meas, (double)captured / el);
            }
        }
        usleep(300);
    }
    smi_stop(p->smi);
    dma_stop(p->dma, p->chan);
    ring_set_eof(s->ring);
    return NULL;
}

static void smidma_close(source_t *s)
{
    smidma_t *p = s->priv;
    if (!p)
        return;
    if (p->smi)
        smi_stop(p->smi);
    if (p->dma && p->chan >= 0)
        dma_stop(p->dma, p->chan);
    usleep(1000);
    g_smi_regs = g_dma_regs = NULL;
    vc_free(p, &p->ring);
    vc_free(p, &p->cbs);
    if (p->lock_fd >= 0)        /* still holding the lock: the file is ours */
        unlink(HANDLE_FILE);
    if (p->smi)
        munmap((void *)p->smi, 4096);
    if (p->dma)
        munmap((void *)p->dma, 4096);
    if (p->mem_fd >= 0) close(p->mem_fd);
    if (p->mbox_fd >= 0) close(p->mbox_fd);
    if (p->smi_fd >= 0) close(p->smi_fd);
    if (p->lock_fd >= 0) close(p->lock_fd);
    free(p->tmp);
    free(p);
    s->priv = NULL;
}

int source_smidma_init(source_t *s)
{
    const source_cfg_t *c = &s->cfg;
    if (c->smi_width != 8 && c->smi_width != 16) {
        LOGE("smi width must be 8 or 16");
        return -1;
    }
    if (!peripheral_base_is_2837()) {
        LOGE("smi: gapless capture supports BCM2837 boards (Pi Zero 2W, Pi 3) only; try --source smidev");
        return -1;
    }
    const int chan = c->dma_chan;
    if (chan < 0 || chan > 14) {
        LOGE("smi: DMA channel must be 0..14");
        return -1;
    }
    if (channel_reserved_by_linux(chan) && !c->dma_force) {
        LOGE("smi: DMA channel %d is in Linux's brcm,dma-channel-mask; choose a free one (--dma-chan)", chan);
        return -1;
    }
    smidma_t *p = calloc(1, sizeof(*p));
    if (!p)
        return -1;
    p->smi_fd = p->mem_fd = p->mbox_fd = p->lock_fd = -1;
    p->chan = chan;
    s->priv = p;
    s->close_fn = smidma_close;
    if (capture_lock(p))
        goto fail;

    /* 1. Width, timing and DMA thresholds through the kernel driver. */
    p->smi_fd = open(c->smi_dev, O_RDWR);
    if (p->smi_fd < 0) {
        LOGE("cannot open %s: %s (dtoverlay=smi-dev loaded? running as root?)", c->smi_dev, strerror(errno));
        goto fail;
    }
    struct smi_settings st;
    memset(&st, 0, sizeof(st));
    if (ioctl(p->smi_fd, BCM2835_SMI_IOC_GET_SETTINGS, &st) != 0) {
        LOGE("SMI GET_SETTINGS failed: %s", strerror(errno));
        goto fail;
    }
    st.data_width = c->smi_width == 16 ? 1 : 0;
    st.pack_data = true;
    st.read_setup_time = c->smi_setup;
    st.read_strobe_time = c->smi_strobe;
    st.read_hold_time = c->smi_hold;
    st.read_pace_time = c->smi_pace;
    st.dma_enable = true;
    if (ioctl(p->smi_fd, BCM2835_SMI_IOC_WRITE_SETTINGS, &st) != 0) {
        LOGE("SMI WRITE_SETTINGS failed: %s", strerror(errno));
        goto fail;
    }

    /* 2. Registers. */
    p->mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    p->mbox_fd = open("/dev/vcio", 0);
    if (p->mem_fd < 0 || p->mbox_fd < 0) {
        LOGE("smi: /dev/mem and /dev/vcio are needed (run as root)");
        goto fail;
    }
    void *v = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, p->mem_fd, PERI_PHYS_2837 + SMI_OFS);
    if (v == MAP_FAILED) goto fail_map;
    p->smi = v;
    v = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, p->mem_fd, PERI_PHYS_2837 + DMA_OFS);
    if (v == MAP_FAILED) goto fail_map;
    p->dma = v;

    /* 3. A killed previous run may still own the channel. */
    reclaim_previous(p);
    /* Refuse a channel someone else is using or has used since boot (the
     * firmware's channels keep their last TI/destination). */
    const uint32_t dcs = p->dma[(DMA_CH(chan) + DMA_CS) / 4];
    const uint32_t dti = p->dma[(DMA_CH(chan) + DMA_TI) / 4];
    const uint32_t ddst = p->dma[(DMA_CH(chan) + DMA_DEST_AD) / 4];
    if ((dcs & DMA_CS_ACTIVE) || ((dti || ddst) && !c->dma_force && !channel_owned(chan))) {
        LOGE("smi: DMA channel %d is or was in use by someone else (cs=0x%08x ti=0x%08x dest=0x%08x); "
             "choose another with --dma-chan", chan, dcs, dti, ddst);
        goto fail;
    }

    /* 4. Firmware-owned buffers for the ring and the control blocks. */
    if (vc_alloc(p, &p->ring, RING_BYTES) || vc_alloc(p, &p->cbs, CB_BYTES)) {
        LOGE("smi: VideoCore memory allocation failed (gpu_mem too small?)");
        goto fail;
    }
    FILE *f = fopen(HANDLE_FILE, "w");
    if (f) {
        fprintf(f, "%d %u %u\n", chan, p->ring.handle, p->cbs.handle);
        fclose(f);
    }
    if (!channel_owned(chan) && (f = fopen(OWNER_FILE, "a"))) {
        fprintf(f, "%d\n", chan);
        fclose(f);
    }

    g_smi_regs = p->smi;
    g_dma_regs = p->dma;
    g_chan = chan;
    signal(SIGSEGV, fatal_handler);
    signal(SIGBUS, fatal_handler);
    signal(SIGABRT, fatal_handler);
    signal(SIGFPE, fatal_handler);
    signal(SIGILL, fatal_handler);

    if (c->smi_width == 8)
        p->tmp = aligned_alloc(64, (s->cfg.block_samples + 63u) & ~63u);
    if ((s->cfg.block_samples * (c->smi_width / 8)) % 64u) {
        LOGE("smi: --block must be a multiple of %u samples", 64u / (c->smi_width / 8));
        goto fail;
    }
    s->thread_fn = smidma_thread;
    s->realtime = true;
    s->nlanes = c->smi_width == 16 ? 2 : 1;
    const int cycles = c->smi_setup + c->smi_strobe + c->smi_hold + c->smi_pace;
    LOGI("smi: gapless DMA capture, channel %d, %d-bit, %d cycles/sample (%.2f MS/s at 125 MHz), ring %u KiB",
         chan, c->smi_width, cycles, 125.0 / cycles, RING_BYTES / 1024);
    return 0;

fail_map:
    LOGE("smi: cannot map peripheral registers: %s", strerror(errno));
fail:
    smidma_close(s);
    s->close_fn = NULL;
    return -1;
}
