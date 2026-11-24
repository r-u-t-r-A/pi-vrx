/* Web viewer for phones/laptops on the Pi's Wi-Fi AP.
 *
 *   /             viewer page (fullscreen video + receiver status)
 *   /stream.mjpg  multipart MJPEG (works in any browser, VLC, ffplay)
 *   /snapshot.jpg single frame
 *   /status.json  receiver/diversity status
 *
 * One encoder thread compresses the newest frame (libjpeg, raw YUV 4:2:0
 * input, no colour conversion) at most `fps` times per second, only while
 * someone is watching. Clients always get the newest JPEG; a client whose
 * socket still holds an unsent frame skips frames instead of building
 * latency.
 */
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <setjmp.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/sockios.h>

#include <jpeglib.h>

#include "common.h"
#include "outputs.h"

struct out_http {
    http_cfg_t cfg;
    frame_pub_t *pub;
    int lfd[2], nlfd;
    pthread_t th_accept, th_enc;
    atomic_bool run;
    pthread_mutex_t mu;
    pthread_cond_t cv;
    uint8_t *jpg;
    size_t jlen;
    uint64_t jseq;
    atomic_int streamers;
    atomic_int waiters;
    atomic_int clients;
    frame_t fr;
};

static const char k_page[] =
    "<!doctype html><html><head><meta charset=utf-8>"
    "<meta name=viewport content=\"width=device-width,initial-scale=1\">"
    "<title>C5 diversity VRX</title><style>"
    "html,body{margin:0;height:100%;background:#000;color:#ddd;font:14px system-ui,sans-serif}"
    "#v{width:100%;height:100%;object-fit:contain;display:block}"
    "#s{position:fixed;left:8px;top:8px;background:rgba(0,0,0,.55);padding:6px 10px;border-radius:6px;"
    "white-space:pre;font-variant-numeric:tabular-nums}"
    "#fs{position:fixed;right:8px;top:8px;background:rgba(0,0,0,.55);color:#ddd;border:1px solid #555;"
    "border-radius:6px;padding:6px 10px}"
    "</style></head><body>"
    "<img id=v src=\"/stream.mjpg\" alt=\"video\"><div id=s>connecting...</div>"
    "<button id=fs>fullscreen</button><script>"
    "const s=document.getElementById('s'),v=document.getElementById('v');"
    "document.getElementById('fs').onclick=()=>{const e=document.documentElement;"
    "(e.requestFullscreen||e.webkitRequestFullscreen||(()=>{})).call(e)};"
    "v.onerror=()=>setTimeout(()=>{v.src='/stream.mjpg?'+Date.now()},1000);"
    "async function poll(){try{const r=await fetch('/status.json',{cache:'no-store'});const j=await r.json();"
    "s.textContent=j.rx.map(x=>'RX '+x.name+': '+(x.locked?(x.std+' '+x.snr_db.toFixed(1)+' dB'):'no lock')"
    "+'  use '+Math.round(x.use*100)+'%').join('\\n')+'\\n'+j.fps.toFixed(1)+' fps  '+j.mode;"
    "}catch(e){s.textContent='status unavailable'}setTimeout(poll,500)}poll();"
    "</script></body></html>";

/* ------------------------------------------------------------ JPEG */

typedef struct {
    struct jpeg_error_mgr pub;
    jmp_buf jb;
} jerr_t;

static void jerr_exit(j_common_ptr c)
{
    jerr_t *e = (jerr_t *)c->err;
    longjmp(e->jb, 1);
}

static int encode_jpeg(const frame_t *f, int quality, uint8_t **out, size_t *outlen)
{
    struct jpeg_compress_struct c;
    jerr_t je;
    unsigned char *mem = NULL;
    unsigned long memlen = 0;
    c.err = jpeg_std_error(&je.pub);
    je.pub.error_exit = jerr_exit;
    if (setjmp(je.jb)) {
        jpeg_destroy_compress(&c);
        free(mem);
        return -1;
    }
    jpeg_create_compress(&c);
    jpeg_mem_dest(&c, &mem, &memlen);
    c.image_width = (JDIMENSION)f->w;
    c.image_height = (JDIMENSION)f->h;
    c.input_components = 3;
    c.in_color_space = JCS_YCbCr;
    jpeg_set_defaults(&c);
    jpeg_set_colorspace(&c, JCS_YCbCr);
    jpeg_set_quality(&c, quality, TRUE);
    c.raw_data_in = TRUE;
    c.dct_method = JDCT_IFAST;
    c.comp_info[0].h_samp_factor = 2;
    c.comp_info[0].v_samp_factor = 2;
    c.comp_info[1].h_samp_factor = c.comp_info[1].v_samp_factor = 1;
    c.comp_info[2].h_samp_factor = c.comp_info[2].v_samp_factor = 1;
    jpeg_start_compress(&c, TRUE);
    JSAMPROW yr[16], ur[8], vr[8];
    JSAMPARRAY planes[3] = {yr, ur, vr};
    const int cw = f->w / 2;
    for (int y = 0; y < f->h; y += 16) {
        for (int i = 0; i < 16; i++)
            yr[i] = f->y + (size_t)clampi(y + i, 0, f->h - 1) * f->w;
        for (int i = 0; i < 8; i++) {
            int cy = clampi(y / 2 + i, 0, f->h / 2 - 1);
            ur[i] = f->u + (size_t)cy * cw;
            vr[i] = f->v + (size_t)cy * cw;
        }
        jpeg_write_raw_data(&c, planes, 16);
    }
    jpeg_finish_compress(&c);
    jpeg_destroy_compress(&c);
    *out = mem;
    *outlen = memlen;
    return 0;
}

static void *enc_thread(void *arg)
{
    out_http_t *o = arg;
    thread_begin("jpeg");
    setpriority(PRIO_PROCESS, (id_t)gettid(), 5);   /* decoding goes first */
    uint64_t fseq = 0;
    double last = 0;
    while (atomic_load(&o->run)) {
        if (atomic_load(&o->streamers) == 0 && atomic_load(&o->waiters) == 0) {
            usleep(20000);
            continue;
        }
        if (!frame_pub_get(o->pub, &o->fr, &fseq, 200))
            continue;
        double now = now_s();
        if (atomic_load(&o->waiters) == 0 && now - last < 1.0 / o->cfg.fps - 0.002)
            continue;
        last = now;
        uint8_t *jpg;
        size_t len;
        if (encode_jpeg(&o->fr, o->cfg.quality, &jpg, &len))
            continue;
        pthread_mutex_lock(&o->mu);
        free(o->jpg);
        o->jpg = jpg;
        o->jlen = len;
        o->jseq++;
        pthread_cond_broadcast(&o->cv);
        pthread_mutex_unlock(&o->mu);
    }
    return NULL;
}

/* Copy the next JPEG newer than *seq. */
static bool next_jpeg(out_http_t *o, uint64_t *seq, uint8_t **buf, size_t *cap, size_t *len)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 2;
    bool ok = false;
    pthread_mutex_lock(&o->mu);
    while (o->jseq == *seq && atomic_load(&o->run))
        if (pthread_cond_timedwait(&o->cv, &o->mu, &ts) == ETIMEDOUT)
            break;
    if (o->jseq != *seq && o->jpg) {
        if (*cap < o->jlen) {
            *buf = realloc(*buf, o->jlen);
            *cap = o->jlen;
        }
        memcpy(*buf, o->jpg, o->jlen);
        *len = o->jlen;
        *seq = o->jseq;
        ok = true;
    }
    pthread_mutex_unlock(&o->mu);
    return ok;
}

/* ------------------------------------------------------------ HTTP */

static bool send_all(int fd, const void *p, size_t n)
{
    const uint8_t *b = p;
    while (n > 0) {
        ssize_t w = send(fd, b, n, MSG_NOSIGNAL);
        if (w < 0 && errno == EINTR)
            continue;
        if (w <= 0)
            return false;
        b += w;
        n -= (size_t)w;
    }
    return true;
}

static bool send_response(int fd, const char *status, const char *type, const void *body, size_t len)
{
    char h[256];
    int n = snprintf(h, sizeof(h),
                     "HTTP/1.0 %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\nCache-Control: no-store\r\n"
                     "Connection: close\r\n\r\n",
                     status, type, len);
    return send_all(fd, h, (size_t)n) && send_all(fd, body, len);
}

typedef struct {
    out_http_t *o;
    int fd;
} client_t;

static void serve_stream(out_http_t *o, int fd)
{
    static const char hdr[] =
        "HTTP/1.0 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=c5divframe\r\n"
        "Cache-Control: no-store\r\nConnection: close\r\n\r\n";
    if (!send_all(fd, hdr, sizeof(hdr) - 1))
        return;
    atomic_fetch_add(&o->streamers, 1);
    uint64_t seq = 0;
    uint8_t *buf = NULL;
    size_t cap = 0, len = 0;
    while (atomic_load(&o->run)) {
        if (!next_jpeg(o, &seq, &buf, &cap, &len))
            continue;
        int queued = 0;
        if (ioctl(fd, SIOCOUTQ, &queued) == 0 && (size_t)queued > len)
            continue;   /* client is behind: drop this frame rather than add latency */
        char ph[128];
        int n = snprintf(ph, sizeof(ph), "--c5divframe\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                         len);
        if (!send_all(fd, ph, (size_t)n) || !send_all(fd, buf, len) || !send_all(fd, "\r\n", 2))
            break;
    }
    free(buf);
    atomic_fetch_sub(&o->streamers, 1);
}

static void *client_thread(void *arg)
{
    client_t *c = arg;
    out_http_t *o = c->o;
    int fd = c->fd;
    free(c);
    atomic_fetch_add(&o->clients, 1);

    struct timeval tv = {.tv_sec = 5};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    char req[2048];
    size_t got = 0;
    while (got < sizeof(req) - 1) {
        ssize_t r = recv(fd, req + got, sizeof(req) - 1 - got, 0);
        if (r <= 0)
            break;
        got += (size_t)r;
        req[got] = 0;
        if (strstr(req, "\r\n\r\n"))
            break;
    }
    req[got] = 0;
    char path[256] = "";
    if (sscanf(req, "GET %255s", path) == 1) {
        char *q = strchr(path, '?');
        if (q)
            *q = 0;
        if (strcmp(path, "/") == 0 || strcmp(path, "/index.html") == 0) {
            send_response(fd, "200 OK", "text/html; charset=utf-8", k_page, sizeof(k_page) - 1);
        } else if (strcmp(path, "/stream.mjpg") == 0) {
            serve_stream(o, fd);
        } else if (strcmp(path, "/snapshot.jpg") == 0) {
            atomic_fetch_add(&o->waiters, 1);
            uint64_t seq;
            pthread_mutex_lock(&o->mu);
            seq = o->jseq;
            pthread_mutex_unlock(&o->mu);
            uint8_t *buf = NULL;
            size_t cap = 0, len = 0;
            if (next_jpeg(o, &seq, &buf, &cap, &len))
                send_response(fd, "200 OK", "image/jpeg", buf, len);
            else
                send_response(fd, "503 Service Unavailable", "text/plain", "no video\n", 9);
            atomic_fetch_sub(&o->waiters, 1);
            free(buf);
        } else if (strcmp(path, "/status.json") == 0) {
            char js[2048];
            size_t n = o->cfg.status ? o->cfg.status(o->cfg.status_ctx, js, sizeof(js)) : 0;
            if (n == 0) {
                strcpy(js, "{}");
                n = 2;
            }
            send_response(fd, "200 OK", "application/json", js, n);
        } else {
            send_response(fd, "404 Not Found", "text/plain", "not found\n", 10);
        }
    } else {
        send_response(fd, "400 Bad Request", "text/plain", "bad request\n", 12);
    }
    close(fd);
    atomic_fetch_sub(&o->clients, 1);
    return NULL;
}

static void *accept_thread(void *arg)
{
    out_http_t *o = arg;
    while (atomic_load(&o->run)) {
        struct pollfd p[2] = {{.fd = o->lfd[0], .events = POLLIN}, {.fd = o->lfd[1], .events = POLLIN}};
        if (poll(p, (nfds_t)o->nlfd, 200) <= 0)
            continue;
        int fd = accept(p[0].revents ? o->lfd[0] : o->lfd[1], NULL, NULL);
        if (fd < 0)
            continue;
        if (atomic_load(&o->clients) >= 16) {
            close(fd);
            continue;
        }
        client_t *c = malloc(sizeof(*c));
        if (!c) {
            close(fd);
            continue;
        }
        c->o = o;
        c->fd = fd;
        pthread_t th;
        if (pthread_create(&th, NULL, client_thread, c)) {
            close(fd);
            free(c);
            continue;
        }
        pthread_detach(th);
    }
    return NULL;
}

static int listen_on(int port)
{
    const int fd = socket(AF_INET6, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0)
        return -1;
    int one = 1, zero = 0;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &zero, sizeof(zero));
    struct sockaddr_in6 a = {.sin6_family = AF_INET6, .sin6_port = htons((uint16_t)port), .sin6_addr = in6addr_any};
    if (bind(fd, (struct sockaddr *)&a, sizeof(a)) || listen(fd, 8)) {
        const int e = errno;
        close(fd);
        errno = e;
        return -1;
    }
    return fd;
}

out_http_t *out_http_start(const http_cfg_t *cfg, frame_pub_t *pub)
{
    out_http_t *o = calloc(1, sizeof(*o));
    if (!o)
        return NULL;
    o->cfg = *cfg;
    if (o->cfg.fps <= 0)
        o->cfg.fps = 30;
    if (o->cfg.quality <= 0)
        o->cfg.quality = 70;
    o->pub = pub;
    pthread_mutex_init(&o->mu, NULL);
    pthread_cond_init(&o->cv, NULL);
    if (frame_alloc(&o->fr))
        goto fail;

    /* port 0: the plain address and the familiar :8080, whichever can be had */
    const int ports[2] = {cfg->port ? cfg->port : 80, cfg->port ? 0 : 8080};
    o->nlfd = 0;
    char list[32] = "";
    for (int i = 0; i < 2; i++) {
        if (!ports[i])
            continue;
        const int fd = listen_on(ports[i]);
        if (fd < 0) {
            if (cfg->port)
                LOGE("web: cannot listen on port %d: %s", ports[i], strerror(errno));
            continue;
        }
        o->lfd[o->nlfd++] = fd;
        snprintf(list + strlen(list), sizeof(list) - strlen(list), "%s%d", list[0] ? " and " : "", ports[i]);
    }
    if (!o->nlfd) {
        if (!cfg->port)
            LOGE("web: ports 80 and 8080 are both unavailable: %s", strerror(errno));
        goto fail;
    }
    atomic_store(&o->run, true);
    pthread_create(&o->th_enc, NULL, enc_thread, o);
    pthread_create(&o->th_accept, NULL, accept_thread, o);
    LOGI("web output: http://<this-host>/ on port %s  (MJPEG %d fps max, quality %d)", list, o->cfg.fps,
         o->cfg.quality);
    return o;
fail:
    frame_release(&o->fr);
    free(o);
    return NULL;
}

void out_http_stop(out_http_t *o)
{
    if (!o)
        return;
    atomic_store(&o->run, false);
    pthread_mutex_lock(&o->mu);
    pthread_cond_broadcast(&o->cv);
    pthread_mutex_unlock(&o->mu);
    pthread_join(o->th_accept, NULL);
    pthread_join(o->th_enc, NULL);
    for (int i = 0; i < o->nlfd; i++)
        close(o->lfd[i]);
    /* Detached client threads notice run=false within ~2 s; give them a moment. */
    for (int i = 0; i < 30 && atomic_load(&o->clients) > 0; i++)
        usleep(100000);
    if (atomic_load(&o->clients) == 0) {
        free(o->jpg);
        frame_release(&o->fr);
        free(o);
    }
}
