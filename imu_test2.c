#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <hw/i2c.h>
#include <devctl.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define I2C_DEV     "/dev/i2c1"
#define IMU_ADDR    0x6A
#define ACCEL_SENS  0.000061
#define HTTP_PORT   8080

// --- NETWORK PIPELINE TARGET DEFINITION ---
#define FLASK_SERVER_IP  "192.168.137.9" //
#define OUTBOUND_UDP_PORT 5001

#define SAMPLE_HZ           25.0
#define BREATH_MIN_BPM      8.0
#define BREATH_MAX_BPM      40.0
#define BREATH_MAX_HZ       (BREATH_MAX_BPM / 60.0)
#define APNEA_SECONDS       15.0
#define POS_ALARM_SAMPLES   25
#define REFRACTORY_SEC      (60.0 / BREATH_MAX_BPM)
#define BREATH_MIN_AMP_G    0.012
#define HIST_LEN            64

/* Fixed 12-byte binary structure matching Python unpack expectations */
typedef struct {
    float pitch;
    float roll;
    float yaw;
} IMUTelemetry;

typedef enum {
    POS_BACK, POS_STOMACH, POS_LEFT_SIDE, POS_RIGHT_SIDE, POS_UNKNOWN
} position_t;

static int g_fd = -1;

/* Latest values for the web page */
static volatile double g_breath_bpm = 0;
static volatile int g_pos = POS_UNKNOWN;
static volatile int g_apnea = 0;
static volatile int g_alert_pos = 0;
static volatile double g_y_sig = 0;

int write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    struct { i2c_sendrecv_t hdr; uint8_t data[2]; } msg;
    memset(&msg, 0, sizeof(msg));
    msg.hdr.slave.addr = addr;
    msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg.hdr.send_len = 2;
    msg.hdr.recv_len = 0;
    msg.hdr.stop = 1;
    msg.data[0] = reg;
    msg.data[1] = val;
    return devctl(g_fd, DCMD_I2C_SENDRECV, &msg, sizeof(msg), NULL);
}

int16_t read_axis(uint8_t addr, uint8_t reg_low) {
    struct { i2c_sendrecv_t hdr; uint8_t data[2]; } msg;
    memset(&msg, 0, sizeof(msg));
    msg.hdr.slave.addr = addr;
    msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg.hdr.send_len = 1;
    msg.hdr.recv_len = 2;
    msg.hdr.stop = 1;
    msg.data[0] = reg_low;
    if (devctl(g_fd, DCMD_I2C_SENDRECV, &msg, sizeof(msg), NULL) != 0)
        return 0;
    return (int16_t)((msg.data[1] << 8) | msg.data[0]);
}

position_t classify_position(double ax, double ay, double az) {
    const double THRESH = 0.6;
    if (az > THRESH)  return POS_BACK;
    if (az < -THRESH) return POS_STOMACH;
    if (ay > THRESH)  return POS_RIGHT_SIDE;
    if (ay < -THRESH) return POS_LEFT_SIDE;
    return POS_UNKNOWN;
}

const char* position_name(position_t p) {
    switch (p) {
        case POS_BACK:       return "BACK (safe)";
        case POS_STOMACH:    return "STOMACH - WARNING";
        case POS_LEFT_SIDE:  return "LEFT SIDE - caution";
        case POS_RIGHT_SIDE: return "RIGHT SIDE - caution";
        default:             return "unknown/transitional";
    }
}

typedef struct {
    double y_slow, prev_ac, hp, lp;
    int primed;
} breath_filter_t;

static void breath_filter_init(breath_filter_t *f) { memset(f, 0, sizeof(*f)); }

static double breath_filter_update(breath_filter_t *f, double ay, double dt) {
    const double tau_slow = 1.0 / (2.0 * M_PI * 0.25);
    double a_slow = dt / (tau_slow + dt);
    if (!f->primed) {
        f->y_slow = ay;
        f->primed = 1;
        return 0.0;
    }
    f->y_slow += a_slow * (ay - f->y_slow);
    double ac = ay - f->y_slow;
    const double tau_hp = 1.0 / (2.0 * M_PI * 0.10);
    double ah = tau_hp / (tau_hp + dt);
    f->hp = ah * (f->hp + ac - f->prev_ac);
    f->prev_ac = ac;
    const double tau_lp = 1.0 / (2.0 * M_PI * BREATH_MAX_HZ);
    double al = dt / (tau_lp + dt);
    f->lp += al * (f->hp - f->lp);
    return f->lp;
}

typedef struct {
    double hist[HIST_LEN];
    int idx, count, n_intervals, i_intervals, breath_count, looking_for_peak;
    double last_peak_t, last_breath_t, intervals[6], bpm;
} breath_detector_t;

static void breath_detector_init(breath_detector_t *d, double t) {
    memset(d, 0, sizeof(*d));
    d->last_peak_t = t;
    d->last_breath_t = t;
    d->looking_for_peak = 1;
}

static double hist_rms(const breath_detector_t *d) {
    if (d->count < 8) return 0.0;
    int n = d->count < HIST_LEN ? d->count : HIST_LEN;
    double sum = 0.0;
    for (int i = 0; i < n; i++) sum += d->hist[i] * d->hist[i];
    return sqrt(sum / n);
}

static int breath_detector_update(breath_detector_t *d, double sig, double t) {
    d->hist[d->idx] = sig;
    d->idx = (d->idx + 1) % HIST_LEN;
    if (d->count < HIST_LEN) d->count++;

    double rms = hist_rms(d);
    double thr = rms * 0.45;
    if (thr < BREATH_MIN_AMP_G) thr = BREATH_MIN_AMP_G;
    if (thr > 0.08) thr = 0.08;

    if (t - d->last_peak_t < REFRACTORY_SEC) return 0;

    int detected = 0;
    if (d->looking_for_peak) {
        if (sig > thr) {
            double interval = t - d->last_breath_t;
            if (interval >= REFRACTORY_SEC) {
                double bpm_inst = 60.0 / interval;
                if (bpm_inst >= BREATH_MIN_BPM && bpm_inst <= BREATH_MAX_BPM) {
                    d->intervals[d->i_intervals] = interval;
                    d->i_intervals = (d->i_intervals + 1) % 6;
                    if (d->n_intervals < 6) d->n_intervals++;
                    double sum = 0.0;
                    for (int i = 0; i < d->n_intervals; i++) sum += d->intervals[i];
                    d->bpm = 60.0 / (sum / d->n_intervals);
                    d->breath_count++;
                    detected = 1;
                }
            }
            d->last_peak_t = t;
            d->last_breath_t = t;
            d->looking_for_peak = 0;
        }
    } else if (sig < -thr * 0.35) {
        d->looking_for_peak = 1;
    }
    return detected;
}

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static int http_listen(int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }
    if (listen(fd, 4) < 0) {
        close(fd);
        return -1;
    }
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    return fd;
}

static void http_send_all(int fd, const char *data, size_t len) {
    size_t off = 0;
    while (off < len) {
        ssize_t n = write(fd, data + off, len - off);
        if (n <= 0) break;
        off += (size_t)n;
    }
}

static void http_reply(int fd, const char *ctype, const char *body) {
    char hdr[256];
    int blen = (int)strlen(body);
    int hlen = snprintf(hdr, sizeof(hdr),
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "Cache-Control: no-store\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n",
        ctype, blen);
    http_send_all(fd, hdr, (size_t)hlen);
    http_send_all(fd, body, (size_t)blen);
}

static void build_json(char *out, size_t outsz) {
    snprintf(out, outsz,
        "{\"position\":\"%s\",\"breath\":%.0f,\"y_sig\":%.3f,"
        "\"apnea\":%s,\"alert_pos\":%s,"
        "\"line\":\"%s | breath %.0f/min%s\"}",
        position_name((position_t)g_pos),
        g_breath_bpm,
        g_y_sig,
        g_apnea ? "true" : "false",
        g_alert_pos ? "true" : "false",
        position_name((position_t)g_pos),
        g_breath_bpm,
        g_apnea ? " | APNEA" : "");
}

static const char *PAGE_HTML =
"<!DOCTYPE html>\n"
"<html lang=\"en\">\n"
"<head>\n"
"<meta charset=\"UTF-8\">\n"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
"<title>TinyVitals</title>\n"
"<style>\n"
"  :root { --bg:#0f1419; --card:#1a2332; --ink:#e7ecf3; --mute:#8b9bb4;\n"
"          --ok:#3dd68c; --warn:#ffb020; --bad:#ff5c5c; --line:#2a3548; }\n"
"  * { box-sizing:border-box; }\n"
"  body { margin:0; min-height:100vh; font-family: system-ui, sans-serif;\n"
"         background:radial-gradient(1200px 600px at 20% -10%, #1e3a5f 0%, var(--bg) 55%);\n"
"         color:var(--ink); padding:24px; }\n"
"  h1 { font-weight:700; letter-spacing:-0.02em; margin:0 0 4px; font-size:1.75rem; }\n"
"  .sub { color:var(--mute); margin-bottom:28px; font-size:0.95rem; }\n"
"  .grid { display:grid; gap:16px; max-width:720px; }\n"
"  .card { background:var(--card); border:1px solid var(--line); border-radius:16px;\n"
"          padding:20px 22px; }\n"
"  .label { color:var(--mute); font-size:0.8rem; text-transform:uppercase;\n"
"           letter-spacing:0.06em; margin-bottom:8px; }\n"
"  .value { font-size:2rem; font-weight:700; font-variant-numeric:tabular-nums; }\n"
"  .unit { font-size:1rem; color:var(--mute); font-weight:500; margin-left:4px; }\n"
"  .line { font-family: ui-monospace, monospace; font-size:1.05rem;\n"
"          background:#0c1018; border-radius:10px; padding:14px 16px;\n"
"          border:1px solid var(--line); white-space:pre-wrap; }\n"
"  .pill { display:inline-block; padding:6px 12px; border-radius:999px;\n"
"          font-size:0.85rem; font-weight:600; }\n"
"  .pill.ok { background:rgba(61,214,140,.15); color:var(--ok); }\n"
"  .pill.warn { background:rgba(255,176,32,.15); color:var(--warn); }\n"
"  .pill.bad { background:rgba(255,92,92,.18); color:var(--bad); }\n"
"  .log { max-height:240px; overflow:auto; font-family:ui-monospace,monospace;\n"
"         font-size:0.85rem; line-height:1.5; color:var(--mute); margin:0; }\n"
"</style>\n"
"</head>\n"
"<body>\n"
"  <h1>TinyVitals</h1>\n"
"  <p class=\"sub\">Live from QNX Pi IMU — refresh via /api.json</p>\n"
"  <div class=\"grid\">\n"
"    <div class=\"card\">\n"
"      <div class=\"label\">Status line</div>\n"
"      <div class=\"line\" id=\"line\">connecting…</div>\n"
"    </div>\n"
"    <div class=\"card\">\n"
"      <div class=\"label\">Position</div>\n"
"      <div class=\"value\" id=\"pos\">--</div>\n"
"      <div style=\"margin-top:10px\"><span class=\"pill ok\" id=\"pill\">—</span></div>\n"
"    </div>\n"
"    <div class=\"card\">\n"
"      <div class=\"label\">Breathing</div>\n"
"      <div class=\"value\"><span id=\"breath\">--</span><span class=\"unit\">/min</span></div>\n"
"    </div>\n"
"    <div class=\"card\">\n"
"      <div class=\"label\">Live log</div>\n"
"      <pre class=\"log\" id=\"log\"></pre>\n"
"    </div>\n"
"  </div>\n"
"<script>\n"
"const logEl = document.getElementById('log');\n"
"let lastLine = '';\n"
"async function tick() {\n"
"  try {\n"
"    const r = await fetch('/api.json?' + Date.now());\n"
"    const d = await r.json();\n"
"    document.getElementById('line').textContent = d.line;\n"
"    document.getElementById('pos').textContent = d.position;\n"
"    document.getElementById('breath').textContent = d.breath;\n"
"    const pill = document.getElementById('pill');\n"
"    if (d.apnea) { pill.textContent = 'APNEA'; pill.className = 'pill bad'; }\n"
"    else if (d.alert_pos || (d.position||'').indexOf('WARNING')>=0) {\n"
"      pill.textContent = 'CHECK POSITION'; pill.className = 'pill warn';\n"
"    } else { pill.textContent = 'OK'; pill.className = 'pill ok'; }\n"
"    if (d.line && d.line !== lastLine) {\n"
"      lastLine = d.line;\n"
"      logEl.textContent += d.line + '\\n';\n"
"      const lines = logEl.textContent.trim().split('\\n');\n"
"      if (lines.length > 30) logEl.textContent = lines.slice(-30).join('\\n') + '\\n';\n"
"      logEl.scrollTop = logEl.scrollHeight;\n"
"    }\n"
"  } catch (e) {\n"
"    document.getElementById('line').textContent = 'waiting for sensor…';\n"
"  }\n"
"}\n"
"tick(); setInterval(tick, 500);\n"
"</script>\n"
"</body>\n"
"</html>\n";

static void handle_client(int cfd) {
    char req[1024];
    ssize_t n = read(cfd, req, sizeof(req) - 1);
    if (n <= 0) {
        close(cfd);
        return;
    }
    req[n] = '\0';

    if (strstr(req, "GET /api.json") == req || strstr(req, "GET /api.json?")) {
        char json[512];
        build_json(json, sizeof(json));
        http_reply(cfd, "application/json; charset=utf-8", json);
    } else {
        http_reply(cfd, "text/html; charset=utf-8", PAGE_HTML);
    }
    close(cfd);
}

static void http_poll(int listen_fd) {
    for (;;) {
        struct sockaddr_in peer;
        socklen_t plen = sizeof(peer);
        int cfd = accept(listen_fd, (struct sockaddr *)&peer, &plen);
        if (cfd < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                break;
            break;
        }
        handle_client(cfd);
    }
}

int main(void) {
    g_fd = open(I2C_DEV, O_RDWR);
    if (g_fd < 0) {
        perror("open i2c");
        return 1;
    }

    int http_fd = http_listen(HTTP_PORT);
    if (http_fd < 0) {
        perror("http listen");
        close(g_fd);
        return 1;
    }

    // --- INITIALIZE OUTBOUND UDP SOCKET FOR FLASK LINK ---
    int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in flask_addr;
    memset(&flask_addr, 0, sizeof(flask_addr));
    flask_addr.sin_family = AF_INET;
    flask_addr.sin_port = htons(OUTBOUND_UDP_PORT);
    inet_pton(AF_INET, FLASK_SERVER_IP, &flask_addr.sin_addr);

    struct { i2c_sendrecv_t hdr; uint8_t data[1]; } who;
    memset(&who, 0, sizeof(who));
    who.hdr.slave.addr = IMU_ADDR;
    who.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    who.hdr.send_len = 1;
    who.hdr.recv_len = 1;
    who.hdr.stop = 1;
    who.data[0] = 0x0F;
    if (devctl(g_fd, DCMD_I2C_SENDRECV, &who, sizeof(who), NULL) != 0) {
        perror("WHO_AM_I");
        close(http_fd);
        close(g_fd);
        return 1;
    }

    printf("WHO_AM_I = 0x%02X (expect 0x6C)\n", who.data[0]);
    if (who.data[0] != 0x6C) {
        printf("IMU not responding.\n");
        close(http_fd);
        close(g_fd);
        return 1;
    }

    write_reg(IMU_ADDR, 0x10, 0x40);
    write_reg(IMU_ADDR, 0x11, 0x00);
    usleep(100000);

    const double dt = 1.0 / SAMPLE_HZ;
    breath_filter_t filt;
    breath_detector_t det;
    breath_filter_init(&filt);
    breath_detector_init(&det, now_sec());

    int consecutive_bad = 0;
    int apnea_latched = 0;
    int pos_latched = 0;

    printf("\nTinyVitals dashboard on THIS Pi\n");
    printf("  Open on your laptop browser:\n");
    printf("    http://<PI_IP>:%d\n", HTTP_PORT);
    printf("  Streaming raw angles to Flask Server at %s:%d\n", FLASK_SERVER_IP, OUTBOUND_UDP_PORT);
    printf("Ctrl+C to stop.\n\n");

    while (1) {
        double t = now_sec();

        int16_t ax_raw = read_axis(IMU_ADDR, 0x28);
        int16_t ay_raw = read_axis(IMU_ADDR, 0x2A);
        int16_t az_raw = read_axis(IMU_ADDR, 0x2C);

        double ax = ax_raw * ACCEL_SENS;
        double ay = ay_raw * ACCEL_SENS;
        double az = az_raw * ACCEL_SENS;

        position_t pos = classify_position(ax, ay, az);

        if (pos == POS_STOMACH || pos == POS_LEFT_SIDE || pos == POS_RIGHT_SIDE) {
            consecutive_bad++;
        } else {
            consecutive_bad = 0;
            pos_latched = 0;
            g_alert_pos = 0;
        }

        double breath_sig = breath_filter_update(&filt, ay, dt);
        int got = breath_detector_update(&det, breath_sig, t);

        double since_breath = t - det.last_breath_t;
        int apnea = (since_breath >= APNEA_SECONDS);

        g_breath_bpm = det.bpm;
        g_pos = (int)pos;
        g_apnea = apnea;
        g_y_sig = breath_sig;

        // --- COMPUTE AND SEND SPATIAL IMU ANGLES TO WEBAPP ---
        // Pitch/Roll computed directly from gravity components
        float pitch_deg = (float)(atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / M_PI);
        float roll_deg  = (float)(atan2(ay, az) * 180.0 / M_PI);
        float yaw_deg   = 0.0f; // Standard 6-axis accelerometers lack absolute heading anchors

        IMUTelemetry packet;
        packet.pitch = pitch_deg;
        packet.roll = roll_deg;
        packet.yaw = yaw_deg;

        sendto(udp_sock, &packet, sizeof(packet), 0, (struct sockaddr *)&flask_addr, sizeof(flask_addr));

        static double last_print = 0.0;
        if (t - last_print >= 0.5 || got) {
            last_print = t;
            printf("%s | breath %.0f/min | Pitch:%+5.1f Roll:%+5.1f%s\n",
                   position_name(pos), det.bpm, pitch_deg, roll_deg,
                   got ? "  *breath*" : "");
        }

        if (consecutive_bad >= POS_ALARM_SAMPLES && !pos_latched) {
            pos_latched = 1;
            g_alert_pos = 1;
            printf("\n*** ALERT: baby not on back ***\n\n");
        }

        if (apnea && !apnea_latched) {
            apnea_latched = 1;
            printf("\n*** ALERT: no breath ***\n\n");
        }
        if (!apnea)
            apnea_latched = 0;

        /* Serve any waiting browser requests without blocking the IMU loop */
        http_poll(http_fd);

        double elapsed = now_sec() - t;
        double sleep_s = dt - elapsed;
        if (sleep_s > 0.0)
            usleep((useconds_t)(sleep_s * 1e6));
    }

    close(udp_sock);
    close(http_fd);
    close(g_fd);
    return 0;
}