#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <hw/i2c.h>
#include <devctl.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>

#define I2C_DEV   "/dev/i2c1"
#define IMU_ADDR  0x6A

#define ACCEL_SENS   0.000061   // g per LSB, +/-2g range
#define GYRO_SENS    0.00875    // dps per LSB, +/-250dps range

int write_reg(const char *dev, uint8_t addr, uint8_t reg, uint8_t val) {
    int fd = open(dev, O_RDWR);
    if (fd < 0) return -1;
    struct { i2c_sendrecv_t hdr; uint8_t data[2]; } msg;
    memset(&msg, 0, sizeof(msg));
    msg.hdr.slave.addr = addr;
    msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg.hdr.send_len = 2;
    msg.hdr.recv_len = 0;
    msg.hdr.stop = 1;
    msg.data[0] = reg;
    msg.data[1] = val;
    int ret = devctl(fd, DCMD_I2C_SENDRECV, &msg, sizeof(msg), NULL);
    close(fd);
    return ret;
}

int16_t read_axis(const char *dev, uint8_t addr, uint8_t reg_low) {
    int fd = open(dev, O_RDWR);
    if (fd < 0) return 0;
    struct { i2c_sendrecv_t hdr; uint8_t data[2]; } msg;
    memset(&msg, 0, sizeof(msg));
    msg.hdr.slave.addr = addr;
    msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg.hdr.send_len = 1;
    msg.hdr.recv_len = 2;
    msg.hdr.stop = 1;
    msg.data[0] = reg_low;
    devctl(fd, DCMD_I2C_SENDRECV, &msg, sizeof(msg), NULL);
    close(fd);
    return (int16_t)((msg.data[1] << 8) | msg.data[0]);
}

typedef enum { POS_BACK, POS_STOMACH, POS_LEFT_SIDE, POS_RIGHT_SIDE, POS_UNKNOWN } position_t;

// Classify based on which axis gravity is dominantly aligned with.
// NOTE: assumes a specific sensor mounting orientation on the onesie/swaddle —
// you MUST calibrate these thresholds against your actual strap orientation
// before trusting the output. Lay baby (or a test object) in each position
// and read the raw ax/ay/az values first to confirm which axis corresponds
// to which real-world direction for YOUR mounting.
position_t classify_position(double ax, double ay, double az) {
    const double THRESH = 0.6; // g threshold for "dominant" axis

    if (az > THRESH)  return POS_BACK;      // sensor face-up == baby on back
    if (az < -THRESH) return POS_STOMACH;   // sensor face-down == baby on stomach
    if (ay > THRESH)  return POS_RIGHT_SIDE;
    if (ay < -THRESH) return POS_LEFT_SIDE;

    return POS_UNKNOWN; // transitional/ambiguous — don't alarm on noise
}

const char* position_name(position_t p) {
    switch (p) {
        case POS_BACK: return "BACK (safe)";
        case POS_STOMACH: return "STOMACH - WARNING";
        case POS_LEFT_SIDE: return "LEFT SIDE - caution";
        case POS_RIGHT_SIDE: return "RIGHT SIDE - caution";
        default: return "unknown/transitional";
    }
}

int main() {
    int fd = open(I2C_DEV, O_RDWR);
    if (fd < 0) { perror("open"); return 1; }

    struct { i2c_sendrecv_t hdr; uint8_t data[1]; } who;
    memset(&who, 0, sizeof(who));
    who.hdr.slave.addr = IMU_ADDR;
    who.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    who.hdr.send_len = 1;
    who.hdr.recv_len = 1;
    who.hdr.stop = 1;
    who.data[0] = 0x0F;
    devctl(fd, DCMD_I2C_SENDRECV, &who, sizeof(who), NULL);
    close(fd);

    printf("WHO_AM_I = 0x%02X (expect 0x6C)\n", who.data[0]);
    if (who.data[0] != 0x6C) {
        printf("IMU not responding correctly, check wiring/address.\n");
        return 1;
    }

    write_reg(I2C_DEV, IMU_ADDR, 0x10, 0x60); // CTRL1_XL
    write_reg(I2C_DEV, IMU_ADDR, 0x11, 0x60); // CTRL2_G
    usleep(100000);

    double dt = 0.2; // 5 readings/sec is plenty for orientation, not fast motion
    position_t last_pos = POS_UNKNOWN;
    int consecutive_bad = 0;
    const int ALARM_THRESHOLD = 5; // require 5 consistent readings (~1s) before alarming — avoids false alarms from brief transitional movement

    printf("\nMonitoring position (Ctrl+C to stop)...\n\n");

    while (1) {
        int16_t ax_raw = read_axis(I2C_DEV, IMU_ADDR, 0x28);
        int16_t ay_raw = read_axis(I2C_DEV, IMU_ADDR, 0x2A);
        int16_t az_raw = read_axis(I2C_DEV, IMU_ADDR, 0x2C);

        double ax = ax_raw * ACCEL_SENS;
        double ay = ay_raw * ACCEL_SENS;
        double az = az_raw * ACCEL_SENS;

        position_t pos = classify_position(ax, ay, az);

        if (pos == POS_STOMACH || pos == POS_LEFT_SIDE || pos == POS_RIGHT_SIDE) {
            consecutive_bad++;
        } else {
            consecutive_bad = 0;
        }

        printf("accel(g): X=%6.2f Y=%6.2f Z=%6.2f | position: %s\n",
               ax, ay, az, position_name(pos));

        if (consecutive_bad == ALARM_THRESHOLD) {
            printf("\n*** ALERT: baby not on back for sustained period — check now ***\n\n");
            // TODO: trigger actual alert here — buzzer via GPIO, LED, network push, etc.
        }

        last_pos = pos;
        usleep((useconds_t)(dt * 1000000));
    }

    return 0;
}
