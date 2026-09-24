/*
 * rkaiq-stream-resync.c: re-apply the sensor's exposure controls after every ISP stream start.
 *
 * Why: the vendor imx219 driver rewrites the sensor's exposure and gain registers from its mode table
 * in s_stream (Armbian linux-rockchip issue 452) but does not re-apply the V4L2 control values, so the
 * kernel's control cache and the sensor disagree after every stream restart. rkaiq_3A_server writes
 * exposure every frame, but the V4L2 core only calls the driver when a value changes, and a converged
 * auto exposure at its 30 ms cap never changes. Result: the second stream after boot is 3x darker than
 * the first and the engine spirals to maximum gain. Mainline fixes this in the driver with
 * __v4l2_ctrl_handler_setup(); until this kernel does, this helper nudges the cached values through the
 * driver (value-1, then value) shortly after each stream start.
 *
 * Usage: rkaiq-stream-resync <rkisp-input-params node> <sensor subdev node>
 *        e.g. rkaiq-stream-resync /dev/video18 /dev/v4l-subdev2
 * Started by microduck-rkaiq.sh next to the engine. Needs read access to both nodes (root, or the video
 * group). Build: gcc -O2 -o rkaiq-stream-resync rkaiq-stream-resync.c
 */
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

/* From the vendor kernel's rk-isp-config.h (not shipped in the Armbian headers). */
#ifndef RKISP_V4L2_EVENT_STREAM_START
#define RKISP_V4L2_EVENT_STREAM_START (V4L2_EVENT_PRIVATE_START + 1)
#define RKISP_V4L2_EVENT_STREAM_STOP  (V4L2_EVENT_PRIVATE_START + 2)
#endif

static const unsigned int cids[] = { V4L2_CID_EXPOSURE, V4L2_CID_ANALOGUE_GAIN };

static void msleep(int ms) { struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L }; nanosleep(&ts, NULL); }

static void nudge(int sd)
{
    for (size_t i = 0; i < sizeof cids / sizeof cids[0]; i++) {
        struct v4l2_control c = { .id = cids[i] };
        struct v4l2_queryctrl q = { .id = cids[i] };
        if (ioctl(sd, VIDIOC_QUERYCTRL, &q) < 0 || ioctl(sd, VIDIOC_G_CTRL, &c) < 0)
            continue;
        int want = c.value;
        int other = want > q.minimum ? want - 1 : want + 1;
        c.value = other; ioctl(sd, VIDIOC_S_CTRL, &c);
        c.value = want;  ioctl(sd, VIDIOC_S_CTRL, &c);
        printf("resync: control 0x%08x re-applied at %d\n", cids[i], want);
    }
    fflush(stdout);
}

int main(int argc, char **argv)
{
    if (argc != 3) { fprintf(stderr, "usage: %s <rkisp-input-params node> <sensor subdev>\n", argv[0]); return 2; }
    int ev = open(argv[1], O_RDWR | O_NONBLOCK);
    int sd = open(argv[2], O_RDWR);
    if (ev < 0 || sd < 0) { perror("open"); return 1; }

    struct v4l2_event_subscription sub = { .type = RKISP_V4L2_EVENT_STREAM_START };
    if (ioctl(ev, VIDIOC_SUBSCRIBE_EVENT, &sub) < 0) { perror("subscribe stream start"); return 1; }
    sub.type = RKISP_V4L2_EVENT_STREAM_STOP;
    ioctl(ev, VIDIOC_SUBSCRIBE_EVENT, &sub);
    printf("resync: watching %s for ISP stream starts, sensor %s\n", argv[1], argv[2]);
    fflush(stdout);

    for (;;) {
        struct pollfd p = { .fd = ev, .events = POLLPRI };
        if (poll(&p, 1, -1) < 0) { if (errno == EINTR) continue; perror("poll"); return 1; }
        struct v4l2_event e;
        while (ioctl(ev, VIDIOC_DQEVENT, &e) == 0) {
            if (e.type != RKISP_V4L2_EVENT_STREAM_START) continue;
            /* The sensor's s_stream runs a little after the ISP announces the start; the mode table
             * write must land before the nudge or the nudge is what gets overwritten. Two passes. */
            msleep(300); nudge(sd);
            msleep(700); nudge(sd);
        }
    }
}
