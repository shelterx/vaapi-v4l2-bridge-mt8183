// v4l2_fmt_test.c
// Small V4L2 multiplane format test for MDP3 (/dev/video0):
//   1. open /dev/video0 once — every op shares this single fd
//   2. VIDIOC_QUERYCAP
//   3. VIDIOC_S_FMT  MM21 on V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
//   4. VIDIOC_S_FMT  NV12 on V4L2_BUF_TYPE_CAPTURE_MPLANE
//   5. VIDIOC_G_FMT  on both queues (same fd)
//   6. print ret, errno, FourCC, dimensions, plane count, per-plane
//      bytesperline / sizeimage
//
// Both queues are multiplanar, so the v4l2_format union member is
// struct v4l2_pix_format_mplane in every kernel era — the per-plane
// data lives in pix_mp.plane_fmt[] and num_planes is at the end.
// (The single-plane struct v4l2_pix_format must NOT be used to parse
// MPLANE results: its plane_count/sizeimage offsets land inside
// plane_fmt[] and produce garbage.)
//
// build: cc -O2 -Wall -Wextra -o v4l2_fmt_test v4l2_fmt_test.c
// run:   sudo ./v4l2_fmt_test [width height]   (default 1920x1080)

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/videodev2.h>

/* MediaTek tiled 8-bit block format — vendor fourcc, not in mainline */
#define V4L2_PIX_FMT_MM21  v4l2_fourcc('M', 'M', '2', '1')

struct fmt_report {
	const char *op;
	int ret;
	int err;
	struct v4l2_pix_format_mplane mp;
};

static void print_fourcc(__u32 f)
{
	char s[5] = { 0 };

	for (int i = 0; i < 4; i++)
		s[i] = (char)((f >> (8 * i)) & 0xff);
	printf("0x%08X ('%s')", f, s);
}

static void print_fmt(const struct fmt_report *rp)
{
	const struct v4l2_pix_format_mplane *mp = &rp->mp;

	if (rp->ret < 0) {
		printf("  ret=%d  errno=%d (%s)\n", rp->ret, rp->err,
		       strerror(rp->err));
		return;
	}
	printf("  ret=0\n");
	printf("  fourcc    : ");
	print_fourcc(mp->pixelformat);
	printf("\n");
	printf("  size      : %ux%u\n", mp->width, mp->height);
	printf("  field     : %u\n", mp->field);
	printf("  planes    : %u\n", mp->num_planes);
	for (int i = 0; i < mp->num_planes && i < VIDEO_MAX_PLANES; i++)
		printf("  plane[%d]: bytesperline=%u sizeimage=%u\n", i,
		       mp->plane_fmt[i].bytesperline,
		       mp->plane_fmt[i].sizeimage);
}

static void do_s_fmt(struct fmt_report *rp, int fd, __u32 type,
		     __u32 fourcc, __u32 w, __u32 h)
{
	struct v4l2_format fmt;

	memset(&fmt, 0, sizeof(fmt));
	fmt.type = type;
	fmt.fmt.pix_mp.width = w;
	fmt.fmt.pix_mp.height = h;
	fmt.fmt.pix_mp.pixelformat = fourcc;
	fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;

	rp->ret = ioctl(fd, VIDIOC_S_FMT, &fmt);
	rp->err = rp->ret < 0 ? errno : 0;
	if (rp->ret == 0)
		rp->mp = fmt.fmt.pix_mp; /* negotiated format written back */
}

static void do_g_fmt(struct fmt_report *rp, int fd, __u32 type)
{
	struct v4l2_format fmt;

	memset(&fmt, 0, sizeof(fmt));
	fmt.type = type;

	rp->ret = ioctl(fd, VIDIOC_G_FMT, &fmt);
	rp->err = rp->ret < 0 ? errno : 0;
	if (rp->ret == 0)
		rp->mp = fmt.fmt.pix_mp;
}

int main(int argc, char **argv)
{
	int fd;
	__u32 w = 1920, h = 1080;
	struct v4l2_capability cap;
	struct fmt_report r[4] = {
		{ .op = "VIDIOC_S_FMT  MM21  V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE",
		  .ret = -1 },
		{ .op = "VIDIOC_S_FMT  NV12  V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE",
		  .ret = -1 },
		{ .op = "VIDIOC_G_FMT  V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE",
		  .ret = -1 },
		{ .op = "VIDIOC_G_FMT  V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE",
		  .ret = -1 },
	};

	if (argc > 2) {
		w = (unsigned)atoi(argv[1]);
		h = (unsigned)atoi(argv[2]);
	}

	fd = open("/dev/video0", O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		fprintf(stderr, "open /dev/video0 failed: %s\n", strerror(errno));
		return 1;
	}
	printf("opened /dev/video0 (fd %d), testing %ux%u\n\n", fd, w, h);

	memset(&cap, 0, sizeof(cap));
	if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
		printf("== VIDIOC_QUERYCAP ==\n");
		printf("  driver    : %s\n", cap.driver);
		printf("  card      : %s\n", cap.card);
		printf("  bus_info  : %s\n", cap.bus_info);
		printf("  cap flags : 0x%08x\n", cap.capabilities);
		printf("\n");
	} else {
		printf("VIDIOC_QUERYCAP failed: %s\n\n", strerror(errno));
	}

	do_s_fmt(&r[0], fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		 V4L2_PIX_FMT_MM21, w, h);
	do_s_fmt(&r[1], fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		 V4L2_PIX_FMT_NV12, w, h);
	do_g_fmt(&r[2], fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	do_g_fmt(&r[3], fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	for (int i = 0; i < 4; i++) {
		printf("== %s ==\n", r[i].op);
		print_fmt(&r[i]);
		printf("\n");
	}

	close(fd);
	return 0;
}
