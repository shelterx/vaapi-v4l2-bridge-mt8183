// v4l2_m2m_jpeg.c
// Standalone JPEG-encoder M2M test — a *differential* M2M repro.
//
// Original purpose was to exercise the GCE (CMDQ) through a M2M client *other
// than* MDP3. THAT PREMISE IS RETRACTED: /dev/video1 (mtk-jpeg-enc) is bound
// to the upstream mtk_jpeg driver, which completes jobs via a DIRECT hardware
// IRQ (platform_get_irq -> mtk_jpeg_enc_irq -> JPEG_ENC_INT_STS DONE bit), NOT
// GCE/cmdq — there are zero gce/cmdq/mailbox references in
// drivers/media/platform/mediatek/jpeg/. See docs/mt8183-mdp3.md ("Differential
// test: JPEG-enc M2M also hangs — but it does NOT use GCE").
//
// So this is NOT a GCE probe. Its value: an independent M2M repro showing a
// second, INDEPENDENT (JPEG-specific) hang. It also hangs (0/4, DQBUF timeout
// ~5 s), consistent with the MDP3 M2M job (tools/v4l2_mdp3_stream.c), but the
// two do NOT share a completion path — so "both hang" does NOT imply a shared
// M2M/GCE bug. This tool says nothing about the MDP3/GCE path.
//
// /dev/video1 (mtk-jpeg-enc), one fd for everything:
//   1. S_FMT YUYV  on V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE  (device input)
//   2. S_FMT JPEG  on V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE (device output)
//   3. REQBUFS + QUERYBUF on both queues
//   4. mmap the planes
//   5. QBUF all buffers on both queues
//   6. fill every input frame with a pseudo-random pattern
//   7. STREAMON(output), DQBUF(capture) x NBUFS
//   8. verify each JPEG: SOI (FF D8) at byte 0, EOI (FF D9) at the last byte
//      of the used region (b.bytesused)
//
// The input pixel content is irrelevant — this test only asks "does the M2M
// job complete and emit a well-formed JPEG?". A solid/noisy YUYV frame
// compresses to a small but valid JPEG (SOI...EOI).
//
// CRITICAL (same as v4l2_mdp3_stream.c): for V4L2_MEMORY_MMAP the mmap()
// offset is planes[i].m.mem_offset (per-plane offset within the driver's
// single contiguous DMA block), NOT planes[i].data_offset (0 for mmap bufs).
//
// build: cc -O2 -Wall -Wextra -o v4l2_m2m_jpeg v4l2_m2m_jpeg.c
// run:   sudo ./v4l2_m2m_jpeg [width height]

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/videodev2.h>

#define V4L2_PIX_FMT_JPEG v4l2_fourcc('J', 'P', 'E', 'G')

#define NBUFS 4
#define DQ_TIMEOUT_MS 5000

struct queue {
	__u32 type;
	const char *name;
	int nbufs;
	int nplanes;
	__u32 w, h;
	void *map[NBUFS][VIDEO_MAX_PLANES];
	size_t len[NBUFS][VIDEO_MAX_PLANES];
	size_t memoff[NBUFS][VIDEO_MAX_PLANES];
	struct v4l2_plane planes[VIDEO_MAX_PLANES];
};

static int s_fmt(int fd, __u32 type, __u32 fourcc, __u32 w, __u32 h,
		 struct v4l2_pix_format_mplane *out_mp)
{
	struct v4l2_format fmt;
	int ret;

	memset(&fmt, 0, sizeof(fmt));
	fmt.type = type;
	fmt.fmt.pix_mp.width = w;
	fmt.fmt.pix_mp.height = h;
	fmt.fmt.pix_mp.pixelformat = fourcc;
	fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;

	ret = ioctl(fd, VIDIOC_S_FMT, &fmt);
	if (ret < 0)
		return ret;
	*out_mp = fmt.fmt.pix_mp;
	return 0;
}

static void print_mp(const char *label, const struct v4l2_pix_format_mplane *mp)
{
	char s[5] = { 0 };
	__u32 f = mp->pixelformat;

	for (int i = 0; i < 4; i++)
		s[i] = (char)((f >> (8 * i)) & 0xff);
	printf("%s: %ux%u planes=%u ('%s')\n", label, mp->width, mp->height,
	       mp->num_planes, s);
	for (int i = 0; i < mp->num_planes && i < VIDEO_MAX_PLANES; i++)
		printf("  plane[%d]: bytesperline=%u sizeimage=%u\n", i,
		       mp->plane_fmt[i].bytesperline,
		       mp->plane_fmt[i].sizeimage);
}

static int reqbufs(int fd, __u32 type, int count)
{
	struct v4l2_requestbuffers rb = {
		.count = (unsigned)count,
		.type = type,
		.memory = V4L2_MEMORY_MMAP,
	};

	if (ioctl(fd, VIDIOC_REQBUFS, &rb) < 0)
		return -1;
	return (int)rb.count;
}

/* QUERYBUF every buffer; stores per-plane mem_offset/length in q->planes. */
static int query_queue(int fd, struct queue *q,
		       const struct v4l2_pix_format_mplane *mp)
{
	q->nbufs = NBUFS;
	q->nplanes = mp->num_planes;
	q->w = mp->width;
	q->h = mp->height;

	for (int i = 0; i < q->nbufs; i++) {
		struct v4l2_buffer b;

		memset(&b, 0, sizeof(b));
		b.type = q->type;
		b.index = (unsigned)i;
		b.memory = V4L2_MEMORY_MMAP; /* required for QUERYBUF */
		b.length = (unsigned)q->nplanes; /* multiplanar: count */
		b.m.planes = q->planes;
		if (ioctl(fd, VIDIOC_QUERYBUF, &b) < 0) {
			fprintf(stderr, "QUERYBUF %s[%d] failed: %s\n",
				q->name, i, strerror(errno));
			return -1;
		}
		for (int j = 0; j < q->nplanes; j++) {
			q->len[i][j] = q->planes[j].length;
			q->memoff[i][j] = q->planes[j].m.mem_offset;
		}
	}
	return 0;
}

static int qbuf(int fd, __u32 type, int nplanes, struct v4l2_plane *planes,
		int idx)
{
	struct v4l2_buffer b;

	memset(&b, 0, sizeof(b));
	b.type = type;
	b.index = (unsigned)idx;
	b.memory = V4L2_MEMORY_MMAP;
	b.length = (unsigned)nplanes; /* multiplanar: plane count */
	b.m.planes = planes;
	if (ioctl(fd, VIDIOC_QBUF, &b) < 0)
		return -1;
	return 0;
}

static int mmap_queue(int fd, struct queue *q)
{
	for (int i = 0; i < q->nbufs; i++) {
		for (int j = 0; j < q->nplanes; j++) {
			q->map[i][j] = mmap(0, q->len[i][j],
					    PROT_READ | PROT_WRITE,
					    MAP_SHARED, fd,
					    q->memoff[i][j]);
			if (q->map[i][j] == MAP_FAILED) {
				fprintf(stderr,
					"mmap %s[%d].plane[%d] failed: %s\n",
					q->name, i, j, strerror(errno));
				return -1;
			}
		}
	}
	return 0;
}

/*
 * Fill the input (YUYV, single plane) with a pseudo-random pattern. Content
 * is irrelevant — we only need the M2M job to complete and emit a valid JPEG.
 */
static void fill_input(struct queue *q)
{
	for (int i = 0; i < q->nbufs; i++) {
		unsigned char *p = q->map[i][0];
		size_t j;

		for (j = 0; j < q->len[i][0]; j++)
			p[j] = (unsigned char)((j * 2654435761u) >> 13);
	}
}

/*
 * JPEG output check: SOI (FF D8) at byte 0 and EOI (FF D9) at the last byte
 * of the used region. bytesused is the driver-reported actual JPEG length
 * (the buffer itself is a max-size allocation).
 */
static int verify_jpeg(const struct queue *q, int i, unsigned bytesused)
{
	const unsigned char *p = q->map[i][0];
	int soi, eoi, ok;

	if (bytesused < 4) {
		printf("  frame %d: bytesused=%u too small\n", i, bytesused);
		return 0;
	}
	soi = (p[0] == 0xff && p[1] == 0xd8);
	eoi = (p[bytesused - 2] == 0xff && p[bytesused - 1] == 0xd9);
	ok = soi && eoi;
	printf("  frame %d: bytesused=%u soi=%d eoi=%d -> %s\n",
	       i, bytesused, soi, eoi, ok ? "OK" : "BAD");
	return ok;
}

/* DQBUF; returns index and (if bytesused != NULL) the used byte count. */
static int dqbuf(int fd, __u32 type, struct v4l2_plane *planes,
		 unsigned *bytesused)
{
	struct v4l2_buffer b;
	struct pollfd pfd = { .fd = fd, .events = POLLIN };

	if (poll(&pfd, 1, DQ_TIMEOUT_MS) <= 0)
		return -1;
	memset(&b, 0, sizeof(b));
	b.type = type;
	b.m.planes = planes;
	if (ioctl(fd, VIDIOC_DQBUF, &b) < 0)
		return -1;
	if (bytesused)
		*bytesused = b.bytesused;
	return (int)b.index;
}

int main(int argc, char **argv)
{
	int fd, pass = 0, fail = 0;
	__u32 w = 1280, h = 720;
	struct v4l2_pix_format_mplane omp, cmp;
	struct queue outq = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			      .name = "out(YUYV)" };
	struct queue capq = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			      .name = "cap(JPEG)" };

	if (argc > 2) {
		w = (unsigned)atoi(argv[1]);
		h = (unsigned)atoi(argv[2]);
	}

	fd = open("/dev/video1", O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		fprintf(stderr, "open /dev/video1 failed: %s\n", strerror(errno));
		return 1;
	}
	printf("opened /dev/video1 (fd %d), streaming %ux%u, %d buffers\n\n",
	       fd, w, h, NBUFS);

	if (s_fmt(fd, outq.type, V4L2_PIX_FMT_YUYV, w, h, &omp)) {
		fprintf(stderr, "S_FMT out(YUYV) failed: %s\n", strerror(errno));
		return 1;
	}
	print_mp("S_FMT out: ", &omp);

	if (s_fmt(fd, capq.type, V4L2_PIX_FMT_JPEG, omp.width, omp.height, &cmp)) {
		fprintf(stderr, "S_FMT cap(JPEG) failed: %s\n", strerror(errno));
		return 1;
	}
	print_mp("S_FMT cap: ", &cmp);

	if (cmp.num_planes != 1) {
		fprintf(stderr, "unexpected cap plane count %u\n", cmp.num_planes);
		return 1;
	}
	if (omp.num_planes != 1) {
		fprintf(stderr, "unexpected out plane count %u for YUYV\n",
			omp.num_planes);
		return 1;
	}

	if (reqbufs(fd, outq.type, NBUFS) < 0) {
		fprintf(stderr, "REQBUFS out failed: %s\n", strerror(errno));
		return 1;
	}
	if (reqbufs(fd, capq.type, NBUFS) < 0) {
		fprintf(stderr, "REQBUFS cap failed: %s\n", strerror(errno));
		return 1;
	}

	if (query_queue(fd, &outq, &omp) || query_queue(fd, &capq, &cmp))
		return 1;

	if (mmap_queue(fd, &outq) || mmap_queue(fd, &capq))
		return 1;

	/* QBUF after mmap (canonical V4L2 order) */
	for (int i = 0; i < NBUFS; i++) {
		if (qbuf(fd, outq.type, outq.nplanes, outq.planes, i))
			fprintf(stderr, "QBUF out[%d] failed: %s\n", i,
				strerror(errno));
		if (qbuf(fd, capq.type, capq.nplanes, capq.planes, i))
			fprintf(stderr, "QBUF cap[%d] failed: %s\n", i,
				strerror(errno));
	}

	fill_input(&outq);
	printf("filled %d input frames (YUYV pseudo-random)\n\n", NBUFS);

	if (ioctl(fd, VIDIOC_STREAMON, &outq.type) < 0) {
		fprintf(stderr, "STREAMON failed: %s\n", strerror(errno));
		return 1;
	}

	/*
	 * All NBUFS out frames are queued (filled) and all NBUFS cap frames are
	 * queued. The driver dequeues both the src and dst buffer of each job,
	 * so we only DQBUF the capture (JPEG) frame and re-queue it.
	 */
	for (int f = 0; f < NBUFS; f++) {
		int idx;
		unsigned bytesused = 0;

		idx = dqbuf(fd, capq.type, capq.planes, &bytesused);
		if (idx < 0) {
			fprintf(stderr, "DQBUF cap failed/timeout: %s\n",
				strerror(errno));
			break;
		}
		if (verify_jpeg(&capq, idx, bytesused))
			pass++;
		else
			fail++;
		qbuf(fd, capq.type, capq.nplanes, capq.planes, idx);
	}

	if (ioctl(fd, VIDIOC_STREAMOFF, &outq.type) < 0)
		fprintf(stderr, "STREAMOFF failed: %s\n", strerror(errno));

	printf("\nresult: %d/%d JPEGs valid, %d failed\n",
	       pass, NBUFS, fail);
	close(fd);
	return fail == 0 && pass == NBUFS ? 0 : 1;
}
