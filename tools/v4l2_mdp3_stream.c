// v4l2_mdp3_stream.c
// Standalone MDP3 streaming test — proves a real format conversion through
// the MDP3 hardware (MM21 -> NV12, or NV12 -> NV12 identity).
//
// /dev/video0 (mtk-mdp3), one fd for everything:
//   1. S_FMT <infmt> on V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE  (device input)
//   2. S_FMT NV12 on V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE    (device output)
//   3. REQBUFS + QUERYBUF on both queues
//   4. mmap the planes
//   5. QBUF all buffers on both queues
//   6. fill every input frame with constants:
//        Y  = 0x11
//        UV = 0x22,0x33 interleaved (NV12 order)
//      Constants survive de-tiling: any byte permutation of a constant
//      plane is the same constant, so the output can be verified without
//      knowing the exact input tile layout.
//   7. STREAMON(output), DQBUF(capture) x NBUFS
//   8. verify each NV12 frame: Y == 0x11, UV alternates 0x22/0x33
//
// CRITICAL: for V4L2_MEMORY_MPLANEs the offset passed to mmap() is
// planes[i].m.mem_offset (per-plane offset within the driver's single
// contiguous DMA block), NOT planes[i].data_offset — which the vb2 layer
// leaves at 0 for mmap buffers. Using data_offset makes every plane
// collide at offset 0 and the capture-queue mmap fails with EINVAL.
//
// build: cc -O2 -Wall -Wextra -o v4l2_mdp3_stream v4l2_mdp3_stream.c
// run:   sudo ./v4l2_mdp3_stream [width height] [infmt]
//         infmt = mm21 (default) | nv12   (input format; cap is always NV12)
//         nv12 input tests the HW job path with an identity convert to
//         isolate MM21-specific issues from a broken job-completion path.

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

#define V4L2_PIX_FMT_MM21  v4l2_fourcc('M', 'M', '2', '1')

#define NBUFS 4
#define DQ_TIMEOUT_MS 5000

#define Y_VAL  0x11
#define U_VAL  0x22
#define V_VAL  0x33

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
			printf("  %s[%d].plane[%d]: mem_offset=%u len=%u\n",
			       q->name, i, j,
			       q->planes[j].m.mem_offset,
			       q->planes[j].length);
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
			printf("  mmap %s[%d].plane[%d] @off %zu len %zu -> %p\n",
			       q->name, i, j,
			       q->memoff[i][j],
			       q->len[i][j], q->map[i][j]);
		}
	}
	return 0;
}

/*
 * Fill input frames with constants. NV12 (1 plane): Y region then
 * interleaved UV. MM21 (2 planes): Y plane, then UV plane.
 */
static void fill_input(struct queue *q, int in_is_nv12)
{
	if (in_is_nv12) {
		for (int i = 0; i < q->nbufs; i++) {
			unsigned char *p = q->map[i][0];
			size_t ylen = (size_t)q->w * q->h;
			size_t j;

			for (j = 0; j < ylen; j++)
				p[j] = Y_VAL;
			for (j = ylen; j < q->len[i][0]; j++)
				p[j] = ((j - ylen) & 1) ? V_VAL : U_VAL;
		}
		return;
	}
	for (int i = 0; i < q->nbufs; i++) {
		memset(q->map[i][0], Y_VAL, q->len[i][0]);
		for (size_t j = 0; j < q->len[i][1]; j++)
			((unsigned char *)q->map[i][1])[j] =
				(j & 1) ? V_VAL : U_VAL;
	}
}

/* Capture is always single-plane NV12: Y region then interleaved UV. */
static int verify_capture(const struct queue *q, int i)
{
	const unsigned char *p = q->map[i][0];
	size_t ylen = (size_t)q->w * q->h;
	size_t total = q->len[i][0];
	size_t bad = 0, j;

	if (ylen > total)
		ylen = total;
	for (j = 0; j < ylen; j++)
		if (p[j] != Y_VAL)
			bad++;
	for (j = ylen; j < total; j++)
		if (p[j] != (((j - ylen) & 1) ? V_VAL : U_VAL))
			bad++;

	printf("  frame %d: %zu/%zu bad\n", i, bad, total);
	return bad == 0;
}

static int dqbuf(int fd, __u32 type, struct v4l2_plane *planes)
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
	return (int)b.index;
}

int main(int argc, char **argv)
{
	int fd, pass = 0, fail = 0;
	__u32 w = 1280, h = 720;
	__u32 in_fmt = V4L2_PIX_FMT_MM21;
	int in_is_nv12 = 0;
	struct v4l2_pix_format_mplane omp, cmp;
	struct queue outq = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			      .name = "out" };
	struct queue capq = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			      .name = "cap(NV12)" };

	if (argc > 2) {
		w = (unsigned)atoi(argv[1]);
		h = (unsigned)atoi(argv[2]);
	}
	if (argc > 3) {
		if (strcmp(argv[3], "nv12") == 0) {
			in_fmt = V4L2_PIX_FMT_NV12;
			in_is_nv12 = 1;
			outq.name = "out(NV12)";
		} else if (strcmp(argv[3], "mm21") == 0) {
			in_fmt = V4L2_PIX_FMT_MM21;
			outq.name = "out(MM21)";
		} else {
			fprintf(stderr, "unknown infmt '%s' (use mm21|nv12)\n",
				argv[3]);
			return 1;
		}
	}

	fd = open("/dev/video0", O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		fprintf(stderr, "open /dev/video0 failed: %s\n", strerror(errno));
		return 1;
	}
	printf("opened /dev/video0 (fd %d), streaming %ux%u, %d buffers\n\n",
	       fd, w, h, NBUFS);

	if (s_fmt(fd, outq.type, in_fmt, w, h, &omp)) {
		fprintf(stderr, "S_FMT out failed: %s\n", strerror(errno));
		return 1;
	}
	print_mp("S_FMT out: ", &omp);

	/* request the capture side at the negotiated input size — no scale */
	if (s_fmt(fd, capq.type, V4L2_PIX_FMT_NV12, omp.width, omp.height, &cmp)) {
		fprintf(stderr, "S_FMT NV12 cap failed: %s\n", strerror(errno));
		return 1;
	}
	print_mp("S_FMT cap: ", &cmp);

	if (cmp.num_planes != 1) {
		fprintf(stderr, "unexpected cap plane count %u\n", cmp.num_planes);
		return 1;
	}
	if (in_is_nv12 ? omp.num_planes != 1 : omp.num_planes != 2) {
		fprintf(stderr, "unexpected out plane count %u for %s\n",
			omp.num_planes, in_is_nv12 ? "NV12" : "MM21");
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

	fill_input(&outq, in_is_nv12);
	printf("filled %d input frames (Y=0x%02x UV=0x%02x/0x%02x)\n",
	       NBUFS, Y_VAL, U_VAL, V_VAL);

	if (ioctl(fd, VIDIOC_STREAMON, &outq.type) < 0) {
		fprintf(stderr, "STREAMON failed: %s\n", strerror(errno));
		return 1;
	}

	/*
	 * All NBUFS out frames are queued (filled) and all NBUFS cap frames
	 * are queued. The driver dequeues BOTH the src and dst buffer of
	 * each job (v4l2_m2m_buf_done in process_done), so we only DQBUF the
	 * capture frame and re-queue it.
	 */
	for (int f = 0; f < NBUFS; f++) {
		int idx = dqbuf(fd, capq.type, capq.planes);

		if (idx < 0) {
			fprintf(stderr, "DQBUF cap failed/timeout: %s\n",
				strerror(errno));
			break;
		}
		if (verify_capture(&capq, idx))
			pass++;
		else
			fail++;
		qbuf(fd, capq.type, capq.nplanes, capq.planes, idx);
	}

	if (ioctl(fd, VIDIOC_STREAMOFF, &outq.type) < 0)
		fprintf(stderr, "STREAMOFF failed: %s\n", strerror(errno));

	printf("\nresult: %d/%d frames verified, %d failed\n",
	       pass, NBUFS, fail);
	close(fd);
	return fail == 0 && pass == NBUFS ? 0 : 1;
}
