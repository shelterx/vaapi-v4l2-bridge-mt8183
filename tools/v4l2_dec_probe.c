// v4l2_dec_probe.c
// Enumerate OUTPUT + CAPTURE formats on a decoder node (default /dev/video5).
// Uses VIDIOC_ENUM_FMT on both MPLANE queue types — the reliable way to see
// what the driver advertises (v4l2-ctl --list-formats can miss the OUTPUT
// side on some stateful/stateless decoders).
//
// build: cc -O2 -Wall -Wextra -o v4l2_dec_probe v4l2_dec_probe.c
// run:   sudo ./v4l2_dec_probe [/dev/videoN]

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/videodev2.h>

static void print_fourcc(__u32 f)
{
	char s[5] = { 0 };

	for (int i = 0; i < 4; i++)
		s[i] = (char)((f >> (8 * i)) & 0xff);
	printf("'%s'", s);
}

static void enum_fmts(int fd, __u32 type, const char *label)
{
	struct v4l2_fmtdesc fmt;
	int idx = 0, n = 0;

	printf("== %s formats ==\n", label);
	memset(&fmt, 0, sizeof(fmt));
	fmt.type = type;
	while (ioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0) {
		printf("  [%d]: ", idx++);
		print_fourcc(fmt.pixelformat);
		printf("  %s\n", fmt.description);
		n++;
		fmt.index++;
	}
	if (n == 0)
		printf("  (none enumerated)\n");
	printf("\n");
}

int main(int argc, char **argv)
{
	const char *path = argc > 1 ? argv[1] : "/dev/video5";
	int fd;
	struct v4l2_capability cap;

	fd = open(path, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		fprintf(stderr, "open %s failed: %s\n", path, strerror(errno));
		return 1;
	}
	printf("opened %s (fd %d)\n\n", path, fd);

	memset(&cap, 0, sizeof(cap));
	if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
		printf("  driver  : %s\n", cap.driver);
		printf("  card    : %s\n", cap.card);
		printf("  caps    : 0x%08x\n\n", cap.capabilities);
	}

	enum_fmts(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, "OUTPUT");
	enum_fmts(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, "CAPTURE");

	close(fd);
	return 0;
}
