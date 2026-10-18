// v4l2_mdp3_diag.c
//
// Read-only MDP3 / GCE evidence collector for the MT8183. It gathers five
// kinds of evidence in one pass and prints a single report:
//
//   [1] Video nodes   - enumerate /dev/video*, QUERYCAP each (driver, card,
//                       caps). Identifies mtk-mdp3 vs vcodec vs jpeg nodes.
//   [2] Formats       - VIDIOC_ENUM_FMT on OUTPUT_MPLANE + CAPTURE_MPLANE for
//                       every node (or one node with --node). Read-only: no
//                       S_FMT / REQBUFS / streaming, so it never disturbs the
//                       bridge or any in-flight decode.
//   [3] GCE DT props  - parse /sys/firmware/devicetree/base/soc/* for
//                       mediatek,gce-client-reg / mediatek,gce-events and
//                       decode the little-endian u32 arrays
//                       ({subsys,client,reg_base,reg_len} / {event,irq}).
//   [4] IRQ counts    - parse /proc/interrupts for mtk_cmdq / mdp / gce lines
//                       (per-CPU counts, total, spurious). With --cmd it takes
//                       a before/after snapshot and prints the delta, which is
//                       the direct test of "does the GCE thread fire DONE when
//                       an MDP3 M2M job runs?".
//   [5] Kernel msgs   - read /dev/kmsg (non-blocking) and print lines matching
//                       mdp|gce|cmdq.
//
// Everything here is strictly read-only: QUERYCAP, ENUM_FMT, sysfs,
// /proc/interrupts, /proc/irq/*/spurious, /dev/kmsg. It does not open a
// video device for streaming, allocate buffers, or touch the decoder, so it
// is safe to run while the bridge is serving a live decode.
//
// build:  cc -O2 -Wall -Wextra -o v4l2_mdp3_diag v4l2_mdp3_diag.c
// run:    sudo ./v4l2_mdp3_diag                 # single-shot report
//         sudo ./v4l2_mdp3_diag --no-dmesg      # skip kernel-message section
//         sudo ./v4l2_mdp3_diag --node /dev/video0
//         # GCE hypothesis test: run an MDP3 M2M workload between two IRQ
//         # snapshots and see whether mtk_cmdq increments:
//         sudo ./v4l2_mdp3_diag --cmd 'sudo ./v4l2_mdp3_stream 1280 736 mm21'
//
// Needs root for /dev/kmsg and to see all /dev/video* nodes.

#define _GNU_SOURCE

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <linux/videodev2.h>

#define DT_SOC "/sys/firmware/devicetree/base/soc"
#define KMSG   "/dev/kmsg"

/* ------------------------------------------------------------------ */
/* small file helpers                                                */

static void *
read_file(const char *path, size_t *out_len)
{
	int fd = open(path, O_RDONLY | O_CLOEXEC);
	struct stat st;
	void *buf;

	if (fd < 0)
		return NULL;
	if (fstat(fd, &st) < 0) {
		close(fd);
		return NULL;
	}
	buf = malloc(st.st_size ? st.st_size : 1);
	if (!buf) {
		close(fd);
		return NULL;
	}
	{
		ssize_t n = read(fd, buf, st.st_size);

		close(fd);
		if (n < 0) {
			free(buf);
			return NULL;
		}
		if (out_len)
			*out_len = (size_t)n;
	}
	return buf;
}

static int
read_u32_array(const char *path, uint32_t *dst, int max)
{
	size_t len = 0;
	void *buf = read_file(path, &len);
	int n, i;

	if (!buf)
		return -1;
	n = (int)(len / sizeof(uint32_t));
	if (n > max)
		n = max;
	/* The device-tree blob stores u32 properties big-endian (network order)
	 * regardless of the host endianness. */
	for (i = 0; i < n; i++) {
		const unsigned char *p = (const unsigned char *)buf + i * 4;

		dst[i] = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
		         ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
	}
	free(buf);
	return n;
}

/* ------------------------------------------------------------------ */
/* [1] video nodes                                                    */

static int is_mdp3_node(const struct v4l2_capability *cap)
{
	return cap->driver[0] &&
	       (strcasestr((const char *)cap->driver, "mdp") ||
	        strcasestr((const char *)cap->card,     "mdp"));
}

static void
section_nodes(void)
{
	DIR *d = opendir("/dev");
	struct dirent *e;
	int count = 0;

	printf("\n[1] VIDEO NODES\n");
	printf("  %-14s %-22s %-28s %s\n", "device", "driver", "card", "caps");
	printf("  %-14s %-22s %-28s %s\n", "------", "------", "----", "----");
	if (!d) {
		printf("  (cannot open /dev)\n");
		return;
	}
	while ((e = readdir(d))) {
		char path[300], desc[64] = "";
		int fd;
		struct v4l2_capability cap;

		if (strncmp(e->d_name, "video", 5) != 0)
			continue;
		snprintf(path, sizeof(path), "/dev/%s", e->d_name);
		fd = open(path, O_RDONLY | O_CLOEXEC);
		if (fd < 0)
			continue;
		if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
			const char *tag = is_mdp3_node(&cap) ? "  <== MDP3" : "";

			snprintf(desc, sizeof(desc), "0x%08x", cap.capabilities);
			printf("  %-14s %-22s %-28s %s%s\n",
			       e->d_name, cap.driver, cap.card, desc, tag);
			count++;
		} else {
			printf("  %-14s (QUERYCAP: %s)\n",
			       e->d_name, strerror(errno));
		}
		close(fd);
	}
	closedir(d);
	if (count == 0)
		printf("  (no /dev/video* nodes found)\n");
}

/* ------------------------------------------------------------------ */
/* [2] formats (read-only ENUM_FMT)                                   */

static void
enum_fmts(int fd, __u32 type, const char *label)
{
	struct v4l2_fmtdesc fmt;
	int idx = 0, n = 0;

	memset(&fmt, 0, sizeof(fmt));
	fmt.type = type;
	while (ioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0) {
		printf("    %s[%d]: '%c%c%c%c'  %s\n", label, idx++,
		       (char)(fmt.pixelformat & 0xff),
		       (char)((fmt.pixelformat >> 8) & 0xff),
		       (char)((fmt.pixelformat >> 16) & 0xff),
		       (char)((fmt.pixelformat >> 24) & 0xff),
		       fmt.description);
		n++;
		fmt.index++;
	}
	if (n == 0)
		printf("    %s: (none)\n", label);
}

static void
section_formats(const char *only_node)
{
	DIR *d = opendir("/dev");
	struct dirent *e;

	printf("\n[2] FORMATS (VIDIOC_ENUM_FMT, read-only)\n");
	if (!d)
		return;
	while ((e = readdir(d))) {
		char path[300];
		int fd;

		if (strncmp(e->d_name, "video", 5) != 0)
			continue;
		if (only_node && strcmp(only_node, e->d_name) != 0)
			continue;
		snprintf(path, sizeof(path), "/dev/%s", e->d_name);
		fd = open(path, O_RDONLY | O_CLOEXEC);
		if (fd < 0)
			continue;
		printf("  /dev/%s:\n", e->d_name);
		enum_fmts(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,  "out");
		enum_fmts(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, "cap");
		close(fd);
	}
	closedir(d);
}

/* ------------------------------------------------------------------ */
/* [3] GCE device-tree properties                                     */

static const char *
gce_marker(const char *name)
{
	/* Mark the nodes that matter for an MDP3 MM21->NV12 de-tile: the
	 * tile/resize/rdma/wdma pipeline stages. */
	if (strstr(name, "mdp3") || strstr(name, "rsz")  ||
	    strstr(name, "rdma") || strstr(name, "wdma") ||
	    strstr(name, "ccorr")|| strstr(name, "disp"))
		return "*";
	return " ";
}

static void
print_gce_node(const char *dir)
{
	char path[560];
	uint32_t reg[8] = { 0 };
	uint32_t ev[8]  = { 0 };
	int nreg, nev, i;
	char compat[128] = "";

	snprintf(path, sizeof(path), "%s/compatible", dir);
	{
		size_t len = 0;
		void *buf = read_file(path, &len);

		if (buf && len) {
			size_t copy = len < sizeof(compat) - 1 ? len : sizeof(compat) - 1;

			memcpy(compat, buf, copy);
			compat[copy] = '\0';
			free(buf);
		}
	}

	snprintf(path, sizeof(path), "%s/mediatek,gce-client-reg", dir);
	nreg = read_u32_array(path, reg, 8);
	snprintf(path, sizeof(path), "%s/mediatek,gce-events", dir);
	nev = read_u32_array(path, ev, 8);

	printf("  [%s] %s\n", gce_marker(dir + (strlen(DT_SOC) + 1)),
	       dir + (strlen(DT_SOC) + 1));
	if (compat[0])
		printf("      compatible        : %s\n", compat);
	if (nreg >= 4)
		printf("      gce-client-reg    : subsys=0x%x client=%u reg_base=0x%x reg_len=0x%x\n",
		       reg[0], reg[1], reg[2], reg[3]);
	else if (nreg > 0)
		printf("      gce-client-reg    : ");
	if (nreg > 0) {
		for (i = 0; i < nreg; i++)
			printf("0x%08x ", reg[i]);
		printf("\n");
	}
	if (nev >= 2)
		printf("      gce-events        : event=%u irq=0x%x\n", ev[0], ev[1]);
	else if (nev > 0) {
		printf("      gce-events        : ");
		for (i = 0; i < nev; i++)
			printf("0x%08x ", ev[i]);
		printf("\n");
	}
}

static void
section_gce_dt(void)
{
	DIR *d = opendir(DT_SOC);
	struct dirent *e;

	printf("\n[3] GCE DEVICE-TREE PROPERTIES (%s)\n", DT_SOC);
	printf("  [%s] = MDP3/display pipeline stage\n", "*");
	if (!d) {
		printf("  (cannot open %s)\n", DT_SOC);
		return;
	}
	while ((e = readdir(d))) {
		char dir[512], probe[560];

		if (e->d_name[0] == '.')
			continue;
		snprintf(dir, sizeof(dir), "%s/%s", DT_SOC, e->d_name);
		snprintf(probe, sizeof(probe), "%s/mediatek,gce-client-reg", dir);
		if (access(probe, F_OK) == 0)
			print_gce_node(dir);
	}
	closedir(d);
}

/* ------------------------------------------------------------------ */
/* [4] interrupt counts                                               */

#define MAX_IRQS 64

struct irq_snap {
	int irq;
	char name[64];
	unsigned long long total;
};

static int
parse_irq_line(char *line, struct irq_snap *s)
{
	char *tok, *save = NULL, *first, *last = NULL;
	unsigned long long total = 0;
	int is_count;

	first = strtok_r(line, " \t\n", &save);
	if (!first || first[strlen(first) - 1] != ':')
		return 0;
	s->irq = atoi(first);

	/* Walk tokens: sum the purely-numeric per-CPU counts; the last token
	 * is the handler name. */
	tok = first;
	while (tok) {
		char *next = strtok_r(NULL, " \t\n", &save);

		if (next)
			last = next;
		is_count = (*tok != '\0');
		for (char *p = tok; *p; p++)
			if (*p < '0' || *p > '9') {
				is_count = 0;
				break;
			}
		if (is_count)
			total += strtoull(tok, NULL, 10);
		tok = next;
	}
	if (!last)
		return 0;
	snprintf(s->name, sizeof(s->name), "%s", last);
	s->total = total;
	return 1;
}

static int
snapshot_irqs(struct irq_snap *arr, int max)
{
	char *line = NULL;
	size_t cap = 0;
	int n = 0;
	FILE *f = fopen("/proc/interrupts", "re");

	if (!f)
		return -1;
	while (getline(&line, &cap, f) >= 0) {
		struct irq_snap s;

		if (!parse_irq_line(line, &s))
			continue;
		if (!strcasestr(s.name, "cmdq") &&
		    !strcasestr(s.name, "mdp")  &&
		    !strcasestr(s.name, "gce"))
			continue;
		if (n < max)
			arr[n++] = s;
	}
	free(line);
	fclose(f);
	return n;
}

static unsigned long long
read_spurious(int irq)
{
	char path[64], line[128] = "";
	FILE *f;

	snprintf(path, sizeof(path), "/proc/irq/%d/spurious", irq);
	f = fopen(path, "re");
	if (!f)
		return 0;
	if (fgets(line, sizeof(line), f)) {
		/* "count 123" */
		return (unsigned long long)atoll(strchr(line, 't') + 1);
	}
	fclose(f);
	return 0;
}

static void
print_irq_list(struct irq_snap *arr, int n, const char *label)
{
	int i;

	printf("  %s\n", label);
	for (i = 0; i < n; i++)
		printf("    IRQ %-4d %-16s total=%-12llu spurious=%llu\n",
		       arr[i].irq, arr[i].name, arr[i].total,
		       read_spurious(arr[i].irq));
	if (n == 0)
		printf("    (no mtk_cmdq / mdp / gce IRQs found)\n");
}

/* Run `cmd` via sh -c with stdout+stderr captured to tmpfd, so the child's
 * output does not interleave with the report. Returns the exit status. */
static int
run_cmd_captured(const char *cmd, int tmpfd)
{
	pid_t pid = fork();
	int status = 0, code = -1;

	if (pid < 0)
		return -1;
	if (pid == 0) {
		dup2(tmpfd, 1);
		dup2(tmpfd, 2);
		execlp("sh", "sh", "-c", cmd, (char *)NULL);
		_exit(127);
	}
	if (waitpid(pid, &status, 0) == pid)
		code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
	return code;
}

static void
print_file_indented(const char *path)
{
	char *line = NULL;
	size_t cap = 0;
	FILE *f = fopen(path, "re");

	if (!f)
		return;
	while (getline(&line, &cap, f) >= 0) {
		char *nl = strchr(line, '\n');

		if (nl)
			*nl = '\0';
		printf("      %s\n", line);
	}
	free(line);
	fclose(f);
}

static void
section_irqs(const char *cmd)
{
	struct irq_snap before[MAX_IRQS], after[MAX_IRQS];
	int nb, na, i, j;

	printf("\n[4] INTERRUPT COUNTS (/proc/interrupts)\n");
	nb = snapshot_irqs(before, MAX_IRQS);
	print_irq_list(before, nb, "  BEFORE:");

	if (cmd) {
		char tmpname[64];
		int tmpfd, code;

		printf("  RUNNING: %s\n", cmd);
		snprintf(tmpname, sizeof(tmpname), "/tmp/v4l2mdp3diag.XXXXXX");
		tmpfd = mkstemp(tmpname);
		if (tmpfd < 0) {
			printf("  (mkstemp failed: %s)\n", strerror(errno));
		} else {
			code = run_cmd_captured(cmd, tmpfd);
			close(tmpfd);
			printf("  (exit status %d) — output:\n", code);
			print_file_indented(tmpname);
			unlink(tmpname);
		}
		na = snapshot_irqs(after, MAX_IRQS);
		print_irq_list(after, na, "  AFTER:");

		printf("  DELTA (after - before):\n");
		for (i = 0; i < na; i++) {
			unsigned long long b = 0;

			for (j = 0; j < nb; j++)
				if (before[j].irq == after[i].irq) {
					b = before[j].total;
					break;
				}
			printf("    IRQ %-4d %-16s %+lld\n",
			       after[i].irq, after[i].name,
			       (long long)(after[i].total - b));
		}
	}
}

/* ------------------------------------------------------------------ */
/* [5] kernel messages                                                */

/* Keep short driver log lines; drop oops/backtrace context (module dumps,
 * CPU/Comm/PID headers, call traces) and anything the tool itself logged. */
static int
kmsg_want(const char *s)
{
	if (!strcasestr(s, "mdp") && !strcasestr(s, "gce") &&
	    !strcasestr(s, "cmdq"))
		return 0;
	if (strcasestr(s, "Modules linked in"))
		return 0;
	if (strcasestr(s, "Call trace"))
		return 0;
	if (strcasestr(s, "Tainted"))
		return 0;
	if (strncmp(s, "CPU:", 4) == 0)
		return 0;
	if (strlen(s) > 200)
		return 0;
	return 1;
}

static void
kmsg_line(char *buf, size_t len, int *shown, int *matched, int cap)
{
	/* Strip the "pri,seq,usec,flags;" prefix. */
	char *semi = memchr(buf, ';', len);

	if (semi) {
		len = strlen(semi + 1);
		memcpy(buf, semi + 1, len + 1);
	}
	if (kmsg_want(buf)) {
		(*matched)++;
		if (*shown < cap) {
			printf("    %s\n", buf);
			(*shown)++;
		}
	}
}

static void
section_kmsg(void)
{
	int fd = open(KMSG, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	char buf[4096];
	long tot = 0;
	int shown = 0, matched = 0;
	const int cap = 200;

	printf("\n[5] KERNEL MESSAGES (/dev/kmsg, filtered mdp|gce|cmdq)\n");
	if (fd < 0) {
		printf("    (cannot open %s: %s — run as root)\n",
		       KMSG, strerror(errno));
		return;
	}
	for (;;) {
		ssize_t n = read(fd, buf, sizeof(buf) - 1);

		if (n <= 0)
			break;
		buf[n] = '\0';
		tot += n;
		/* A read may span several log records; process whole lines. */
		for (char *s = buf; *s; ) {
			char *nl = strchr(s, '\n');

			if (nl)
				*nl = '\0';
			kmsg_line(s, strlen(s), &shown, &matched, cap);
			if (!nl)
				break;
			s = nl + 1;
		}
		if (tot > 8 * 1024 * 1024)
			break;
	}
	close(fd);
	printf("  (matched %d lines, showed %d of them, %ld bytes read)\n",
	       matched, shown, tot);
}

/* ------------------------------------------------------------------ */

int
main(int argc, char **argv)
{
	const char *cmd = NULL;
	const char *only_node = NULL;
	int do_dmesg = 1;
	int i;
	time_t now;

	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--cmd") == 0 && i + 1 < argc) {
			cmd = argv[++i];
		} else if (strcmp(argv[i], "--node") == 0 && i + 1 < argc) {
			only_node = argv[++i];
		} else if (strcmp(argv[i], "--no-dmesg") == 0) {
			do_dmesg = 0;
		} else if (strcmp(argv[i], "-h") == 0 ||
			   strcmp(argv[i], "--help") == 0) {
			fprintf(stderr,
				"usage: %s [--cmd '<shell>'] [--node <dev>]\n"
				"                [--no-dmesg]\n",
				argv[0]);
			return 0;
		} else {
			fprintf(stderr, "unknown arg: %s\n", argv[i]);
			return 2;
		}
	}

	printf("== v4l2_mdp3_diag — read-only MDP3/GCE evidence ==\n");
	time(&now);
	printf("time: %suid: %d\n", ctime(&now), (int)getuid());

	section_nodes();
	section_formats(only_node);
	section_gce_dt();
	section_irqs(cmd);
	if (do_dmesg)
		section_kmsg();

	printf("\n== end of report ==\n");
	return 0;
}
