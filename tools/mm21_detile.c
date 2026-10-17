// mm21_detile.c
// De-tile MediaTek MM21 (tiled 8-bit) -> linear NV12, then RGB, for verification.
//
// MM21 layout (authoritative: Linux kernel pixfmt-yuv-planar + libyuv
// DetilePlane / MM21ToNV12):
//   Y  plane : 16x32 tiles (16 wide x 32 tall), row-major tile order.
//              tile = 16*32 = 512 bytes.
//   UV plane : SAME tile grid as luma (aw/16 x ah/32). Each tile is
//              8x16 CHROMA samples (U,V interleaved = NV12), 8*16*2 = 256
//              bytes, one tile per 16x32 luma tile. (The kernel doc says
//              "the number of luma and chroma tiles are identical".)
//   width aligned to 16, height aligned to 32. Two non-contiguous planes.
//
// build: cc -O2 -Wall -Wextra -o mm21_detile mm21_detile.c
// run:   ./mm21_detile <in.mm21> <w> <h> <frame> <out.ppm>
//   e.g. ./mm21_detile bbb.raw 853 480 0 frame0.ppm

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
	if (argc != 6) {
		fprintf(stderr,
			"usage: %s <in.mm21> <w> <h> <frame> <out.ppm>\n", argv[0]);
		return 1;
	}
	int w = atoi(argv[2]);
	int h = atoi(argv[3]);
	int frame = atoi(argv[4]);

	/* aligned dimensions */
	int aw = ((w + 15) / 16) * 16;
	int ah = ((h + 31) / 32) * 32;
	int half_w = aw / 2;
	int half_h = ah / 2;

	/* Y tiles: 16 x 32, row-major */
	int y_tc = aw / 16;      /* tile columns */
	int y_tr = ah / 32;      /* tile rows */
	int y_size = y_tc * y_tr * (16 * 32);

	/* UV: SAME tile grid as luma (y_tc x y_tr); each tile is 8x16 chroma
	 * samples (interleaved, 256 B), one per 16x32 luma tile. */
	int u_tc = y_tc;
	int u_tr = y_tr;
	int uv_size = u_tc * u_tr * (8 * 16 * 2);

	int frame_size = y_size + uv_size;

	FILE *f = fopen(argv[1], "rb");
	if (!f) {
		perror("open input");
		return 1;
	}
	if (fseek(f, (long)frame * frame_size, SEEK_SET) != 0) {
		fprintf(stderr, "seek to frame %d failed (frame_size=%d)\n",
			frame, frame_size);
		fclose(f);
		return 1;
	}
	unsigned char *yt = malloc(y_size);
	unsigned char *uvt = malloc(uv_size);
	size_t ry = fread(yt, 1, y_size, f);
	size_t ruv = fread(uvt, 1, uv_size, f);
	fclose(f);
	if (ry != (size_t)y_size || ruv != (size_t)uv_size) {
		fprintf(stderr, "short read: Y=%zu/%d UV=%zu/%d\n", ry, y_size,
			ruv, uv_size);
		free(yt);
		free(uvt);
		return 1;
	}

	/* de-tile Y: 16x32 tiles, row-major */
	unsigned char *yl = malloc((size_t)aw * ah);
	for (int tr = 0; tr < y_tr; tr++)
		for (int tc = 0; tc < y_tc; tc++) {
			const unsigned char *ts =
				yt + (size_t)(tr * y_tc + tc) * (16 * 32);
			for (int r = 0; r < 32; r++)
				for (int c = 0; c < 16; c++)
					yl[(size_t)(tr * 32 + r) * aw + (tc * 16 + c)] =
						ts[r * 16 + c];
		}

	/* de-tile UV: 8x16 chroma tiles (interleaved), same grid as luma */
	unsigned char *uvl = malloc((size_t)half_w * half_h * 2);
	for (int tr = 0; tr < u_tr; tr++)
		for (int tc = 0; tc < u_tc; tc++) {
			const unsigned char *ts =
				uvt + (size_t)(tr * u_tc + tc) * (8 * 16 * 2);
			for (int r = 0; r < 16; r++)
				for (int c = 0; c < 8; c++) {
					size_t di = ((size_t)(tr * 16 + r) * half_w + (tc * 8 + c)) * 2;
					uvl[di] = ts[r * 16 + c * 2];
					uvl[di + 1] = ts[r * 16 + c * 2 + 1];
				}
		}

	/* NV12 -> RGB24 PPM (BT.601), cropped to w x h */
	FILE *o = fopen(argv[5], "wb");
	if (!o) {
		perror("open output");
		free(yt);
		free(uvt);
		free(yl);
		free(uvl);
		return 1;
	}
	fprintf(o, "P6\n%d %d\n255\n", w, h);
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			int Yv = yl[(size_t)y * aw + x];
			int cx = x / 2, cy = y / 2;
			int U = uvl[((size_t)cy * half_w + cx) * 2];
			int V = uvl[((size_t)cy * half_w + cx) * 2 + 1];
			int r = Yv + 1.402 * (V - 128);
			int g = Yv - 0.344136 * (U - 128) - 0.714136 * (V - 128);
			int b = Yv + 1.772 * (U - 128);
			if (r < 0) r = 0;
			if (r > 255) r = 255;
			if (g < 0) g = 0;
			if (g > 255) g = 255;
			if (b < 0) b = 0;
			if (b > 255) b = 255;
			fputc(r, o);
			fputc(g, o);
			fputc(b, o);
		}
	}
	fclose(o);
	free(yt);
	free(uvt);
	free(yl);
	free(uvl);
	printf("wrote %s  (%dx%d, aligned %dx%d, frame %d; Y %dx%d tiles, UV %dx%d)\n",
	       argv[5], w, h, aw, ah, frame, y_tc, y_tr, u_tc, u_tr);
	return 0;
}
