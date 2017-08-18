#ifndef _LINUX_DITERING_H
#define _LINUX_DITERING_H

#include <linux/types.h>

/* describe current image size and working crop */
typedef struct {
	/* source image size */
	unsigned short width;
	unsigned short height;

	/* crop size */
	unsigned short left;
	unsigned short right;
	unsigned short top;
	unsigned short bottom;
} dthr_prop;

/* dithering helper required to support input format in right way */
typedef struct {
	void *priv;
	unsigned char pix_sz;
	u8 (*input_sampler)(void *pix, void *priv);
} dthr_helper;

/*
 * dthr_alg type
 * describe one instance of dithering algorithm
 * contain name, function pointer and output type
 * if is_binary = 1, output byte could be black(0xff)
 * and white(0x00), otherwise output in rage 0..255
 */
typedef struct {
	char *name;
	unsigned (*fn)(dthr_prop *, dthr_helper *, void *in_p, void *out_p);
	unsigned char is_binary;
} dthr_alg;

#define DITH_CNT	6
extern const dthr_alg dither_list[DITH_CNT];

#endif
