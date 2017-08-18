/* lib/dithering.c
 *
 * (c) 2016 Yota Devices
 *
 * dithering algorithms for framebuffer drivers
 *
 */

#include <linux/module.h>
#include <linux/dithering.h>

#define gl256_to_none(x)	(x)
#define gl256_to_mono(x)	((x > 127) ? 255 : 0)
#define gl256_inrange(x)	((x < 0) ? 0 : ((x > 255) ? 255 : x))

/*
 *	Nearest value lookup table for for next 16 gray colors:
 *	0 17 34 51 68 85 102 119 136 153 170 187 204 221 238 255
 */

unsigned char gl256_to_16(unsigned char x)
{
	static const unsigned char colors256_16[] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
		0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
		0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x20, 0x20, 0x20, 0x20,
		0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
		0x20, 0x20, 0x20, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
		0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
		0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
		0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x50, 0x50, 0x50,
		0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50,
		0x50, 0x50, 0x50, 0x50, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60,
		0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60,
		0x60, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70,
		0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x80, 0x80,
		0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
		0x80, 0x80, 0x80, 0x80, 0x80, 0x90, 0x90, 0x90, 0x90, 0x90,
		0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90,
		0x90, 0x90, 0xa0, 0xa0, 0xa0, 0xa0, 0xa0, 0xa0, 0xa0, 0xa0,
		0xa0, 0xa0, 0xa0, 0xa0, 0xa0, 0xa0, 0xa0, 0xa0, 0xa0, 0xb0,
		0xb0, 0xb0, 0xb0, 0xb0, 0xb0, 0xb0, 0xb0, 0xb0, 0xb0, 0xb0,
		0xb0, 0xb0, 0xb0, 0xb0, 0xb0, 0xb0, 0xc0, 0xc0, 0xc0, 0xc0,
		0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0,
		0xc0, 0xc0, 0xc0, 0xd0, 0xd0, 0xd0, 0xd0, 0xd0, 0xd0, 0xd0,
		0xd0, 0xd0, 0xd0, 0xd0, 0xd0, 0xd0, 0xd0, 0xd0, 0xd0, 0xd0,
		0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0,
		0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xf0, 0xf0, 0xf0,
		0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0
	};

	return colors256_16[x];
}


#define SIMPLE_CONVERT(type)						\
static unsigned convert_##type(dthr_prop *desc, dthr_helper *sampler,	\
					void *in_p, void *out_p)	\
{									\
	u8 *dst = out_p;						\
	u8 *src = in_p;							\
	unsigned short i, j;						\
									\
	src += desc->top * desc->width * sampler->pix_sz;		\
									\
	for (j = desc->top; j < desc->bottom; ++j) {			\
		src += desc->left * sampler->pix_sz;			\
									\
		for (i = desc->left; i < desc->right; ++i) {		\
			*dst = gl256_to_##type(sampler->input_sampler(src,\
							sampler->priv));\
			dst++;						\
			src+= sampler->pix_sz;				\
		}							\
									\
		src += (desc->width - desc->right) * sampler->pix_sz;	\
	}								\
									\
	return (unsigned)(dst - (u8 *)out_p);				\
}

#define ATKINSON_DITHERING(type)						\
static unsigned atkinson_##type##_dithering(dthr_prop *desc,			\
				dthr_helper *sampler, void *in_p, void *out_p)	\
{										\
	u8 *dst = out_p;							\
	u8 *src = in_p;								\
										\
	signed char error;							\
	signed short tmp;							\
	unsigned short i, j;							\
										\
	signed char *pixel1 = dst + 1;						\
	signed char *pixel2 = dst + 2;						\
	signed char *pixel4 = dst + (desc->right - desc->left);			\
	signed char *pixel3 = pixel4 - 1;					\
	signed char *pixel5 = pixel4 + 1;					\
	signed char *pixel6 = pixel4 + (desc->right - desc->left);		\
										\
	inline void pixels_inc(void) {						\
		pixel1++;							\
		pixel2++;							\
		pixel3++;							\
		pixel4++;							\
		pixel5++;							\
		pixel6++;							\
		dst++;								\
	}									\
										\
	memset(dst, 0, (desc->right - desc->left) * (desc->bottom - desc->top));\
										\
	for (j = desc->top; j < desc->bottom; ++j) {				\
		src += desc->left * sampler->pix_sz;				\
										\
		tmp = *(signed char *)dst + 					\
				sampler->input_sampler(src, sampler->priv);	\
		tmp = gl256_inrange(tmp);					\
		*dst = gl256_to_##type(tmp);					\
		error = tmp - *dst;						\
		error /= 8;							\
										\
		*pixel1 += error;						\
		*pixel2 += error;						\
		*pixel4 += error;						\
		*pixel5 += error;						\
		*pixel6 += error;						\
										\
		pixels_inc();							\
		src+= sampler->pix_sz;						\
										\
		for (i = (desc->left + 1); i < (desc->right - 2); ++i) {	\
										\
			tmp = *(signed char *)dst + 				\
				sampler->input_sampler(src, sampler->priv);	\
										\
			tmp = gl256_inrange(tmp);				\
			*dst = gl256_to_##type(tmp);				\
			error = tmp - *dst;					\
			error /= 8;						\
										\
			*pixel1 += error;					\
			*pixel2 += error;					\
			*pixel3 += error;					\
			*pixel4 += error;					\
			*pixel5 += error;					\
			*pixel6 += error;					\
										\
			pixels_inc();						\
			src+= sampler->pix_sz;					\
		}								\
										\
		tmp = *(signed char *)dst + 					\
				sampler->input_sampler(src, sampler->priv);	\
		tmp = gl256_inrange(tmp);					\
		*dst = gl256_to_##type(tmp);					\
		error = tmp - *dst;						\
		error /= 8;							\
										\
		*pixel1 += error;						\
		*pixel3 += error;						\
		*pixel4 += error;						\
		*pixel5 += error;						\
		*pixel6 += error;						\
		pixels_inc();							\
		src+= sampler->pix_sz;						\
										\
		tmp = *(signed char *)dst + 					\
				sampler->input_sampler(src, sampler->priv);	\
		tmp = gl256_inrange(tmp);					\
		*dst = gl256_to_##type(tmp);					\
		error = tmp - *dst;						\
		error /= 8;							\
										\
		*pixel3 += error;						\
		*pixel4 += error;						\
		*pixel6 += error;						\
		pixels_inc();							\
		src+= sampler->pix_sz;						\
										\
		src += (desc->width - desc->right) * sampler->pix_sz;		\
	}									\
										\
	return (unsigned)(dst - (u8 *)out_p);					\
}

#define STEINBERG_DITHERING(type)						\
static unsigned steinberg_##type##_dithering(dthr_prop *desc,			\
				dthr_helper *sampler, void *in_p, void *out_p)	\
{										\
	u8 *dst = out_p;							\
	u8 *src = in_p;								\
										\
	signed char error;							\
	signed short tmp;							\
	unsigned short i, j;							\
										\
	signed char *pixel1 = dst + 1;						\
	signed char *pixel3 = dst + (desc->right - desc->left);			\
	signed char *pixel2 = pixel3 - 1;					\
	signed char *pixel4 = pixel3 + 1;					\
										\
	inline void pixels_inc(void) {						\
		pixel1++;							\
		pixel2++;							\
		pixel3++;							\
		pixel4++;							\
		dst++;								\
	}									\
										\
	memset(dst, 0, (desc->right - desc->left) * (desc->bottom - desc->top));\
										\
	for (j = desc->top; j < desc->bottom; ++j) {				\
		src += desc->left * sampler->pix_sz;				\
										\
		tmp = *(signed char *)dst + 					\
				sampler->input_sampler(src, sampler->priv);	\
		tmp = gl256_inrange(tmp);					\
		*dst = gl256_to_##type(tmp);					\
		error = tmp - *dst;						\
										\
		*pixel1 += (7 * error) / 16;					\
		*pixel3 += (5 * error) / 16;					\
		*pixel4 += error / 16;						\
										\
		pixels_inc();							\
		src+= sampler->pix_sz;						\
										\
		for (i = (desc->left + 1); i < (desc->right - 1); ++i) {	\
										\
			tmp = *(signed char *)dst + 				\
				sampler->input_sampler(src, sampler->priv);	\
										\
			tmp = gl256_inrange(tmp);				\
			*dst = gl256_to_##type(tmp);				\
			error = tmp - *dst;					\
										\
			*pixel1 += (7 * error) / 16;				\
			*pixel2 += (3 * error) / 16;				\
			*pixel3 += (5 * error) / 16;				\
			*pixel4 += error / 16;					\
										\
			pixels_inc();						\
			src+= sampler->pix_sz;					\
		}								\
										\
		tmp = *(signed char *)dst + 					\
				sampler->input_sampler(src, sampler->priv);	\
		tmp = gl256_inrange(tmp);					\
		*dst = gl256_to_##type(tmp);					\
		error = tmp - *dst;						\
										\
		*pixel2 += (3 * error) / 16;					\
		*pixel3 += (5 * error) / 16;					\
										\
		pixels_inc();							\
		src+= sampler->pix_sz;						\
										\
		src += (desc->width - desc->right) * sampler->pix_sz;		\
	}									\
										\
	return (unsigned)(dst - (u8 *)out_p);					\
}

SIMPLE_CONVERT(none);
SIMPLE_CONVERT(mono);

ATKINSON_DITHERING(mono);
ATKINSON_DITHERING(16);

STEINBERG_DITHERING(mono);
STEINBERG_DITHERING(16);

const dthr_alg dither_list[DITH_CNT] = {
	{.name = "none_16",   .fn = convert_none, .is_binary = 0},
	{.name = "none_mono", .fn = convert_mono, .is_binary = 1},
	{.name = "atkinson_16",   .fn = atkinson_16_dithering,   .is_binary = 0},
	{.name = "atkinson_mono", .fn = atkinson_mono_dithering, .is_binary = 1},
	{.name = "steinberg_16",   .fn = steinberg_16_dithering,   .is_binary = 0},
	{.name = "steinberg_mono", .fn = steinberg_mono_dithering, .is_binary = 1},
};
EXPORT_SYMBOL(dither_list);
