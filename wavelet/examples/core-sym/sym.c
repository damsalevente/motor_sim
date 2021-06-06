/**
 * @brief The single-loop core approach using the 4x4 core with the vertical vectorization.
 */
#include "libdwt.h"
#include "dwt-sym.h"
#include "dwt-sym-ms.h"
#include "inline.h"
#include <stdlib.h>

int dwt_util_image_check_s(
	float min_value,
	float max_value,
	const void *ptr,
	int stride_x,
	int stride_y,
	int size_i_big_x,
	int size_i_big_y
)
{
	const int eps = 1e-3f;

	int err_nan = 0;
	int err_int = 0;
	int err_max = 0;
	int err_min = 0;

	for(int y = 0; y < size_i_big_y; y++)
	{
		for(int x = 0; x < size_i_big_x; x++)
		{
			const float px = *addr2_const_s(ptr, y, x, stride_x, stride_y);

			// isnan
			if( px != px )
			{
				if( !err_nan++ )
					dwt_util_log(LOG_WARN, "%s: NaN value (%f) at (y=%i, x=%i). Such an incident will be reported only once.\n", __FUNCTION__, px, y, x);
			}

			// minimum integer value
			if( abs((int)px) < 0 )
			{
				if( !err_int++ )
					dwt_util_log(LOG_WARN, "%s: Wrong value (%f) at (y=%i, x=%i). Such an incident will be reported only once.\n", __FUNCTION__, px, y, x);
			}

			// maximum value
			if( px - eps > max_value )
			{
				if( !err_max++ )
					dwt_util_log(LOG_WARN, "%s: Maximum pixel intensity exceeded (%f > %f) at (y=%i, x=%i). Such an incident will be reported only once.\n", __FUNCTION__, px, max_value, y, x);
			}

			// minimum value
			if( px + eps < min_value )
			{
				if( !err_min++ )
					dwt_util_log(LOG_WARN, "%s: Minimum pixel intensity exceeded (%f < %f) at (y=%i, x=%i). Such an incident will be reported only once.\n", __FUNCTION__, px, 0.0f, y, x);
			}
		}
	}

	dwt_util_log(LOG_WARN, "%s: %i NaN errors, %i int. errors, %i max. errors, %i min. errors.\n",
		__FUNCTION__, err_nan, err_int, err_max, err_min);

	float min, max;

	dwt_util_find_min_max_s(
		ptr,
		size_i_big_x,
		size_i_big_y,
		stride_x,
		stride_y,
		&min,
		&max
	);

	dwt_util_log(LOG_WARN, "%s: min=%f max=%f.\n",
		__FUNCTION__, min, max);

	return 0;
}

static
int next_fwd_size(int size, int J)
{
	// J=0: 1
	// J=1: 2
	// J=2: 4
	const int size1 = size-1;
	const int mult1 = (1<<J)-1;

	const int new_size = 1 + ( ( size1 + mult1 ) & ~mult1 );

	dwt_util_log(LOG_DBG, "J=%i: size %i => %i\n", J, size, new_size);

	return new_size;
}

int main()
{
	int clock_type = dwt_util_clock_autoselect();

// 	int j = 1;
// 	int j = 2;
// 	int j = 3;
// 	int j = 4;
	int j = 5;
// 	int j = 6;
// 	int j = 7; // out of range
// 	int j = 8; // out of range

#if 1
// 	int size_x = 4096;
// 	int size_y = 4096;
// 	int size_x = 512;
// 	int size_y = 512;
// 	int size_x = 512+64; // J=5: pro násobky 64 funguje rovnou, netřeb bufferování
// 	int size_y = 512+64;
// 	int size_x = 513; // NOTE: OK
// 	int size_y = 513; // NOTE: OK
	// equal(j=7): 513+0, 513+128, 513+256
	// equal(j=3): 513+0, 513+8 [16x16]
	// equal(j=2): 513+0, 513+4 [8x8]
	// fails(j=7): 511, 512, 514, 515, 516, 517, 518, 519
	const int size = next_fwd_size(512, j);
	const int size_y = size;
	const int size_x = size;

	dwt_util_log(LOG_DBG, "size=(%i,%i)\n", size_y, size_x);

	int pattern_type = 1;

	int stride_y = sizeof(float);
	int stride_x = dwt_util_get_opt_stride(stride_y * size_x);

	void *src = dwt_util_alloc_image2(stride_x, stride_y, size_x, size_y);
	void *dst = dwt_util_alloc_image2(stride_x, stride_y, size_x, size_y);
	void *ref_transform = dwt_util_alloc_image2(stride_x, stride_y, size_x, size_y);

	dwt_util_test_image_fill2_s(src, stride_x, stride_y, size_x, size_y, 0, pattern_type);
	dwt_util_save_to_pgm_s("src.pgm", 1.0, src, stride_x, stride_y, size_x, size_y);

	// in case of in-place transform
	dwt_util_test_image_fill2_s(dst, stride_x, stride_y, size_x, size_y, 0, pattern_type);
	dwt_util_test_image_fill2_s(ref_transform, stride_x, stride_y, size_x, size_y, 0, pattern_type);
#else
	int size_x;
	int size_y;
	int stride_y;
	int stride_x;
	void *src;
	void *dst;
	dwt_util_load_from_pgm_s("Lenna.pgm", 1.f, &src, &stride_x, &stride_y, &size_x, &size_y);
	dwt_util_load_from_pgm_s("Lenna.pgm", 1.f, &dst, &stride_x, &stride_y, &size_x, &size_y);

	void *ref_transform;
	dwt_util_load_from_pgm_s("Lenna.pgm", 1.f, &ref_transform, &stride_x, &stride_y, &size_x, &size_y);
#endif

	// forward
	dwt_clock_t start = dwt_util_get_clock(clock_type);
#if 0
	// not in-place
	cdf97_2f_dl_4x4_s(
		size_x,
		size_y,
		src,
		stride_x,
		stride_y,
		dst,
		stride_x,
		stride_y
	);
#endif
#if 0
	// TODO
	// not in-place
	cdf97_2f_dl_2x2_s(
		size_x,
		size_y,
		src,
		stride_x,
		stride_y,
		dst,
		stride_x,
		stride_y
	);
#endif
#if 0
	// in-place
	cdf97_2f_dl_4x4_s(
		size_x,
		size_y,
		dst,
		stride_x,
		stride_y,
		dst,
		stride_x,
		stride_y
	);
#endif
#if 0
	// TODO: in-place, multi-scale
	ms_cdf97_2f_dl_4x4_fused_s(
		size_x,
		size_y,
		dst,
		stride_x,
		stride_y,
		j
	);
#endif
#if 1
	// TODO: not in-place, multi-scale
	// FIXME: *** currently uses _test implementation ***
	ms_cdf97_2f_dl_4x4_fused2_s(
		size_x,
		size_y,
		src, stride_x, stride_y,
		dst, stride_x, stride_y,
		j
	);
#endif
#if 0
	// TODO
	// in-place, multi-scale
	ms_cdf97_2f_dl_2x2_s(
		size_x,
		size_y,
		dst,
		stride_x,
		stride_y,
		j
	);
#endif
#if 0
	// in-place, multi-scale, several loops
	// FIXME: *** not fused ***
	dwt_cdf97_2f_dl_4x4_s(dst, stride_x, stride_y, size_x, size_y, size_x, size_y, &j, 1, 0);
#endif
#if 0
	// TODO
	// in-place, multi-scale, several loops
	dwt_cdf97_2f_dl_2x2_s(dst, stride_x, stride_y, size_x, size_y, size_x, size_y, &j, 1, 0);
#endif
#if 0
	// copy + in-place
	dwt_util_copy_s(src, dst, stride_x, stride_y, size_x, size_y);
	cdf97_2f_dl_4x4_s(
		size_x,
		size_y,
		dst,
		stride_x,
		stride_y,
		dst,
		stride_x,
		stride_y
	);
#endif
#if 0
	// reference, copy + in-place
	dwt_util_copy_s(src, dst, stride_x, stride_y, size_x, size_y);
	dwt_cdf97_2f_inplace_s(dst, stride_x, stride_y, size_x, size_y, size_x, size_y, &j, 1, 0);
#endif
#if 0
	// reference, in-place
	dwt_cdf97_2f_inplace_s(dst, stride_x, stride_y, size_x, size_y, size_x, size_y, &j, 1, 0);
#endif

	dwt_clock_t stop = dwt_util_get_clock(clock_type);
	double secs = (stop - start)/(double)dwt_util_get_frequency(clock_type);
	dwt_util_log(LOG_INFO, "forward @ image: %f Mpel; total: %f secs; pel: %f nsecs\n", (float)size_x*size_y/1024/1024, secs, secs/size_x/size_y*1024*1024*1024);
	dwt_util_save_log_to_pgm_s("dwt.pgm", dst, stride_x, stride_y, size_x, size_y);

	// ref. fwd. transform
	dwt_cdf97_2f_inplace_s(ref_transform, stride_x, stride_y, size_x, size_y, size_x, size_y, &j, 1, 0);
// 	const int error_x = size_x-2;
// 	const int error_y = 4;
// 	dwt_util_log(LOG_DBG, "correct write (%i,%i) = %f\n", error_y, error_x, *addr2_const_s(ref_transform, error_y, error_x, stride_x, stride_y));
	// compare
	if( dwt_util_compare2_destructive_s(
		ref_transform,
		dst,
		stride_x,
		stride_y,
		stride_x,
		stride_y,
		size_x,
		size_y
	) )
	{
		dwt_util_log(LOG_ERR, "transform differ!\n");
	}
	else
	{
		dwt_util_log(LOG_INFO, "transform equal :)\n");
	}
	dwt_util_save_to_pgm_s("err-transform.pgm", 1.0, ref_transform, stride_x, stride_y, size_x, size_y);

#if 1
	// inverse: reference, inplace
	dwt_cdf97_2i_inplace_s(dst, stride_x, stride_y, size_x, size_y, size_x, size_y, j, 1, 0);
#endif
#if 0
	// inverse: in-place, multi-scale, several loops
	dwt_cdf97_2i_dl_4x4_s(dst, stride_x, stride_y, size_x, size_y, size_x, size_y, j, 1, 0);
#endif
#if 0
	// inverse: not in-place, multi-scale TODO
	ms_cdf97_2i_dl_4x4_fused2_s(
		size_x,
		size_y,
		dst, stride_x, stride_y,
		dst, stride_x, stride_y,
		j
	);
#endif
	dwt_util_log(LOG_DBG, "saving reconstructed image...\n");
	dwt_util_image_check_s(0.f, 1.f, dst, stride_x, stride_y, size_x, size_y);
	dwt_util_save_to_pgm_s("dst.pgm", 1.0, dst, stride_x, stride_y, size_x, size_y);
	dwt_util_save_log_to_pgm_s("dst-log.pgm", dst, stride_x, stride_y, size_x, size_y);

	// compare
	if( dwt_util_compare2_destructive_s(
		dst,
		src,
		stride_x,
		stride_y,
		stride_x,
		stride_y,
		size_x,
		size_y
	) )
	{
		dwt_util_log(LOG_ERR, "images differ!\n");
	}
	else
	{
		dwt_util_log(LOG_INFO, "images equal :)\n");
	}

	dwt_util_save_to_pgm_s("err.pgm", 1.0, dst, stride_x, stride_y, size_x, size_y);

	dwt_util_free_image(&src);
	dwt_util_free_image(&dst);

// 	float fwd_secs, inv_secs;
// 	dwt_util_perf_cdf97_2f_dl_4x4_s(
// 		size_x,
// 		size_y,
// 		1, // opt_stride
// 		1, // M
// 		10, // N
// 		clock_type,
// 		&fwd_secs,
// 		&inv_secs,
// 		1, // flush
// 		0 // template type
// 	);
// 	dwt_util_log(LOG_INFO, "perf: %f secs\n", fwd_secs);

	return 0;
}
