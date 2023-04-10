/*
Copyright (c) 2018 Raspberry Pi (Trading) Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Authors: John Cox
*/


#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <stdint.h>

#include "config.h"

#include "libavutil/avassert.h"
#include "libavutil/mmal_sand_fns.h"

#pragma GCC diagnostic push
// Many many redundant decls in the header files
#pragma GCC diagnostic ignored "-Wredundant-decls"
#include <interface/vctypes/vc_image_types.h>
#include <interface/vcsm/user-vcsm.h>
#pragma GCC diagnostic pop

#include "mmal_mem.h"
#include "mmal_zc_frames.h"


#define OPT_PREFER_CMA 0

struct mmal_cache_flush_env_s {
  struct vcsm_user_clean_invalid2_s v;
};


// GPU memory alloc fns (internal)

static void gpu_free_internal(GPU_MEM_PTR_T * const p)
{
    if (p->arm != NULL)
        vcsm_unlock_ptr(p->arm);
    if (p->vcsm_handle != 0)
        vcsm_free(p->vcsm_handle);
    memset(p, 0, sizeof(*p));  // Ensure we crash hard if we try and use this again
}


static int gpu_malloc_internal(GPU_MEM_PTR_T * const p,
    const int numbytes, const unsigned int cache_type, const char * const name)
{
    memset(p, 0, sizeof(*p));
    p->numbytes = (numbytes + 255) & ~255;  // Round up

    if ((p->vcsm_handle = vcsm_malloc_cache(p->numbytes, cache_type | 0x80, (char *)name)) == 0)
    {
        av_log(NULL, AV_LOG_ERROR, "Unable to alloc %d bytes from VCSM for %s\n", p->numbytes, name);
        goto fail;
    }
    if ((p->vc_handle = vcsm_vc_hdl_from_hdl(p->vcsm_handle)) == 0)
    {
        av_log(NULL, AV_LOG_ERROR, "Unable to VC handle from VCSM for %s\n", name);
        goto fail;
    }
    if ((p->arm = vcsm_lock(p->vcsm_handle)) == NULL)
    {
        av_log(NULL, AV_LOG_ERROR, "Unable to lock handle from VCSM for %s\n", name);
        goto fail;
    }
    if ((p->vc = vcsm_vc_addr_from_hdl(p->vcsm_handle)) == 0)
    {
        av_log(NULL, AV_LOG_ERROR, "Unable to get VC addr from VCSM for %s\n", name);
        goto fail;
    }

    return 0;

fail:
    gpu_free_internal(p);
    return AVERROR(ENOMEM);
}

// Public gpu fns

// Allocate memory on GPU
// Fills in structure <p> containing ARM pointer, videocore handle, videocore memory address, numbytes
// Returns 0 on success.
// This allocates memory that will not be cached in ARM's data cache.
// Therefore safe to use without data cache flushing.
int gpu_malloc_uncached(int numbytes, GPU_MEM_PTR_T *p)
{
    return gpu_malloc_internal(p, numbytes, VCSM_CACHE_TYPE_NONE, "ffmpeg uncached");
}

// This allocates data that will be
//    Cached in ARM L2
//    Uncached in VPU L2
int gpu_malloc_cached(int numbytes, GPU_MEM_PTR_T *p)
{
    return gpu_malloc_internal(p, numbytes, VCSM_CACHE_TYPE_HOST, "ffmpeg cached");
}

void gpu_free(GPU_MEM_PTR_T * const p) {
    gpu_free_internal(p);
}

void mmal_mem_gpu_uninit(void)
{
    vcsm_exit();
}

int mmal_mem_gpu_init(const unsigned int flags)
{
    int use_cma;

    (void)flags;

    if (vcsm_init_ex(0, -1) == 0)
        use_cma = 1;
    else if (vcsm_init_ex(1, -1) == 0)
        use_cma = 0;
    else
        return AVERROR(EINVAL);

    return use_cma + 1;
}

// ----------------------------------------------------------------------------
//
// Cache flush functions

#define CACHE_EL_MAX ((sizeof(mmal_cache_buf_t) - sizeof (struct vcsm_user_clean_invalid2_s)) / sizeof (struct vcsm_user_clean_invalid2_block_s))

mmal_cache_flush_env_t * mmal_cache_flush_init(mmal_cache_buf_t * const buf)
{
  mmal_cache_flush_env_t * const rfe = (mmal_cache_flush_env_t *)buf;
  *rfe = (mmal_cache_flush_env_t){.v={.op_count = 0}};
  return rfe;
}

void mmal_cache_flush_abort(mmal_cache_flush_env_t * const rfe)
{
  // Nothing needed
}

int mmal_cache_flush_execute(mmal_cache_flush_env_t * const rfe)
{
    int rc = 0;
    if (rfe->v.op_count != 0) {
        if (vcsm_clean_invalid2(&rfe->v) != 0)
        {
          const int err = errno;
          if (err != EINVAL)
            av_log(NULL, AV_LOG_WARNING, "vcsm_clean_invalid2 failed: errno=%d\n", err);

          rc = AVERROR(err);
        }
        rfe->v.op_count = 0;
    }
    return rc;
}

int mmal_cache_flush_finish(mmal_cache_flush_env_t * const rfe)
{
  int rc = mmal_cache_flush_execute(rfe);;

  return rc;
}

inline void mmal_cache_flush_add_gm_blocks(mmal_cache_flush_env_t * const rfe, const GPU_MEM_PTR_T * const gm, const unsigned int mode,
  const unsigned int offset0, const unsigned int block_size, const unsigned int blocks, const unsigned int block_stride)
{
  struct vcsm_user_clean_invalid2_block_s * const b = rfe->v.s + rfe->v.op_count++;

  av_assert1(rfe->v.op_count <= CACHE_EL_MAX);

  b->invalidate_mode = mode;
  b->block_count = blocks;
  b->start_address = gm->arm + offset0;
  b->block_size = block_size;
  b->inter_block_stride = block_stride;
}

void mmal_cache_flush_add_gm_range(mmal_cache_flush_env_t * const rfe, const GPU_MEM_PTR_T * const gm, const unsigned int mode,
  const unsigned int offset, const unsigned int size)
{
  // Deal with empty pointer trivially
  if (gm == NULL || size == 0)
    return;

  av_assert1(offset <= gm->numbytes);
  av_assert1(size <= gm->numbytes);
  av_assert1(offset + size <= gm->numbytes);

  mmal_cache_flush_add_gm_blocks(rfe, gm, mode, offset, size, 1, 0);
}

void mmal_cache_flush_add_gm_ptr(mmal_cache_flush_env_t * const rfe, const GPU_MEM_PTR_T * const gm, const unsigned int mode)
{
  mmal_cache_flush_add_gm_blocks(rfe, gm, mode, 0, gm->numbytes, 1, 0);
}


void mmal_cache_flush_add_frame(mmal_cache_flush_env_t * const rfe, const AVFrame * const frame, const unsigned int mode)
{
#if !MMAL_ONE_BUF
#error Fixme! (NIF)
#endif
  if (gpu_is_buf1(frame)) {
    mmal_cache_flush_add_gm_ptr(rfe, gpu_buf1_gmem(frame), mode);
  }
  else
  {
    mmal_cache_flush_add_gm_ptr(rfe, gpu_buf3_gmem(frame, 0), mode);
    mmal_cache_flush_add_gm_ptr(rfe, gpu_buf3_gmem(frame, 1), mode);
    mmal_cache_flush_add_gm_ptr(rfe, gpu_buf3_gmem(frame, 2), mode);
  }
}

// Flush an area of a frame
// Width, height, x0, y0 in luma pels
void mmal_cache_flush_add_frame_block(mmal_cache_flush_env_t * const rfe, const AVFrame * const frame, const unsigned int mode,
  const unsigned int x0, const unsigned int y0, const unsigned int width, const unsigned int height,
  const unsigned int uv_shift, const int do_luma, const int do_chroma)
{
  const unsigned int y_offset = frame->linesize[0] * y0;
  const unsigned int y_size = frame->linesize[0] * height;
  // Round UV up/down to get everything
  const unsigned int uv_rnd = (1U << uv_shift) >> 1;
  const unsigned int uv_offset = frame->linesize[1] * (y0 >> uv_shift);
  const unsigned int uv_size = frame->linesize[1] * ((y0 + height + uv_rnd) >> uv_shift) - uv_offset;

#if 0
  // *** frame->height is cropped height so not good
  // As all unsigned they will also reject -ve
  // Test individually as well as added to reject overflow
  av_assert0(start_line <= (unsigned int)frame->height);  // ***** frame height cropped
  av_assert0(n <= (unsigned int)frame->height);
  av_assert0(start_line + n <= (unsigned int)frame->height);
#endif

  if (!gpu_is_buf1(frame))
  {
    if (do_luma) {
      mmal_cache_flush_add_gm_range(rfe, gpu_buf3_gmem(frame, 0), mode, y_offset, y_size);
    }
    if (do_chroma) {
      mmal_cache_flush_add_gm_range(rfe, gpu_buf3_gmem(frame, 1), mode, uv_offset, uv_size);
      mmal_cache_flush_add_gm_range(rfe, gpu_buf3_gmem(frame, 2), mode, uv_offset, uv_size);
    }
  }
  else if (!av_mmal_is_sand_frame(frame))
  {
    const GPU_MEM_PTR_T * const gm = gpu_buf1_gmem(frame);
    if (do_luma) {
      mmal_cache_flush_add_gm_range(rfe, gm, mode, (frame->data[0] - gm->arm) + y_offset, y_size);
    }
    if (do_chroma) {
      mmal_cache_flush_add_gm_range(rfe, gm, mode, (frame->data[1] - gm->arm) + uv_offset, uv_size);
      mmal_cache_flush_add_gm_range(rfe, gm, mode, (frame->data[2] - gm->arm) + uv_offset, uv_size);
    }
  }
  else
  {
    const unsigned int stride1 = av_mmal_sand_frame_stride1(frame);
    const unsigned int stride2 = av_mmal_sand_frame_stride2(frame);
    const unsigned int xshl = av_mmal_sand_frame_xshl(frame);
    const unsigned int xleft = x0 & ~((stride1 >> xshl) - 1);
    const unsigned int block_count = (((x0 + width - xleft) << xshl) + stride1 - 1) / stride1;  // Same for Y & C
    av_assert1(rfe->v.op_count + do_chroma + do_luma < CACHE_EL_MAX);

    if (do_chroma)
    {
      struct vcsm_user_clean_invalid2_block_s * const b = rfe->v.s + rfe->v.op_count++;
      b->invalidate_mode = mode;
      b->block_count = block_count;
      b->start_address = av_mmal_sand_frame_pos_c(frame, xleft >> 1, y0 >> 1);
      b->block_size = uv_size;
      b->inter_block_stride = stride1 * stride2;
    }
    if (do_luma)
    {
      struct vcsm_user_clean_invalid2_block_s * const b = rfe->v.s + rfe->v.op_count++;
      b->invalidate_mode = mode;
      b->block_count = block_count;
      b->start_address = av_mmal_sand_frame_pos_y(frame, xleft, y0);
      b->block_size = y_size;
      b->inter_block_stride = stride1 * stride2;
    }
  }
}

// Call this to clean and invalidate a region of memory
void mmal_cache_flush_one_gm_ptr(const GPU_MEM_PTR_T *const p, const mmal_cache_flush_mode_t mode)
{
  mmal_cache_buf_t cbuf;
  mmal_cache_flush_env_t * rfe = mmal_cache_flush_init(&cbuf);
  mmal_cache_flush_add_gm_ptr(rfe, p, mode);
  mmal_cache_flush_finish(rfe);
}

