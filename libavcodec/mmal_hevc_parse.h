/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * H.265 parser code
 */

#ifndef AVCODEC_MMAL_HEVC_PARSE_H
#define AVCODEC_MMAL_HEVC_PARSE_H

#include <stdint.h>

#include "mmal_hevc_ps.h"
#include "mmal_hevc_sei.h"

int ff_hevc_mmal_decode_extradata(const uint8_t *data, int size, HEVCMMALParamSets *ps,
                             HEVCSEIContext *sei, int *is_nalff, int *nal_length_size,
                             int err_recognition, int apply_defdispwin, void *logctx);

#endif /* AVCODEC_MMAL_HEVC_PARSE_H */
