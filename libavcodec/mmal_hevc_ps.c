/*
 * HEVC Parameter Set decoding
 *
 * Copyright (C) 2012 - 2103 Guillaume Martres
 * Copyright (C) 2012 - 2103 Mickael Raulet
 * Copyright (C) 2012 - 2013 Gildas Cocherel
 * Copyright (C) 2013 Vittorio Giovara
 * Copyright (C) 2018 John Cox for Raspberry Pi (Trading)
 *
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

#include "libavutil/imgutils.h"
#include "golomb.h"
#include "mmal_hevc_data.h"
#include "mmal_hevc_ps.h"
#include "mmal_hevcdec.h"

static const uint8_t default_scaling_list_intra[] = {
    16, 16, 16, 16, 17, 18, 21, 24,
    16, 16, 16, 16, 17, 19, 22, 25,
    16, 16, 17, 18, 20, 22, 25, 29,
    16, 16, 18, 21, 24, 27, 31, 36,
    17, 17, 20, 24, 30, 35, 41, 47,
    18, 19, 22, 27, 35, 44, 54, 65,
    21, 22, 25, 31, 41, 54, 70, 88,
    24, 25, 29, 36, 47, 65, 88, 115
};

static const uint8_t default_scaling_list_inter[] = {
    16, 16, 16, 16, 17, 18, 20, 24,
    16, 16, 16, 17, 18, 20, 24, 25,
    16, 16, 17, 18, 20, 24, 25, 28,
    16, 17, 18, 20, 24, 25, 28, 33,
    17, 18, 20, 24, 25, 28, 33, 41,
    18, 20, 24, 25, 28, 33, 41, 54,
    20, 24, 25, 28, 33, 41, 54, 71,
    24, 25, 28, 33, 41, 54, 71, 91
};

static const AVRational vui_sar[] = {
    {  0,   1 },
    {  1,   1 },
    { 12,  11 },
    { 10,  11 },
    { 16,  11 },
    { 40,  33 },
    { 24,  11 },
    { 20,  11 },
    { 32,  11 },
    { 80,  33 },
    { 18,  11 },
    { 15,  11 },
    { 64,  33 },
    { 160, 99 },
    {  4,   3 },
    {  3,   2 },
    {  2,   1 },
};


// pps_cb_qp_offset: -12,+12
// slice_cb_qp_offset: -12,+12 also
//   "The value of pps_cb_qp_offset + slice_cb_qp_offset shall be in the range of -12 to +12, inclusive."
// cr_qp_offset_list[n]: -12,+12
// So worst case total offset: -24,+24

#define T(n) ((((48+(n))/6-10)<<3) | (48+(n))%6)
#define C(B,n) T(B*6+(n) < 0 ? -B*6 : (n) > 51 ? 51 : (n))
#define M(B,n) C(B,(-n))

// Sizeof the QP_START_BLOCK
#define QP_OFFSET_0 (8*6 + 12*2)
#define QP_START(B) \
    M(B,48), M(B,48), M(B,48), M(B,48), M(B,48), M(B,48),\
    M(B,48), M(B,48), M(B,48), M(B,48), M(B,48), M(B,48),\
    M(B,48), M(B,48), M(B,48), M(B,48), M(B,48), M(B,48),\
    M(B,48), M(B,48), M(B,48), M(B,48), M(B,48), M(B,48),\
\
    M(B,48), M(B,47), M(B,46), M(B,45), M(B,44), M(B,43),\
    M(B,42), M(B,41), M(B,40), M(B,39), M(B,38), M(B,37),\
    M(B,36), M(B,35), M(B,34), M(B,33), M(B,32), M(B,31),\
    M(B,30), M(B,29), M(B,28), M(B,27), M(B,26), M(B,25),\
    M(B,24), M(B,23), M(B,22), M(B,21), M(B,20), M(B,19),\
    M(B,18), M(B,17), M(B,16), M(B,15), M(B,14), M(B,13),\
    M(B,12), M(B,11), M(B,10), M(B, 9), M(B, 8), M(B, 7),\
    M(B, 6), M(B, 5), M(B, 4), M(B, 3), M(B, 2), M(B, 1)
#define QP_END(B) \
    C(B,51), C(B,51), C(B,51), C(B,51), C(B,51), C(B,51),\
    C(B,51), C(B,51), C(B,51), C(B,51), C(B,51), C(B,51),\
    C(B,51), C(B,51), C(B,51), C(B,51), C(B,51), C(B,51)

#define T1(B)\
{\
    QP_START(B),\
    C(B, 0), C(B, 1), C(B, 2), C(B, 3), C(B, 4), C(B, 5), C(B, 6), C(B, 7), C(B, 8), C(B, 9),\
    C(B,10), C(B,11), C(B,12), C(B,13), C(B,14), C(B,15), C(B,16), C(B,17), C(B,18), C(B,19),\
    C(B,20), C(B,21), C(B,22), C(B,23), C(B,24), C(B,25), C(B,26), C(B,27), C(B,28), C(B,29),\
    C(B,29), C(B,30), C(B,31), C(B,32), C(B,33), C(B,33), C(B,34), C(B,34), C(B,35), C(B,35),\
    C(B,36), C(B,36), C(B,37), C(B,37), C(B,38), C(B,39), C(B,40), C(B,41), C(B,42), C(B,43),\
    C(B,44), C(B,45),\
    C(B,46), C(B,47), C(B,48), C(B,49), C(B,50), C(B,51),\
    QP_END(B)\
}
#define T0(B)\
{\
    QP_START(B),\
    C(B, 0), C(B, 1), C(B, 2), C(B, 3), C(B, 4), C(B, 5), C(B, 6), C(B, 7), C(B, 8), C(B, 9),\
    C(B,10), C(B,11), C(B,12), C(B,13), C(B,14), C(B,15), C(B,16), C(B,17), C(B,18), C(B,19),\
    C(B,20), C(B,21), C(B,22), C(B,23), C(B,24), C(B,25), C(B,26), C(B,27), C(B,28), C(B,29),\
    C(B,30), C(B,31), C(B,32), C(B,33), C(B,34), C(B,35), C(B,36), C(B,37), C(B,38), C(B,39),\
    C(B,40), C(B,41), C(B,42), C(B,43), C(B,44), C(B,45), C(B,46), C(B,47), C(B,48), C(B,49),\
    C(B,50), C(B,51),\
    C(B,51), C(B,51), C(B,51), C(B,51), C(B,51), C(B,51),\
    QP_END(B)\
}

#define QP_TABLE_SIZE (QP_OFFSET_0 + 52 + 12*2)

static const int8_t qp_c_bd_0[8][QP_TABLE_SIZE] = {T0(0),T0(1),T0(2),T0(3),T0(4),T0(5),T0(6),T0(7)};
static const int8_t qp_c_bd_1[8][QP_TABLE_SIZE] = {T1(0),T1(1),T1(2),T1(3),T1(4),T1(5),T1(6),T1(7)};

#undef T
#undef C
#undef QP_END

#define C(B,n) ((n)<0?0:(n)>51?51:(n))
// We do need a lot of -ve padding to cope with high bit depths that give -ve qps
#define QP_DBLK_OFFSET_0 QP_OFFSET_0
#define QP_END(B)\
 51, 51, 51, 51, 51, 51

// These don't need all the padding we have here (12 top/bottom would be enough)
static const uint8_t qp_c_dblk_0[] = T0(0);
static const uint8_t qp_c_dblk_1[] = T1(0);

#undef T
#undef M
#undef C
#undef QP_END
#undef QP_START


static void remove_pps(HEVCMMALParamSets * const s, const int id)
{
    if (s->pps_list[id] && s->pps == (const HEVCMMALPPS*)s->pps_list[id]->data)
        s->pps = NULL;
    av_buffer_unref(&s->pps_list[id]);
}

static void remove_sps(HEVCMMALParamSets * const s, const int id)
{
    int i;
    if (s->sps_list[id]) {
        if (s->sps == (const HEVCMMALSPS*)s->sps_list[id]->data)
            s->sps = NULL;

        /* drop all PPS that depend on this SPS */
        for (i = 0; i < FF_ARRAY_ELEMS(s->pps_list); i++)
            if (s->pps_list[i] && ((HEVCMMALPPS*)s->pps_list[i]->data)->sps_id == id)
                remove_pps(s, i);

        av_assert0(!(s->sps_list[id] && s->sps == (HEVCMMALSPS*)s->sps_list[id]->data));
    }
    av_buffer_unref(&s->sps_list[id]);
}

static void remove_vps(HEVCMMALParamSets * const s, const int id)
{
    int i;
    if (s->vps_list[id]) {
        if (s->vps == (const HEVCMMALVPS*)s->vps_list[id]->data)
            s->vps = NULL;

        for (i = 0; i < FF_ARRAY_ELEMS(s->sps_list); i++)
            if (s->sps_list[i] && ((HEVCMMALSPS*)s->sps_list[i]->data)->vps_id == id)
                remove_sps(s, i);
    }
    av_buffer_unref(&s->vps_list[id]);
}

int ff_hevc_mmal_decode_short_term_rps(GetBitContext * const gb, AVCodecContext * const avctx,
                                  ShortTermRPS * const rps, const HEVCMMALSPS * const sps, const int is_slice_header)
{
    uint8_t rps_predict = 0;
    int delta_poc;
    int k0 = 0;
    int k1 = 0;
    int k  = 0;
    int i;

    if (rps != sps->st_rps && sps->nb_st_rps)
        rps_predict = get_bits1(gb);

    if (rps_predict) {
        const ShortTermRPS *rps_ridx;
        int delta_rps;
        unsigned abs_delta_rps;
        uint8_t use_delta_flag = 0;
        uint8_t delta_rps_sign;

        if (is_slice_header) {
            unsigned int delta_idx = get_ue_golomb_long(gb) + 1;
            if (delta_idx > sps->nb_st_rps) {
                av_log(avctx, AV_LOG_ERROR,
                       "Invalid value of delta_idx in slice header RPS: %d > %d.\n",
                       delta_idx, sps->nb_st_rps);
                return AVERROR_INVALIDDATA;
            }
            rps_ridx = &sps->st_rps[sps->nb_st_rps - delta_idx];
            rps->rps_idx_num_delta_pocs = rps_ridx->num_delta_pocs;
        } else
            rps_ridx = &sps->st_rps[rps - sps->st_rps - 1];

        delta_rps_sign = get_bits1(gb);
        abs_delta_rps  = get_ue_golomb_long(gb) + 1;
        if (abs_delta_rps < 1 || abs_delta_rps > 32768) {
            av_log(avctx, AV_LOG_ERROR,
                   "Invalid value of abs_delta_rps: %d\n",
                   abs_delta_rps);
            return AVERROR_INVALIDDATA;
        }
        delta_rps      = (1 - (delta_rps_sign << 1)) * abs_delta_rps;
        for (i = 0; i <= rps_ridx->num_delta_pocs; i++) {
            int used = rps->used[k] = get_bits1(gb);

            if (!used)
                use_delta_flag = get_bits1(gb);

            if (used || use_delta_flag) {
                if (i < rps_ridx->num_delta_pocs)
                    delta_poc = delta_rps + rps_ridx->delta_poc[i];
                else
                    delta_poc = delta_rps;
                rps->delta_poc[k] = delta_poc;
                if (delta_poc < 0)
                    k0++;
                else
                    k1++;
                k++;
            }
        }

        if (k >= FF_ARRAY_ELEMS(rps->used)) {
            av_log(avctx, AV_LOG_ERROR,
                   "Invalid num_delta_pocs: %d\n", k);
            return AVERROR_INVALIDDATA;
        }

        rps->num_delta_pocs    = k;
        rps->num_negative_pics = k0;
        // sort in increasing order (smallest first)
        if (rps->num_delta_pocs != 0) {
            int used, tmp;
            for (i = 1; i < rps->num_delta_pocs; i++) {
                delta_poc = rps->delta_poc[i];
                used      = rps->used[i];
                for (k = i - 1; k >= 0; k--) {
                    tmp = rps->delta_poc[k];
                    if (delta_poc < tmp) {
                        rps->delta_poc[k + 1] = tmp;
                        rps->used[k + 1]      = rps->used[k];
                        rps->delta_poc[k]     = delta_poc;
                        rps->used[k]          = used;
                    }
                }
            }
        }
        if ((rps->num_negative_pics >> 1) != 0) {
            int used;
            k = rps->num_negative_pics - 1;
            // flip the negative values to largest first
            for (i = 0; i < rps->num_negative_pics >> 1; i++) {
                delta_poc         = rps->delta_poc[i];
                used              = rps->used[i];
                rps->delta_poc[i] = rps->delta_poc[k];
                rps->used[i]      = rps->used[k];
                rps->delta_poc[k] = delta_poc;
                rps->used[k]      = used;
                k--;
            }
        }
    } else {
        unsigned int prev, nb_positive_pics;
        rps->num_negative_pics = get_ue_golomb_long(gb);
        nb_positive_pics       = get_ue_golomb_long(gb);

        if (rps->num_negative_pics >= HEVC_MAX_REFS ||
            nb_positive_pics >= HEVC_MAX_REFS) {
            av_log(avctx, AV_LOG_ERROR, "Too many refs in a short term RPS.\n");
            return AVERROR_INVALIDDATA;
        }

        rps->num_delta_pocs = rps->num_negative_pics + nb_positive_pics;
        if (rps->num_delta_pocs) {
            prev = 0;
            for (i = 0; i < rps->num_negative_pics; i++) {
                delta_poc = get_ue_golomb_long(gb) + 1;
                if (delta_poc < 1 || delta_poc > 32768) {
                    av_log(avctx, AV_LOG_ERROR,
                        "Invalid value of delta_poc: %d\n",
                        delta_poc);
                    return AVERROR_INVALIDDATA;
                }
                prev -= delta_poc;
                rps->delta_poc[i] = prev;
                rps->used[i]      = get_bits1(gb);
            }
            prev = 0;
            for (i = 0; i < nb_positive_pics; i++) {
                delta_poc = get_ue_golomb_long(gb) + 1;
                if (delta_poc < 1 || delta_poc > 32768) {
                    av_log(avctx, AV_LOG_ERROR,
                        "Invalid value of delta_poc: %d\n",
                        delta_poc);
                    return AVERROR_INVALIDDATA;
                }
                prev += delta_poc;
                rps->delta_poc[rps->num_negative_pics + i] = prev;
                rps->used[rps->num_negative_pics + i]      = get_bits1(gb);
            }
        }
    }
    return 0;
}


static int decode_profile_tier_level(GetBitContext * const gb, AVCodecContext * const avctx,
                                      PTLCommon * const ptl)
{
    int i;

    if (get_bits_left(gb) < 2+1+5 + 32 + 4 + 16 + 16 + 12)
        return -1;

    ptl->profile_space = get_bits(gb, 2);
    ptl->tier_flag     = get_bits1(gb);
    ptl->profile_idc   = get_bits(gb, 5);
    if (ptl->profile_idc == FF_PROFILE_HEVC_MAIN)
        av_log(avctx, AV_LOG_DEBUG, "Main profile bitstream\n");
    else if (ptl->profile_idc == FF_PROFILE_HEVC_MAIN_10)
        av_log(avctx, AV_LOG_DEBUG, "Main 10 profile bitstream\n");
    else if (ptl->profile_idc == FF_PROFILE_HEVC_MAIN_STILL_PICTURE)
        av_log(avctx, AV_LOG_DEBUG, "Main Still Picture profile bitstream\n");
    else if (ptl->profile_idc == FF_PROFILE_HEVC_REXT)
        av_log(avctx, AV_LOG_DEBUG, "Range Extension profile bitstream\n");
    else
        av_log(avctx, AV_LOG_WARNING, "Unknown HEVC profile: %d\n", ptl->profile_idc);

    for (i = 0; i < 32; i++) {
        ptl->profile_compatibility_flag[i] = get_bits1(gb);

        if (ptl->profile_idc == 0 && i > 0 && ptl->profile_compatibility_flag[i])
            ptl->profile_idc = i;
    }
    ptl->progressive_source_flag    = get_bits1(gb);
    ptl->interlaced_source_flag     = get_bits1(gb);
    ptl->non_packed_constraint_flag = get_bits1(gb);
    ptl->frame_only_constraint_flag = get_bits1(gb);

    skip_bits(gb, 16); // XXX_reserved_zero_44bits[0..15]
    skip_bits(gb, 16); // XXX_reserved_zero_44bits[16..31]
    skip_bits(gb, 12); // XXX_reserved_zero_44bits[32..43]

    return 0;
}

static int parse_ptl(GetBitContext * const gb, AVCodecContext * const avctx,
                      PTL * const ptl, const int max_num_sub_layers)
{
    int i;
    if (decode_profile_tier_level(gb, avctx, &ptl->general_ptl) < 0 ||
        get_bits_left(gb) < 8 + (8*2 * (max_num_sub_layers - 1 > 0))) {
        av_log(avctx, AV_LOG_ERROR, "PTL information too short\n");
        return -1;
    }

    ptl->general_ptl.level_idc = get_bits(gb, 8);

    for (i = 0; i < max_num_sub_layers - 1; i++) {
        ptl->sub_layer_profile_present_flag[i] = get_bits1(gb);
        ptl->sub_layer_level_present_flag[i]   = get_bits1(gb);
    }

    if (max_num_sub_layers - 1> 0)
        for (i = max_num_sub_layers - 1; i < 8; i++)
            skip_bits(gb, 2); // reserved_zero_2bits[i]
    for (i = 0; i < max_num_sub_layers - 1; i++) {
        if (ptl->sub_layer_profile_present_flag[i] &&
            decode_profile_tier_level(gb, avctx, &ptl->sub_layer_ptl[i]) < 0) {
            av_log(avctx, AV_LOG_ERROR,
                   "PTL information for sublayer %i too short\n", i);
            return -1;
        }
        if (ptl->sub_layer_level_present_flag[i]) {
            if (get_bits_left(gb) < 8) {
                av_log(avctx, AV_LOG_ERROR,
                       "Not enough data for sublayer %i level_idc\n", i);
                return -1;
            } else
                ptl->sub_layer_ptl[i].level_idc = get_bits(gb, 8);
        }
    }

    return 0;
}

static void decode_sublayer_hrd(GetBitContext * const gb, const unsigned int nb_cpb,
                                const int subpic_params_present)
{
    int i;

    for (i = 0; i < nb_cpb; i++) {
        get_ue_golomb_long(gb); // bit_rate_value_minus1
        get_ue_golomb_long(gb); // cpb_size_value_minus1

        if (subpic_params_present) {
            get_ue_golomb_long(gb); // cpb_size_du_value_minus1
            get_ue_golomb_long(gb); // bit_rate_du_value_minus1
        }
        skip_bits1(gb); // cbr_flag
    }
}

static int decode_hrd(GetBitContext * const gb, const int common_inf_present,
                      const int max_sublayers)
{
    int nal_params_present = 0, vcl_params_present = 0;
    int subpic_params_present = 0;
    int i;

    if (common_inf_present) {
        nal_params_present = get_bits1(gb);
        vcl_params_present = get_bits1(gb);

        if (nal_params_present || vcl_params_present) {
            subpic_params_present = get_bits1(gb);

            if (subpic_params_present) {
                skip_bits(gb, 8); // tick_divisor_minus2
                skip_bits(gb, 5); // du_cpb_removal_delay_increment_length_minus1
                skip_bits(gb, 1); // sub_pic_cpb_params_in_pic_timing_sei_flag
                skip_bits(gb, 5); // dpb_output_delay_du_length_minus1
            }

            skip_bits(gb, 4); // bit_rate_scale
            skip_bits(gb, 4); // cpb_size_scale

            if (subpic_params_present)
                skip_bits(gb, 4);  // cpb_size_du_scale

            skip_bits(gb, 5); // initial_cpb_removal_delay_length_minus1
            skip_bits(gb, 5); // au_cpb_removal_delay_length_minus1
            skip_bits(gb, 5); // dpb_output_delay_length_minus1
        }
    }

    for (i = 0; i < max_sublayers; i++) {
        int low_delay = 0;
        unsigned int nb_cpb = 1;
        int fixed_rate = get_bits1(gb);

        if (!fixed_rate)
            fixed_rate = get_bits1(gb);

        if (fixed_rate)
            get_ue_golomb_long(gb);  // elemental_duration_in_tc_minus1
        else
            low_delay = get_bits1(gb);

        if (!low_delay) {
            nb_cpb = get_ue_golomb_long(gb) + 1;
            if (nb_cpb < 1 || nb_cpb > 32) {
                av_log(NULL, AV_LOG_ERROR, "nb_cpb %d invalid\n", nb_cpb);
                return AVERROR_INVALIDDATA;
            }
        }

        if (nal_params_present)
            decode_sublayer_hrd(gb, nb_cpb, subpic_params_present);
        if (vcl_params_present)
            decode_sublayer_hrd(gb, nb_cpb, subpic_params_present);
    }
    return 0;
}

int ff_hevc_mmal_decode_nal_vps(GetBitContext * const gb, AVCodecContext * const avctx,
                           HEVCMMALParamSets * const ps)
{
    int i,j;
    int vps_id = 0;
    ptrdiff_t nal_size;
    HEVCMMALVPS *vps;
    AVBufferRef *vps_buf = av_buffer_allocz(sizeof(*vps));

    if (!vps_buf)
        return AVERROR(ENOMEM);
    vps = (HEVCMMALVPS*)vps_buf->data;

    av_log(avctx, AV_LOG_DEBUG, "Decoding VPS\n");

    nal_size = gb->buffer_end - gb->buffer;
    if (nal_size > sizeof(vps->data)) {
        av_log(avctx, AV_LOG_WARNING, "Truncating likely oversized VPS "
               "(%"PTRDIFF_SPECIFIER" > %"SIZE_SPECIFIER")\n",
               nal_size, sizeof(vps->data));
        vps->data_size = sizeof(vps->data);
    } else {
        vps->data_size = nal_size;
    }
    memcpy(vps->data, gb->buffer, vps->data_size);

    vps_id = get_bits(gb, 4);
    if (vps_id >= HEVC_MAX_VPS_COUNT) {
        av_log(avctx, AV_LOG_ERROR, "VPS id out of range: %d\n", vps_id);
        goto err;
    }

    if (get_bits(gb, 2) != 3) { // vps_reserved_three_2bits
        av_log(avctx, AV_LOG_ERROR, "vps_reserved_three_2bits is not three\n");
        goto err;
    }

    vps->vps_max_layers               = get_bits(gb, 6) + 1;
    vps->vps_max_sub_layers           = get_bits(gb, 3) + 1;
    vps->vps_temporal_id_nesting_flag = get_bits1(gb);

    if (get_bits(gb, 16) != 0xffff) { // vps_reserved_ffff_16bits
        av_log(avctx, AV_LOG_ERROR, "vps_reserved_ffff_16bits is not 0xffff\n");
        goto err;
    }

    if (vps->vps_max_sub_layers > HEVC_MAX_SUB_LAYERS) {
        av_log(avctx, AV_LOG_ERROR, "vps_max_sub_layers out of range: %d\n",
               vps->vps_max_sub_layers);
        goto err;
    }

    if (parse_ptl(gb, avctx, &vps->ptl, vps->vps_max_sub_layers) < 0)
        goto err;

    vps->vps_sub_layer_ordering_info_present_flag = get_bits1(gb);

    i = vps->vps_sub_layer_ordering_info_present_flag ? 0 : vps->vps_max_sub_layers - 1;
    for (; i < vps->vps_max_sub_layers; i++) {
        vps->vps_max_dec_pic_buffering[i] = get_ue_golomb_long(gb) + 1;
        vps->vps_num_reorder_pics[i]      = get_ue_golomb_long(gb);
        vps->vps_max_latency_increase[i]  = get_ue_golomb_long(gb) - 1;

        if (vps->vps_max_dec_pic_buffering[i] > HEVC_MAX_DPB_SIZE || !vps->vps_max_dec_pic_buffering[i]) {
            av_log(avctx, AV_LOG_ERROR, "vps_max_dec_pic_buffering_minus1 out of range: %d\n",
                   vps->vps_max_dec_pic_buffering[i] - 1);
            goto err;
        }
        if (vps->vps_num_reorder_pics[i] > vps->vps_max_dec_pic_buffering[i] - 1) {
            av_log(avctx, AV_LOG_WARNING, "vps_max_num_reorder_pics out of range: %d\n",
                   vps->vps_num_reorder_pics[i]);
            if (avctx->err_recognition & AV_EF_EXPLODE)
                goto err;
        }
    }

    vps->vps_max_layer_id   = get_bits(gb, 6);
    vps->vps_num_layer_sets = get_ue_golomb_long(gb) + 1;
    if (vps->vps_num_layer_sets < 1 || vps->vps_num_layer_sets > 1024 ||
        (vps->vps_num_layer_sets - 1LL) * (vps->vps_max_layer_id + 1LL) > get_bits_left(gb)) {
        av_log(avctx, AV_LOG_ERROR, "too many layer_id_included_flags\n");
        goto err;
    }

    for (i = 1; i < vps->vps_num_layer_sets; i++)
        for (j = 0; j <= vps->vps_max_layer_id; j++)
            skip_bits(gb, 1);  // layer_id_included_flag[i][j]

    vps->vps_timing_info_present_flag = get_bits1(gb);
    if (vps->vps_timing_info_present_flag) {
        vps->vps_num_units_in_tick               = get_bits_long(gb, 32);
        vps->vps_time_scale                      = get_bits_long(gb, 32);
        vps->vps_poc_proportional_to_timing_flag = get_bits1(gb);
        if (vps->vps_poc_proportional_to_timing_flag)
            vps->vps_num_ticks_poc_diff_one = get_ue_golomb_long(gb) + 1;
        vps->vps_num_hrd_parameters = get_ue_golomb_long(gb);
        if (vps->vps_num_hrd_parameters > (unsigned)vps->vps_num_layer_sets) {
            av_log(avctx, AV_LOG_ERROR,
                   "vps_num_hrd_parameters %d is invalid\n", vps->vps_num_hrd_parameters);
            goto err;
        }
        for (i = 0; i < vps->vps_num_hrd_parameters; i++) {
            int common_inf_present = 1;

            get_ue_golomb_long(gb); // hrd_layer_set_idx
            if (i)
                common_inf_present = get_bits1(gb);
            decode_hrd(gb, common_inf_present, vps->vps_max_sub_layers);
        }
    }
    get_bits1(gb); /* vps_extension_flag */

    if (get_bits_left(gb) < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "Overread VPS by %d bits\n", -get_bits_left(gb));
        if (ps->vps_list[vps_id])
            goto err;
    }

    if (ps->vps_list[vps_id] &&
        !memcmp(ps->vps_list[vps_id]->data, vps_buf->data, vps_buf->size)) {
        av_buffer_unref(&vps_buf);
    } else {
        remove_vps(ps, vps_id);
        ps->vps_list[vps_id] = vps_buf;
    }

    return 0;

err:
    av_buffer_unref(&vps_buf);
    return AVERROR_INVALIDDATA;
}

static void decode_vui(GetBitContext * const gb, AVCodecContext * const avctx,
                       const int apply_defdispwin, HEVCMMALSPS * const sps)
{
    VUI backup_vui, * const vui = &sps->vui;
    GetBitContext backup;
    int sar_present, alt = 0;

    av_log(avctx, AV_LOG_DEBUG, "Decoding VUI\n");

    sar_present = get_bits1(gb);
    if (sar_present) {
        uint8_t sar_idx = get_bits(gb, 8);
        if (sar_idx < FF_ARRAY_ELEMS(vui_sar))
            vui->sar = vui_sar[sar_idx];
        else if (sar_idx == 255) {
            vui->sar.num = get_bits(gb, 16);
            vui->sar.den = get_bits(gb, 16);
        } else
            av_log(avctx, AV_LOG_WARNING,
                   "Unknown SAR index: %u.\n", sar_idx);
    }

    vui->overscan_info_present_flag = get_bits1(gb);
    if (vui->overscan_info_present_flag)
        vui->overscan_appropriate_flag = get_bits1(gb);

    vui->video_signal_type_present_flag = get_bits1(gb);
    if (vui->video_signal_type_present_flag) {
        vui->video_format                    = get_bits(gb, 3);
        vui->video_full_range_flag           = get_bits1(gb);
        vui->colour_description_present_flag = get_bits1(gb);
        if (vui->video_full_range_flag && sps->pix_fmt == AV_PIX_FMT_YUV420P)
            sps->pix_fmt = AV_PIX_FMT_YUVJ420P;
        if (vui->colour_description_present_flag) {
            vui->colour_primaries        = get_bits(gb, 8);
            vui->transfer_characteristic = get_bits(gb, 8);
            vui->matrix_coeffs           = get_bits(gb, 8);

            // Set invalid values to "unspecified"
            if (!av_color_primaries_name(vui->colour_primaries))
                vui->colour_primaries = AVCOL_PRI_UNSPECIFIED;
            if (!av_color_transfer_name(vui->transfer_characteristic))
                vui->transfer_characteristic = AVCOL_TRC_UNSPECIFIED;
            if (!av_color_space_name(vui->matrix_coeffs))
                vui->matrix_coeffs = AVCOL_SPC_UNSPECIFIED;
            if (vui->matrix_coeffs == AVCOL_SPC_RGB) {
                switch (sps->pix_fmt) {
                case AV_PIX_FMT_YUV444P:
                    sps->pix_fmt = AV_PIX_FMT_GBRP;
                    break;
                case AV_PIX_FMT_YUV444P10:
                    sps->pix_fmt = AV_PIX_FMT_GBRP10;
                    break;
                case AV_PIX_FMT_YUV444P12:
                    sps->pix_fmt = AV_PIX_FMT_GBRP12;
                    break;
                }
            }
        }
    }

    vui->chroma_loc_info_present_flag = get_bits1(gb);
    if (vui->chroma_loc_info_present_flag) {
        vui->chroma_sample_loc_type_top_field    = get_ue_golomb_long(gb);
        vui->chroma_sample_loc_type_bottom_field = get_ue_golomb_long(gb);
    }

    vui->neutra_chroma_indication_flag = get_bits1(gb);
    vui->field_seq_flag                = get_bits1(gb);
    vui->frame_field_info_present_flag = get_bits1(gb);

    // Backup context in case an alternate header is detected
    memcpy(&backup, gb, sizeof(backup));
    memcpy(&backup_vui, vui, sizeof(backup_vui));
    if (get_bits_left(gb) >= 68 && show_bits_long(gb, 21) == 0x100000) {
        vui->default_display_window_flag = 0;
        av_log(avctx, AV_LOG_WARNING, "Invalid default display window\n");
    } else
        vui->default_display_window_flag = get_bits1(gb);

    if (vui->default_display_window_flag) {
        int vert_mult  = 1 + (sps->chroma_format_idc < 2);
        int horiz_mult = 1 + (sps->chroma_format_idc < 3);
        vui->def_disp_win.left_offset   = get_ue_golomb_long(gb) * horiz_mult;
        vui->def_disp_win.right_offset  = get_ue_golomb_long(gb) * horiz_mult;
        vui->def_disp_win.top_offset    = get_ue_golomb_long(gb) *  vert_mult;
        vui->def_disp_win.bottom_offset = get_ue_golomb_long(gb) *  vert_mult;

        if (apply_defdispwin &&
            avctx->flags2 & AV_CODEC_FLAG2_IGNORE_CROP) {
            av_log(avctx, AV_LOG_DEBUG,
                   "discarding vui default display window, "
                   "original values are l:%u r:%u t:%u b:%u\n",
                   vui->def_disp_win.left_offset,
                   vui->def_disp_win.right_offset,
                   vui->def_disp_win.top_offset,
                   vui->def_disp_win.bottom_offset);

            vui->def_disp_win.left_offset   =
            vui->def_disp_win.right_offset  =
            vui->def_disp_win.top_offset    =
            vui->def_disp_win.bottom_offset = 0;
        }
    }

timing_info:
    vui->vui_timing_info_present_flag = get_bits1(gb);

    if (vui->vui_timing_info_present_flag) {
        if( get_bits_left(gb) < 66 && !alt) {
            // The alternate syntax seem to have timing info located
            // at where def_disp_win is normally located
            av_log(avctx, AV_LOG_WARNING,
                   "Strange VUI timing information, retrying...\n");
            memcpy(vui, &backup_vui, sizeof(backup_vui));
            memcpy(gb, &backup, sizeof(backup));
            alt = 1;
            goto timing_info;
        }
        vui->vui_num_units_in_tick               = get_bits_long(gb, 32);
        vui->vui_time_scale                      = get_bits_long(gb, 32);
        if (alt) {
            av_log(avctx, AV_LOG_INFO, "Retry got %"PRIu32"/%"PRIu32" fps\n",
                   vui->vui_time_scale, vui->vui_num_units_in_tick);
        }
        vui->vui_poc_proportional_to_timing_flag = get_bits1(gb);
        if (vui->vui_poc_proportional_to_timing_flag)
            vui->vui_num_ticks_poc_diff_one_minus1 = get_ue_golomb_long(gb);
        vui->vui_hrd_parameters_present_flag = get_bits1(gb);
        if (vui->vui_hrd_parameters_present_flag)
            decode_hrd(gb, 1, sps->max_sub_layers);
    }

    vui->bitstream_restriction_flag = get_bits1(gb);
    if (vui->bitstream_restriction_flag) {
        if (get_bits_left(gb) < 8 && !alt) {
            av_log(avctx, AV_LOG_WARNING,
                   "Strange VUI bitstream restriction information, retrying"
                   " from timing information...\n");
            memcpy(vui, &backup_vui, sizeof(backup_vui));
            memcpy(gb, &backup, sizeof(backup));
            alt = 1;
            goto timing_info;
        }
        vui->tiles_fixed_structure_flag              = get_bits1(gb);
        vui->motion_vectors_over_pic_boundaries_flag = get_bits1(gb);
        vui->restricted_ref_pic_lists_flag           = get_bits1(gb);
        vui->min_spatial_segmentation_idc            = get_ue_golomb_long(gb);
        vui->max_bytes_per_pic_denom                 = get_ue_golomb_long(gb);
        vui->max_bits_per_min_cu_denom               = get_ue_golomb_long(gb);
        vui->log2_max_mv_length_horizontal           = get_ue_golomb_long(gb);
        vui->log2_max_mv_length_vertical             = get_ue_golomb_long(gb);
    }

    if (get_bits_left(gb) < 1 && !alt) {
        // XXX: Alternate syntax when sps_range_extension_flag != 0?
        av_log(avctx, AV_LOG_WARNING,
               "Overread in VUI, retrying from timing information...\n");
        memcpy(vui, &backup_vui, sizeof(backup_vui));
        memcpy(gb, &backup, sizeof(backup));
        alt = 1;
        goto timing_info;
    }
}

static void set_default_scaling_list_data(ScalingList * const sl)
{
    int matrixId;

    for (matrixId = 0; matrixId < 6; matrixId++) {
        // 4x4 default is 16
        memset(sl->sl[0][matrixId], 16, 16);
        sl->sl_dc[0][matrixId] = 16; // default for 16x16
        sl->sl_dc[1][matrixId] = 16; // default for 32x32
    }

    memcpy(sl->sl[1][0], default_scaling_list_intra, 64);
    memcpy(sl->sl[1][1], default_scaling_list_intra, 64);
    memcpy(sl->sl[1][2], default_scaling_list_intra, 64);

    memcpy(sl->sl[1][3], default_scaling_list_inter, 64);
    memcpy(sl->sl[1][4], default_scaling_list_inter, 64);
    memcpy(sl->sl[1][5], default_scaling_list_inter, 64);

    memcpy(sl->sl[2][0], default_scaling_list_intra, 64);
    memcpy(sl->sl[2][1], default_scaling_list_intra, 64);
    memcpy(sl->sl[2][2], default_scaling_list_intra, 64);

    memcpy(sl->sl[2][3], default_scaling_list_inter, 64);
    memcpy(sl->sl[2][4], default_scaling_list_inter, 64);
    memcpy(sl->sl[2][5], default_scaling_list_inter, 64);

    memcpy(sl->sl[3][0], default_scaling_list_intra, 64);
    memcpy(sl->sl[3][1], default_scaling_list_intra, 64);
    memcpy(sl->sl[3][2], default_scaling_list_intra, 64);

    memcpy(sl->sl[3][3], default_scaling_list_inter, 64);
    memcpy(sl->sl[3][4], default_scaling_list_inter, 64);
    memcpy(sl->sl[3][5], default_scaling_list_inter, 64);
}

static int scaling_list_data(GetBitContext * const gb, AVCodecContext * const avctx, ScalingList * const sl,
                             const HEVCMMALSPS * const sps)
{
    uint8_t scaling_list_pred_mode_flag;
    int32_t scaling_list_dc_coef[2][6];
    int size_id, matrix_id, pos;
    int i;

    for (size_id = 0; size_id < 4; size_id++)
        for (matrix_id = 0; matrix_id < 6; matrix_id += ((size_id == 3) ? 3 : 1)) {
            scaling_list_pred_mode_flag = get_bits1(gb);
            if (!scaling_list_pred_mode_flag) {
                unsigned int delta = get_ue_golomb_long(gb);
                /* Only need to handle non-zero delta. Zero means default,
                 * which should already be in the arrays. */
                if (delta) {
                    // Copy from previous array.
                    delta *= (size_id == 3) ? 3 : 1;
                    if (matrix_id < delta) {
                        av_log(avctx, AV_LOG_ERROR,
                               "Invalid delta in scaling list data: %d.\n", delta);
                        return AVERROR_INVALIDDATA;
                    }

                    memcpy(sl->sl[size_id][matrix_id],
                           sl->sl[size_id][matrix_id - delta],
                           size_id > 0 ? 64 : 16);
                    if (size_id > 1)
                        sl->sl_dc[size_id - 2][matrix_id] = sl->sl_dc[size_id - 2][matrix_id - delta];
                }
            } else {
                int next_coef, coef_num;
                int32_t scaling_list_delta_coef;

                next_coef = 8;
                coef_num  = FFMIN(64, 1 << (4 + (size_id << 1)));
                if (size_id > 1) {
                    scaling_list_dc_coef[size_id - 2][matrix_id] = get_se_golomb(gb) + 8;
                    next_coef = scaling_list_dc_coef[size_id - 2][matrix_id];
                    sl->sl_dc[size_id - 2][matrix_id] = next_coef;
                }
                for (i = 0; i < coef_num; i++) {
                    if (size_id == 0)
                        pos = 4 * ff_hevc_mmal_diag_scan4x4_y[i] +
                                  ff_hevc_mmal_diag_scan4x4_x[i];
                    else
                        pos = 8 * ff_hevc_mmal_diag_scan8x8_y[i] +
                                  ff_hevc_mmal_diag_scan8x8_x[i];

                    scaling_list_delta_coef = get_se_golomb(gb);
                    next_coef = (next_coef + 256U + scaling_list_delta_coef) % 256;
                    sl->sl[size_id][matrix_id][pos] = next_coef;
                }
            }
        }

    if (sps->chroma_format_idc == 3) {
        for (i = 0; i < 64; i++) {
            sl->sl[3][1][i] = sl->sl[2][1][i];
            sl->sl[3][2][i] = sl->sl[2][2][i];
            sl->sl[3][4][i] = sl->sl[2][4][i];
            sl->sl[3][5][i] = sl->sl[2][5][i];
        }
        sl->sl_dc[1][1] = sl->sl_dc[0][1];
        sl->sl_dc[1][2] = sl->sl_dc[0][2];
        sl->sl_dc[1][4] = sl->sl_dc[0][4];
        sl->sl_dc[1][5] = sl->sl_dc[0][5];
    }


    return 0;
}

static int map_pixel_format(HEVCMMALSPS * const sps)
{
    const int cfmt = sps->chroma_format_idc;

    sps->pix_fmt = AV_PIX_FMT_NONE;
    switch (sps->bit_depth) {
    case 8:
        if (cfmt == 1)
            sps->pix_fmt = AV_PIX_FMT_SAND128;
        break;
    case 10:
        if (cfmt == 1)
            sps->pix_fmt = AV_PIX_FMT_SAND64_10;
        break;
    default:
        break;
    }

    sps->hshift[0] = sps->vshift[0] = 0;
    sps->hshift[2] = sps->hshift[1] = cfmt > 2 ? 0 : 1; // 1 unless 4:4:4
    sps->vshift[2] = sps->vshift[1] = cfmt > 1 ? 0 : 1; // 1 unless 4:4:4 or 4:2:2

    sps->pixel_shift = sps->bit_depth > 8 ? 1 : 0;

    return 0;
}

static int ff_hevc_mmal_parse_sps(HEVCMMALSPS * const sps, GetBitContext * const gb, unsigned int * const sps_id,
                      const int apply_defdispwin, AVBufferRef * const * const vps_list, AVCodecContext * const avctx)
{
    HEVCMMALWindow *ow;
    int ret = 0;
    int log2_diff_max_min_transform_block_size;
    int bit_depth_chroma, start, vui_present, sublayer_ordering_info;
    int i;

    // Coded parameters

    sps->vps_id = get_bits(gb, 4);
    if (sps->vps_id >= HEVC_MAX_VPS_COUNT) {
        av_log(avctx, AV_LOG_ERROR, "VPS id out of range: %d\n", sps->vps_id);
        return AVERROR_INVALIDDATA;
    }

    if (vps_list && !vps_list[sps->vps_id]) {
        av_log(avctx, AV_LOG_ERROR, "VPS %d does not exist\n",
               sps->vps_id);
        return AVERROR_INVALIDDATA;
    }

    sps->max_sub_layers = get_bits(gb, 3) + 1;
    if (sps->max_sub_layers > HEVC_MAX_SUB_LAYERS) {
        av_log(avctx, AV_LOG_ERROR, "sps_max_sub_layers out of range: %d\n",
               sps->max_sub_layers);
        return AVERROR_INVALIDDATA;
    }

    sps->temporal_id_nesting_flag = get_bits(gb, 1);

    if ((ret = parse_ptl(gb, avctx, &sps->ptl, sps->max_sub_layers)) < 0)
        return ret;

    *sps_id = get_ue_golomb_long(gb);
    if (*sps_id >= HEVC_MAX_SPS_COUNT) {
        av_log(avctx, AV_LOG_ERROR, "SPS id out of range: %d\n", *sps_id);
        return AVERROR_INVALIDDATA;
    }

    sps->chroma_format_idc = get_ue_golomb_long(gb);
    if (sps->chroma_format_idc > 3U) {
        av_log(avctx, AV_LOG_ERROR, "chroma_format_idc %d is invalid\n", sps->chroma_format_idc);
        return AVERROR_INVALIDDATA;
    }

    if (sps->chroma_format_idc == 3)
        sps->separate_colour_plane_flag = get_bits1(gb);

    if (sps->separate_colour_plane_flag)
        sps->chroma_format_idc = 0;

    sps->width  = get_ue_golomb_long(gb);
    sps->height = get_ue_golomb_long(gb);
    if ((ret = av_image_check_size(sps->width,
                                   sps->height, 0, avctx)) < 0)
        return ret;

    if (get_bits1(gb)) { // pic_conformance_flag
        int vert_mult  = 1 + (sps->chroma_format_idc < 2);
        int horiz_mult = 1 + (sps->chroma_format_idc < 3);
        sps->pic_conf_win.left_offset   = get_ue_golomb_long(gb) * horiz_mult;
        sps->pic_conf_win.right_offset  = get_ue_golomb_long(gb) * horiz_mult;
        sps->pic_conf_win.top_offset    = get_ue_golomb_long(gb) *  vert_mult;
        sps->pic_conf_win.bottom_offset = get_ue_golomb_long(gb) *  vert_mult;

        if (avctx->flags2 & AV_CODEC_FLAG2_IGNORE_CROP) {
            av_log(avctx, AV_LOG_DEBUG,
                   "discarding sps conformance window, "
                   "original values are l:%u r:%u t:%u b:%u\n",
                   sps->pic_conf_win.left_offset,
                   sps->pic_conf_win.right_offset,
                   sps->pic_conf_win.top_offset,
                   sps->pic_conf_win.bottom_offset);

            sps->pic_conf_win.left_offset   =
            sps->pic_conf_win.right_offset  =
            sps->pic_conf_win.top_offset    =
            sps->pic_conf_win.bottom_offset = 0;
        }
        sps->output_window = sps->pic_conf_win;
    }

    sps->bit_depth   = get_ue_golomb_long(gb) + 8;
    bit_depth_chroma = get_ue_golomb_long(gb) + 8;
    if (sps->chroma_format_idc && bit_depth_chroma != sps->bit_depth) {
        av_log(avctx, AV_LOG_ERROR,
               "Luma bit depth (%d) is different from chroma bit depth (%d), "
               "this is unsupported.\n",
               sps->bit_depth, bit_depth_chroma);
        return AVERROR_INVALIDDATA;
    }

    ret = map_pixel_format(sps);
    if (ret < 0)
        return ret;

    sps->log2_max_poc_lsb = get_ue_golomb_long(gb) + 4;
    if (sps->log2_max_poc_lsb > 16) {
        av_log(avctx, AV_LOG_ERROR, "log2_max_pic_order_cnt_lsb_minus4 out range: %d\n",
               sps->log2_max_poc_lsb - 4);
        return AVERROR_INVALIDDATA;
    }

    sublayer_ordering_info = get_bits1(gb);
    start = sublayer_ordering_info ? 0 : sps->max_sub_layers - 1;
    for (i = start; i < sps->max_sub_layers; i++) {
        sps->temporal_layer[i].max_dec_pic_buffering = get_ue_golomb_long(gb) + 1;
        sps->temporal_layer[i].num_reorder_pics      = get_ue_golomb_long(gb);
        sps->temporal_layer[i].max_latency_increase  = get_ue_golomb_long(gb) - 1;
        if (sps->temporal_layer[i].max_dec_pic_buffering > (unsigned)HEVC_MAX_DPB_SIZE) {
            av_log(avctx, AV_LOG_ERROR, "sps_max_dec_pic_buffering_minus1 out of range: %d\n",
                   sps->temporal_layer[i].max_dec_pic_buffering - 1U);
            return AVERROR_INVALIDDATA;
        }
        if (sps->temporal_layer[i].num_reorder_pics > sps->temporal_layer[i].max_dec_pic_buffering - 1) {
            av_log(avctx, AV_LOG_WARNING, "sps_max_num_reorder_pics out of range: %d\n",
                   sps->temporal_layer[i].num_reorder_pics);
            if (avctx->err_recognition & AV_EF_EXPLODE ||
                sps->temporal_layer[i].num_reorder_pics > HEVC_MAX_DPB_SIZE - 1) {
                return AVERROR_INVALIDDATA;
            }
            sps->temporal_layer[i].max_dec_pic_buffering = sps->temporal_layer[i].num_reorder_pics + 1;
        }
    }

    if (!sublayer_ordering_info) {
        for (i = 0; i < start; i++) {
            sps->temporal_layer[i].max_dec_pic_buffering = sps->temporal_layer[start].max_dec_pic_buffering;
            sps->temporal_layer[i].num_reorder_pics      = sps->temporal_layer[start].num_reorder_pics;
            sps->temporal_layer[i].max_latency_increase  = sps->temporal_layer[start].max_latency_increase;
        }
    }

    sps->log2_min_cb_size                    = get_ue_golomb_long(gb) + 3;
    sps->log2_diff_max_min_coding_block_size = get_ue_golomb_long(gb);
    sps->log2_min_tb_size                    = get_ue_golomb_long(gb) + 2;
    log2_diff_max_min_transform_block_size   = get_ue_golomb_long(gb);
    sps->log2_max_trafo_size                 = log2_diff_max_min_transform_block_size +
                                               sps->log2_min_tb_size;

    if (sps->log2_min_cb_size < 3 || sps->log2_min_cb_size > 30) {
        av_log(avctx, AV_LOG_ERROR, "Invalid value %d for log2_min_cb_size", sps->log2_min_cb_size);
        return AVERROR_INVALIDDATA;
    }

    if (sps->log2_diff_max_min_coding_block_size > 30) {
        av_log(avctx, AV_LOG_ERROR, "Invalid value %d for log2_diff_max_min_coding_block_size", sps->log2_diff_max_min_coding_block_size);
        return AVERROR_INVALIDDATA;
    }

    if (sps->log2_min_tb_size >= sps->log2_min_cb_size || sps->log2_min_tb_size < 2) {
        av_log(avctx, AV_LOG_ERROR, "Invalid value for log2_min_tb_size");
        return AVERROR_INVALIDDATA;
    }

    if (log2_diff_max_min_transform_block_size < 0 || log2_diff_max_min_transform_block_size > 30) {
        av_log(avctx, AV_LOG_ERROR, "Invalid value %d for log2_diff_max_min_transform_block_size", log2_diff_max_min_transform_block_size);
        return AVERROR_INVALIDDATA;
    }

    {
        const unsigned int CtbLog2SizeY = sps->log2_min_cb_size + sps->log2_diff_max_min_coding_block_size;
        // Not a bitstream limitation, but all profiles
        if (CtbLog2SizeY < 4 || CtbLog2SizeY > HEVC_MAX_LOG2_CTB_SIZE) {
            av_log(avctx, AV_LOG_ERROR, "Invalid value %d for CtbLog2SizeY", CtbLog2SizeY);
            return AVERROR_INVALIDDATA;
        }

        if (sps->log2_max_trafo_size > FFMIN(5, CtbLog2SizeY)) {
            av_log(avctx, AV_LOG_ERROR, "Invalid value %d for MaxTbLog2SizeY", sps->log2_max_trafo_size);
            return AVERROR_INVALIDDATA;
        }

        // Inferred parameters
        sps->log2_ctb_size = CtbLog2SizeY;
//        sps->log2_min_pu_size = sps->log2_min_cb_size - 1;
    }

    sps->max_transform_hierarchy_depth_inter = get_ue_golomb_long(gb);
    sps->max_transform_hierarchy_depth_intra = get_ue_golomb_long(gb);

    sps->scaling_list_enable_flag = get_bits1(gb);
    if (sps->scaling_list_enable_flag) {
        set_default_scaling_list_data(&sps->scaling_list);

        if (get_bits1(gb)) {
            ret = scaling_list_data(gb, avctx, &sps->scaling_list, sps);
            if (ret < 0)
                return ret;
        }
    }

    sps->amp_enabled_flag = get_bits1(gb);
    sps->sao_enabled      = get_bits1(gb);

    // Set pcm defaults (0) so we don't have to test _enabled when we
    // want to use them
    memset(&sps->pcm, 0, sizeof(sps->pcm));

    if (get_bits1(gb))  // pcm_enabled_flag
    {
        const unsigned int limit_max_pcm = FFMIN(5,
            sps->log2_min_cb_size + sps->log2_diff_max_min_coding_block_size);
        sps->pcm.bit_depth   = get_bits(gb, 4) + 1;
        sps->pcm.bit_depth_chroma = get_bits(gb, 4) + 1;
        sps->pcm.log2_min_pcm_cb_size = get_ue_golomb_long(gb) + 3;
        sps->pcm.log2_max_pcm_cb_size = sps->pcm.log2_min_pcm_cb_size +
                                        get_ue_golomb_long(gb);
        if (FFMAX(sps->pcm.bit_depth, sps->pcm.bit_depth_chroma) > sps->bit_depth) {
            av_log(avctx, AV_LOG_ERROR,
                   "PCM bit depth (%d, %d) is greater than normal bit depth (%d)\n",
                   sps->pcm.bit_depth, sps->pcm.bit_depth_chroma, sps->bit_depth);
            return AVERROR_INVALIDDATA;
        }
        if (sps->pcm.log2_min_pcm_cb_size < sps->log2_min_cb_size ||
            sps->pcm.log2_max_pcm_cb_size > limit_max_pcm) {
            av_log(avctx, AV_LOG_ERROR, "Bad PCM CB min/max size (%d->%d)",
                   sps->pcm.log2_min_pcm_cb_size, sps->pcm.log2_max_pcm_cb_size);
            return AVERROR_INVALIDDATA;
        }

        sps->pcm.loop_filter_disable_flag = get_bits1(gb);
    }

    // Could be based on min_pcm_cb_size but much easier logic if we just stick
    // with 8 (and costs us little)
    sps->pcm_width = (sps->width + 63) >> 6;  // 8 for min size, 8 bits per byte - round up
    sps->pcm_height = (sps->height + 7) >> 3;

    sps->nb_st_rps = get_ue_golomb_long(gb);
    if (sps->nb_st_rps > HEVC_MAX_SHORT_TERM_REF_PIC_SETS) {
        av_log(avctx, AV_LOG_ERROR, "Too many short term RPS: %d.\n",
               sps->nb_st_rps);
        return AVERROR_INVALIDDATA;
    }
    for (i = 0; i < sps->nb_st_rps; i++) {
        if ((ret = ff_hevc_mmal_decode_short_term_rps(gb, avctx, &sps->st_rps[i],
                                                 sps, 0)) < 0)
            return ret;
    }

    sps->long_term_ref_pics_present_flag = get_bits1(gb);
    if (sps->long_term_ref_pics_present_flag) {
        sps->num_long_term_ref_pics_sps = get_ue_golomb_long(gb);
        if (sps->num_long_term_ref_pics_sps > HEVC_MAX_LONG_TERM_REF_PICS) {
            av_log(avctx, AV_LOG_ERROR, "num_long_term_ref_pics_sps %d is out of range.\n",
                   sps->num_long_term_ref_pics_sps);
            return AVERROR_INVALIDDATA;
        }
        for (i = 0; i < sps->num_long_term_ref_pics_sps; i++) {
            sps->lt_ref_pic_poc_lsb_sps[i]       = get_bits(gb, sps->log2_max_poc_lsb);
            sps->used_by_curr_pic_lt_sps_flag[i] = get_bits1(gb);
        }
    }

    sps->sps_temporal_mvp_enabled_flag          = get_bits1(gb);
    sps->intra_filters_disable = get_bits1(gb) ? 0 : FILTER_STRONG; // sps->sps_strong_intra_smoothing_enable_flag
    sps->vui.sar = (AVRational){0, 1};
    vui_present = get_bits1(gb);
    if (vui_present)
        decode_vui(gb, avctx, apply_defdispwin, sps);

    if (get_bits1(gb)) { // sps_extension_flag
        int sps_extension_flag[1];
        for (i = 0; i < 1; i++)
            sps_extension_flag[i] = get_bits1(gb);
        skip_bits(gb, 7); //sps_extension_7bits = get_bits(gb, 7);
        if (sps_extension_flag[0]) {
            int extended_precision_processing_flag;
            int cabac_bypass_alignment_enabled_flag;

            sps->transform_skip_rotation_enabled_flag = get_bits1(gb);
            sps->transform_skip_context_enabled_flag  = get_bits1(gb);
            sps->implicit_rdpcm_enabled_flag = get_bits1(gb);

            sps->explicit_rdpcm_enabled_flag = get_bits1(gb);

            extended_precision_processing_flag = get_bits1(gb);
            if (extended_precision_processing_flag)
                av_log(avctx, AV_LOG_WARNING,
                   "extended_precision_processing_flag not yet implemented\n");

            if (get_bits1(gb))          // sps->intra_smoothing_disabled_flag
                sps->intra_filters_disable |= FILTER_EITHER;
            sps->high_precision_offsets_enabled_flag = get_bits1(gb);
            sps->persistent_rice_adaptation_enabled_flag = get_bits1(gb);

            cabac_bypass_alignment_enabled_flag  = get_bits1(gb);
            if (cabac_bypass_alignment_enabled_flag)
                av_log(avctx, AV_LOG_WARNING,
                   "cabac_bypass_alignment_enabled_flag not yet implemented\n");
        }
    }
    if (apply_defdispwin) {
        sps->output_window.left_offset   += sps->vui.def_disp_win.left_offset;
        sps->output_window.right_offset  += sps->vui.def_disp_win.right_offset;
        sps->output_window.top_offset    += sps->vui.def_disp_win.top_offset;
        sps->output_window.bottom_offset += sps->vui.def_disp_win.bottom_offset;
    }

    ow = &sps->output_window;
    if (ow->left_offset >= INT_MAX - ow->right_offset     ||
        ow->top_offset  >= INT_MAX - ow->bottom_offset    ||
        ow->left_offset + ow->right_offset  >= sps->width ||
        ow->top_offset  + ow->bottom_offset >= sps->height) {
        av_log(avctx, AV_LOG_WARNING, "Invalid cropping offsets: %u/%u/%u/%u\n",
               ow->left_offset, ow->right_offset, ow->top_offset, ow->bottom_offset);
        if (avctx->err_recognition & AV_EF_EXPLODE) {
            return AVERROR_INVALIDDATA;
        }
        av_log(avctx, AV_LOG_WARNING,
               "Displaying the whole video surface.\n");
        memset(ow, 0, sizeof(*ow));
        memset(&sps->pic_conf_win, 0, sizeof(sps->pic_conf_win));
    }

    // Inferred parameters

    sps->ctb_width  = (sps->width  + (1 << sps->log2_ctb_size) - 1) >> sps->log2_ctb_size;
    sps->ctb_height = (sps->height + (1 << sps->log2_ctb_size) - 1) >> sps->log2_ctb_size;
    sps->ctb_size   = sps->ctb_width * sps->ctb_height;

    sps->min_cb_width  = sps->width  >> sps->log2_min_cb_size;
    sps->min_cb_height = sps->height >> sps->log2_min_cb_size;
    sps->min_tb_width  = sps->width  >> sps->log2_min_tb_size;
    sps->min_tb_height = sps->height >> sps->log2_min_tb_size;
    sps->min_pu_width  = sps->width  >> LOG2_MIN_PU_SIZE;
    sps->min_pu_height = sps->height >> LOG2_MIN_PU_SIZE;
    sps->tb_mask       = (1 << (sps->log2_ctb_size - sps->log2_min_tb_size)) - 1;

    sps->qp_bd_offset = 6 * (sps->bit_depth - 8);
    sps->wp_offset_half_range = (1U << (sps->high_precision_offsets_enabled_flag ? sps->bit_depth - 1 : 7));

    if (av_mod_uintp2(sps->width, sps->log2_min_cb_size) ||
        av_mod_uintp2(sps->height, sps->log2_min_cb_size)) {
        av_log(avctx, AV_LOG_ERROR, "Invalid coded frame dimensions.\n");
        return AVERROR_INVALIDDATA;
    }

    if (sps->max_transform_hierarchy_depth_inter > sps->log2_ctb_size - sps->log2_min_tb_size) {
        av_log(avctx, AV_LOG_ERROR, "max_transform_hierarchy_depth_inter out of range: %d\n",
               sps->max_transform_hierarchy_depth_inter);
        return AVERROR_INVALIDDATA;
    }
    if (sps->max_transform_hierarchy_depth_intra > sps->log2_ctb_size - sps->log2_min_tb_size) {
        av_log(avctx, AV_LOG_ERROR, "max_transform_hierarchy_depth_intra out of range: %d\n",
               sps->max_transform_hierarchy_depth_intra);
        return AVERROR_INVALIDDATA;
    }
    if (sps->log2_max_trafo_size > FFMIN(sps->log2_ctb_size, 5)) {
        av_log(avctx, AV_LOG_ERROR,
               "max transform block size out of range: %d\n",
               sps->log2_max_trafo_size);
        return AVERROR_INVALIDDATA;
    }

    if (get_bits_left(gb) < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "Overread SPS by %d bits\n", -get_bits_left(gb));
        return AVERROR_INVALIDDATA;
    }

    return 0;
}

int ff_hevc_mmal_decode_nal_sps(GetBitContext *gb, AVCodecContext *avctx,
                           HEVCMMALParamSets *ps, int apply_defdispwin)
{
    HEVCMMALSPS *sps;
    AVBufferRef *sps_buf = av_buffer_allocz(sizeof(*sps));
    unsigned int sps_id;
    int ret;
    ptrdiff_t nal_size;

    if (!sps_buf)
        return AVERROR(ENOMEM);
    sps = (HEVCMMALSPS*)sps_buf->data;

    av_log(avctx, AV_LOG_DEBUG, "Decoding SPS\n");

    nal_size = gb->buffer_end - gb->buffer;
    if (nal_size > sizeof(sps->data)) {
        av_log(avctx, AV_LOG_WARNING, "Truncating likely oversized SPS "
               "(%"PTRDIFF_SPECIFIER" > %"SIZE_SPECIFIER")\n",
               nal_size, sizeof(sps->data));
        sps->data_size = sizeof(sps->data);
    } else {
        sps->data_size = nal_size;
    }
    memcpy(sps->data, gb->buffer, sps->data_size);

    ret = ff_hevc_mmal_parse_sps(sps, gb, &sps_id,
                            apply_defdispwin,
                            ps->vps_list, avctx);
    if (ret < 0) {
        av_buffer_unref(&sps_buf);
        return ret;
    }

    if (avctx->debug & FF_DEBUG_BITSTREAM) {
        av_log(avctx, AV_LOG_DEBUG,
               "Parsed SPS: id %d; coded wxh: %dx%d; "
               "cropped wxh: %dx%d; pix_fmt: %s.\n",
               sps_id, sps->width, sps->height,
               sps->width - (sps->output_window.left_offset + sps->output_window.right_offset),
               sps->height - (sps->output_window.top_offset + sps->output_window.bottom_offset),
               av_get_pix_fmt_name(sps->pix_fmt));
    }

    /* check if this is a repeat of an already parsed SPS, then keep the
     * original one.
     * otherwise drop all PPSes that depend on it */
    if (ps->sps_list[sps_id] &&
        !memcmp(ps->sps_list[sps_id]->data, sps_buf->data, sps_buf->size)) {
        av_buffer_unref(&sps_buf);
    } else {
        remove_sps(ps, sps_id);
        ps->sps_list[sps_id] = sps_buf;
    }

    return 0;
}

static void hevc_pps_free(void *opaque, uint8_t *data)
{
    HEVCMMALPPS *pps = (HEVCMMALPPS*)data;

    av_freep(&pps->column_width);
    av_freep(&pps->row_height);
    av_freep(&pps->col_bd);
    av_freep(&pps->row_bd);
    av_freep(&pps->col_idxX);
    av_freep(&pps->ctb_addr_rs_to_ts);
    av_freep(&pps->ctb_addr_ts_to_rs);
    av_freep(&pps->tile_pos_ts);
    av_freep(&pps->tile_size);
    av_freep(&pps->tile_id);
    av_freep(&pps->ctb_ts_flags);

    av_freep(&pps);
}

static int get_offset_list(GetBitContext * const gb, AVCodecContext * const avctx, unsigned int n_minus_1, int8_t * offsets)
{
    do
    {
        const int offset = get_se_golomb_long(gb);
        if (offset < -12 || offset > 12) {
            av_log(avctx, AV_LOG_ERROR, "qp_offset_list[]: %d out of range\n", offset);
            return AVERROR_INVALIDDATA;
        }
        *offsets++ = offset;
    } while (n_minus_1-- != 0);
    return 0;
}

static int pps_range_extensions(GetBitContext * const gb, AVCodecContext * const avctx,
                                HEVCMMALPPS * const pps, const HEVCMMALSPS * const sps)
{
    if (pps->transform_skip_enabled_flag) {
        pps->log2_max_transform_skip_block_size = get_ue_golomb_long(gb) + 2;
    }
    pps->cross_component_prediction_enabled_flag = get_bits1(gb);
    if (pps->cross_component_prediction_enabled_flag &&
        (sps->chroma_format_idc != 3 || sps->separate_colour_plane_flag))
    {
        av_log(avctx, AV_LOG_ERROR, "cross_component_prediction_enabled but chroma_format_idc != 3\n");
        return AVERROR_INVALIDDATA;
    }
    pps->chroma_qp_offset_list_enabled_flag = get_bits1(gb);
    if (pps->chroma_qp_offset_list_enabled_flag) {
        int err;

        pps->diff_cu_chroma_qp_offset_depth = get_ue_golomb_long(gb);
        pps->chroma_qp_offset_list_len_minus1 = get_ue_golomb_long(gb);
        if (pps->chroma_qp_offset_list_len_minus1 > 5) {
            av_log(avctx, AV_LOG_ERROR,
                   "chroma_qp_offset_list_len_minus1 shall be in the range [0, 5].\n");
            return AVERROR_INVALIDDATA;
        }
        av_log(avctx, AV_LOG_WARNING, "cb_qp_offset_list not tested yet.\n");

        if ((err = get_offset_list(gb, avctx, pps->chroma_qp_offset_list_len_minus1, pps->cb_qp_offset_list)) != 0 ||
            (err = get_offset_list(gb, avctx, pps->chroma_qp_offset_list_len_minus1, pps->cr_qp_offset_list)) != 0)
            return err;
    }

    {
        const unsigned int max_offset = sps->bit_depth > 10 ? sps->bit_depth - 10 : 0;

        pps->log2_sao_offset_scale_luma = get_ue_golomb_long(gb);
        if (pps->log2_sao_offset_scale_luma > max_offset) {
            av_log(avctx, AV_LOG_ERROR, "log2_sao_offset_scale_luma invalid");
            return AVERROR_INVALIDDATA;
        }
        pps->log2_sao_offset_scale_chroma = get_ue_golomb_long(gb);
        if (pps->log2_sao_offset_scale_chroma > max_offset) {
            av_log(avctx, AV_LOG_ERROR, "log2_sao_offset_scale_chroma invalid");
            return AVERROR_INVALIDDATA;
        }
    }

    return(0);
}

static inline int setup_pps(AVCodecContext * const avctx,
                            HEVCMMALPPS * const pps, const HEVCMMALSPS * const sps)
{
    int pic_area_in_ctbs;
    int i, j, x, y, ctb_addr_rs, tile_id;

    // Inferred parameters

    // qp_y -> qp_u/qp_v tables
    // The tables have at least -24,+24 overrun after adding offset here
    // which should allow for clipless offseting

    pps->qp_dblk_x[0] = qp_c_dblk_0 + QP_DBLK_OFFSET_0;  // No offset for luma, but may be useful for general code
    pps->qp_bd_x[0] = qp_c_bd_0[sps->bit_depth - 8] + QP_OFFSET_0;

    if (sps->chroma_format_idc == 1) {
        pps->qp_dblk_x[1] = qp_c_dblk_1 + pps->cb_qp_offset + QP_DBLK_OFFSET_0;
        pps->qp_bd_x[1] = qp_c_bd_1[sps->bit_depth - 8] + pps->cb_qp_offset + QP_OFFSET_0;
        pps->qp_dblk_x[2] = qp_c_dblk_1 + pps->cr_qp_offset + QP_DBLK_OFFSET_0;
        pps->qp_bd_x[2] = qp_c_bd_1[sps->bit_depth - 8] + pps->cr_qp_offset + QP_OFFSET_0;
    }
    else
    {
        pps->qp_dblk_x[1] = qp_c_dblk_0 + pps->cb_qp_offset + QP_DBLK_OFFSET_0;
        pps->qp_bd_x[1] = qp_c_bd_0[sps->bit_depth - 8] + pps->cb_qp_offset + QP_OFFSET_0;
        pps->qp_dblk_x[2] = qp_c_dblk_0 + pps->cr_qp_offset + QP_DBLK_OFFSET_0;
        pps->qp_bd_x[2] = qp_c_bd_0[sps->bit_depth - 8] + pps->cr_qp_offset + QP_OFFSET_0;
    }

    pps->col_bd   = av_malloc_array(pps->num_tile_columns + 1, sizeof(*pps->col_bd));
    pps->row_bd   = av_malloc_array(pps->num_tile_rows + 1,    sizeof(*pps->row_bd));
    pps->col_idxX = av_malloc_array(sps->ctb_width,    sizeof(*pps->col_idxX));
    if (!pps->col_bd || !pps->row_bd || !pps->col_idxX)
        return AVERROR(ENOMEM);

    if (pps->uniform_spacing_flag) {
        if (!pps->column_width) {
            pps->column_width = av_malloc_array(pps->num_tile_columns, sizeof(*pps->column_width));
            pps->row_height   = av_malloc_array(pps->num_tile_rows,    sizeof(*pps->row_height));
        }
        if (!pps->column_width || !pps->row_height)
            return AVERROR(ENOMEM);

        for (i = 0; i < pps->num_tile_columns; i++) {
            pps->column_width[i] = ((i + 1) * sps->ctb_width) / pps->num_tile_columns -
                                   (i * sps->ctb_width) / pps->num_tile_columns;
        }

        for (i = 0; i < pps->num_tile_rows; i++) {
            pps->row_height[i] = ((i + 1) * sps->ctb_height) / pps->num_tile_rows -
                                 (i * sps->ctb_height) / pps->num_tile_rows;
        }
    }

    {
        const unsigned int td_mask = 63 >> (sps->log2_ctb_size + sps->pixel_shift);
        pps->col_bd[0] = 0;
        pps->tile_wpp_inter_disable = 0;
        for (i = 0; i < pps->num_tile_columns; i++)
        {
            pps->col_bd[i + 1] = pps->col_bd[i] + pps->column_width[i];

            // Avoid trying tile parallel if the columns don't fall on cache boundries
            // (this causes too much pain syncing flushes with the QPU)
            // Ignore the final (RHS of pic) tile boundry
            if ((pps->col_bd[i] & td_mask) != 0) {
                pps->tile_wpp_inter_disable = 1;
            }
        }

        // If we can start the next row before finishing the first line of
        // this one then we must wait at the end of the tile
        // * if this happens a lot then there are better but more complicated
        //   conditions that we could apply
        if (pps->tile_wpp_inter_disable) {
            for (i = 0; i < pps->num_tile_rows; i++)
            {
                if (pps->row_height[i] <= MMAL_MAX_JOBS) {
                    pps->tile_wpp_inter_disable = 2;
                    break;
                }
            }
        }
    }

    pps->row_bd[0] = 0;
    for (i = 0; i < pps->num_tile_rows; i++)
        pps->row_bd[i + 1] = pps->row_bd[i] + pps->row_height[i];

    for (i = 0, j = 0; i < sps->ctb_width; i++) {
        if (i >= pps->col_bd[j + 1])
            j++;
        pps->col_idxX[i] = j;
    }

    /**
     * 6.5
     */
    pic_area_in_ctbs     = sps->ctb_size;

    pps->ctb_addr_rs_to_ts = av_malloc_array(pic_area_in_ctbs,    sizeof(*pps->ctb_addr_rs_to_ts));
    pps->ctb_addr_ts_to_rs = av_malloc_array(pic_area_in_ctbs,    sizeof(*pps->ctb_addr_ts_to_rs));
    pps->tile_id           = av_malloc_array(pic_area_in_ctbs,    sizeof(*pps->tile_id));
    pps->tile_size         = av_malloc_array(pps->num_tile_columns * pps->num_tile_rows, sizeof(*pps->tile_size));
    pps->tile_pos_ts       = av_malloc_array(pps->num_tile_columns * pps->num_tile_rows, sizeof(*pps->tile_pos_ts));
    pps->ctb_ts_flags      = av_malloc_array(pic_area_in_ctbs,    sizeof(*pps->ctb_ts_flags));
    if (!pps->ctb_addr_rs_to_ts || !pps->ctb_addr_ts_to_rs ||
        !pps->tile_id || pps->tile_pos_ts == NULL || pps->tile_size == NULL) {
        return AVERROR(ENOMEM);
    }

    memset(pps->ctb_ts_flags, 0, pic_area_in_ctbs * sizeof(*pps->ctb_ts_flags));

    for (ctb_addr_rs = 0; ctb_addr_rs < pic_area_in_ctbs; ctb_addr_rs++) {
        int tb_x   = ctb_addr_rs % sps->ctb_width;
        int tb_y   = ctb_addr_rs / sps->ctb_width;
        int tile_x = 0;
        int tile_y = 0;
        int val    = 0;

        for (i = 0; i < pps->num_tile_columns; i++) {
            if (tb_x < pps->col_bd[i + 1]) {
                tile_x = i;
                break;
            }
        }

        for (i = 0; i < pps->num_tile_rows; i++) {
            if (tb_y < pps->row_bd[i + 1]) {
                tile_y = i;
                break;
            }
        }

        for (i = 0; i < tile_x; i++)
            val += pps->row_height[tile_y] * pps->column_width[i];
        for (i = 0; i < tile_y; i++)
            val += sps->ctb_width * pps->row_height[i];

        val += (tb_y - pps->row_bd[tile_y]) * pps->column_width[tile_x] +
               tb_x - pps->col_bd[tile_x];

        pps->ctb_addr_rs_to_ts[ctb_addr_rs] = val;
        pps->ctb_addr_ts_to_rs[val]         = ctb_addr_rs;
    }

    {
        uint8_t * pflags = pps->ctb_ts_flags;
        uint16_t * ptid = pps->tile_id;

        for (j = 0, tile_id = 0; j < pps->num_tile_rows; j++)
        {
            for (i = 0; i < pps->num_tile_columns; i++, tile_id++)
            {
                const unsigned int tile_w = pps->column_width[i];

                pflags[0] |= CTB_TS_FLAGS_CIREQ;

                for (x = 0; x != tile_w; ++x) {
                    pflags[x] |= CTB_TS_FLAGS_TOT;
                }

                for (y = pps->row_bd[j]; y < pps->row_bd[j + 1]; y++)
                {
                    pflags[0] |= CTB_TS_FLAGS_SOTL;

                    if (pps->entropy_coding_sync_enabled_flag)
                    {
                        if (pps->column_width[i] != 1)
                            pflags[1] |= CTB_TS_FLAGS_CSAVE;
                        else
                            pflags[0] |= CTB_TS_FLAGS_CIREQ;

                        if ((pflags[0] & CTB_TS_FLAGS_CIREQ) == 0)
                            pflags[0] |= CTB_TS_FLAGS_CLOAD;
                    }

                    for (x = 0; x != tile_w; ++x)
                        *ptid++ = tile_id;

                    pflags += tile_w;
                    pflags[-1] |= CTB_TS_FLAGS_EOTL;
                    if (i + 1 == pps->num_tile_columns)
                        pflags[-1] |= CTB_TS_FLAGS_EOL;
                }

                pflags[-1] |= CTB_TS_FLAGS_EOT;
            }
        }
    }

    {
        unsigned int ts = 0;
        for (j = 0; j < pps->num_tile_rows; j++)
            for (i = 0; i < pps->num_tile_columns; i++)
            {
                const unsigned int size = pps->column_width[i] * pps->row_height[j];
                pps->tile_size[j * pps->num_tile_columns + i] = size;
                pps->tile_pos_ts[j * pps->num_tile_columns + i] = ts;
                ts += size;
            }
    }

    return 0;
}

int ff_hevc_mmal_decode_nal_pps(GetBitContext * const gb, AVCodecContext * const avctx,
                           HEVCMMALParamSets * const ps)
{
    const HEVCMMALSPS *sps = NULL;
    int i, ret = 0;
    unsigned int pps_id = 0;
    ptrdiff_t nal_size;
    unsigned log2_parallel_merge_level_minus2;

    AVBufferRef *pps_buf;
    HEVCMMALPPS *pps = av_mallocz(sizeof(*pps));

    if (!pps)
        return AVERROR(ENOMEM);

    pps_buf = av_buffer_create((uint8_t *)pps, sizeof(*pps),
                               hevc_pps_free, NULL, 0);
    if (!pps_buf) {
        av_freep(&pps);
        return AVERROR(ENOMEM);
    }

    av_log(avctx, AV_LOG_DEBUG, "Decoding PPS\n");

    nal_size = gb->buffer_end - gb->buffer;
    if (nal_size > sizeof(pps->data)) {
        av_log(avctx, AV_LOG_WARNING, "Truncating likely oversized PPS "
               "(%"PTRDIFF_SPECIFIER" > %"SIZE_SPECIFIER")\n",
               nal_size, sizeof(pps->data));
        pps->data_size = sizeof(pps->data);
    } else {
        pps->data_size = nal_size;
    }
    memcpy(pps->data, gb->buffer, pps->data_size);

    // Default values
    pps->loop_filter_across_tiles_enabled_flag = 1;
    pps->num_tile_columns                      = 1;
    pps->num_tile_rows                         = 1;
    pps->uniform_spacing_flag                  = 1;
    pps->disable_dbf                           = 0;
    pps->beta_offset                           = 0;
    pps->tc_offset                             = 0;
    pps->log2_max_transform_skip_block_size    = 2;

    // Coded parameters
    pps_id = get_ue_golomb_long(gb);
    if (pps_id >= HEVC_MAX_PPS_COUNT) {
        av_log(avctx, AV_LOG_ERROR, "PPS id out of range: %d\n", pps_id);
        ret = AVERROR_INVALIDDATA;
        goto err;
    }
    pps->sps_id = get_ue_golomb_long(gb);
    if (pps->sps_id >= HEVC_MAX_SPS_COUNT) {
        av_log(avctx, AV_LOG_ERROR, "SPS id out of range: %d\n", pps->sps_id);
        ret = AVERROR_INVALIDDATA;
        goto err;
    }
    if (!ps->sps_list[pps->sps_id]) {
        av_log(avctx, AV_LOG_ERROR, "SPS %u does not exist.\n", pps->sps_id);
        ret = AVERROR_INVALIDDATA;
        goto err;
    }
    sps = (HEVCMMALSPS *)ps->sps_list[pps->sps_id]->data;

    pps->dependent_slice_segments_enabled_flag = get_bits1(gb);
    pps->output_flag_present_flag              = get_bits1(gb);
    pps->num_extra_slice_header_bits           = get_bits(gb, 3);

    pps->sign_data_hiding_flag = get_bits1(gb);

    pps->cabac_init_present_flag = get_bits1(gb);

    pps->num_ref_idx_l0_default_active = get_ue_golomb_long(gb) + 1;
    if (pps->num_ref_idx_l0_default_active < 1 || pps->num_ref_idx_l0_default_active > 15) {
        av_log(avctx, AV_LOG_ERROR, "pps->num_ref_idx_l0_default_active invalid\n");
        ret = AVERROR_INVALIDDATA;
        goto err;
    }
    pps->num_ref_idx_l1_default_active = get_ue_golomb_long(gb) + 1;
    if (pps->num_ref_idx_l1_default_active < 1 || pps->num_ref_idx_l1_default_active > 15) {
        av_log(avctx, AV_LOG_ERROR, "pps->num_ref_idx_l1_default_active invalid\n");
        ret = AVERROR_INVALIDDATA;
        goto err;
    }

    pps->pic_init_qp_minus26 = get_se_golomb(gb);
    if (pps->pic_init_qp_minus26 > 25 || pps->pic_init_qp_minus26 < -(26 + sps->qp_bd_offset)) {
        av_log(avctx, AV_LOG_ERROR,
               "init_qp_minus26 %d is outside the valid range "
               "[%d, %d].\n",
               pps->pic_init_qp_minus26,
               -(26 + sps->qp_bd_offset), 25);
        ret = AVERROR_INVALIDDATA;
        goto err;
    }

    pps->constrained_intra_pred_flag = get_bits1(gb);
    pps->transform_skip_enabled_flag = get_bits1(gb);

    pps->cu_qp_delta_enabled_flag = get_bits1(gb);
    pps->log2_min_cu_qp_delta_size = sps->log2_ctb_size;
    if (pps->cu_qp_delta_enabled_flag)
    {
        const unsigned int diff_cu_qp_delta_depth = get_ue_golomb_long(gb);

        if (diff_cu_qp_delta_depth > sps->log2_diff_max_min_coding_block_size) {
            av_log(avctx, AV_LOG_ERROR, "diff_cu_qp_delta_depth %d is invalid\n",
                   diff_cu_qp_delta_depth);
            ret = AVERROR_INVALIDDATA;
            goto err;
        }

        pps->log2_min_cu_qp_delta_size = sps->log2_ctb_size - diff_cu_qp_delta_depth;
    }

    pps->cb_qp_offset = get_se_golomb(gb);
    if (pps->cb_qp_offset < -12 || pps->cb_qp_offset > 12) {
        av_log(avctx, AV_LOG_ERROR, "pps_cb_qp_offset out of range: %d\n",
               pps->cb_qp_offset);
        ret = AVERROR_INVALIDDATA;
        goto err;
    }
    pps->cr_qp_offset = get_se_golomb(gb);
    if (pps->cr_qp_offset < -12 || pps->cr_qp_offset > 12) {
        av_log(avctx, AV_LOG_ERROR, "pps_cr_qp_offset out of range: %d\n",
               pps->cr_qp_offset);
        ret = AVERROR_INVALIDDATA;
        goto err;
    }
    pps->pic_slice_level_chroma_qp_offsets_present_flag = get_bits1(gb);

    pps->weighted_pred_flag   = get_bits1(gb);
    pps->weighted_bipred_flag = get_bits1(gb);

    pps->transquant_bypass_enable_flag    = get_bits1(gb);
    pps->tiles_enabled_flag               = get_bits1(gb);
    pps->entropy_coding_sync_enabled_flag = get_bits1(gb);

    if (pps->tiles_enabled_flag) {
        pps->num_tile_columns = get_ue_golomb_long(gb) + 1;
        pps->num_tile_rows    = get_ue_golomb_long(gb) + 1;
        if (pps->num_tile_columns <= 0 ||
            pps->num_tile_columns >= sps->width) {
            av_log(avctx, AV_LOG_ERROR, "num_tile_columns_minus1 out of range: %d\n",
                   pps->num_tile_columns - 1);
            ret = AVERROR_INVALIDDATA;
            goto err;
        }
        if (pps->num_tile_rows <= 0 ||
            pps->num_tile_rows >= sps->height) {
            av_log(avctx, AV_LOG_ERROR, "num_tile_rows_minus1 out of range: %d\n",
                   pps->num_tile_rows - 1);
            ret = AVERROR_INVALIDDATA;
            goto err;
        }

        pps->column_width = av_malloc_array(pps->num_tile_columns, sizeof(*pps->column_width));
        pps->row_height   = av_malloc_array(pps->num_tile_rows,    sizeof(*pps->row_height));
        if (!pps->column_width || !pps->row_height) {
            ret = AVERROR(ENOMEM);
            goto err;
        }

        pps->uniform_spacing_flag = get_bits1(gb);
        if (!pps->uniform_spacing_flag) {
            uint64_t sum = 0;
            for (i = 0; i < pps->num_tile_columns - 1; i++) {
                pps->column_width[i] = get_ue_golomb_long(gb) + 1;
                sum                 += pps->column_width[i];
            }
            if (sum >= sps->ctb_width) {
                av_log(avctx, AV_LOG_ERROR, "Invalid tile widths.\n");
                ret = AVERROR_INVALIDDATA;
                goto err;
            }
            pps->column_width[pps->num_tile_columns - 1] = sps->ctb_width - sum;

            sum = 0;
            for (i = 0; i < pps->num_tile_rows - 1; i++) {
                pps->row_height[i] = get_ue_golomb_long(gb) + 1;
                sum               += pps->row_height[i];
            }
            if (sum >= sps->ctb_height) {
                av_log(avctx, AV_LOG_ERROR, "Invalid tile heights.\n");
                ret = AVERROR_INVALIDDATA;
                goto err;
            }
            pps->row_height[pps->num_tile_rows - 1] = sps->ctb_height - sum;
        }
        pps->loop_filter_across_tiles_enabled_flag = get_bits1(gb);
    }

    pps->seq_loop_filter_across_slices_enabled_flag = get_bits1(gb);

    pps->deblocking_filter_control_present_flag = get_bits1(gb);
    if (pps->deblocking_filter_control_present_flag) {
        pps->deblocking_filter_override_enabled_flag = get_bits1(gb);
        pps->disable_dbf                             = get_bits1(gb);
        if (!pps->disable_dbf) {
            int beta_offset_div2 = get_se_golomb(gb);
            int tc_offset_div2   = get_se_golomb(gb) ;
            if (beta_offset_div2 < -6 || beta_offset_div2 > 6) {
                av_log(avctx, AV_LOG_ERROR, "pps_beta_offset_div2 out of range: %d\n",
                       beta_offset_div2);
                ret = AVERROR_INVALIDDATA;
                goto err;
            }
            if (tc_offset_div2 < -6 || tc_offset_div2 > 6) {
                av_log(avctx, AV_LOG_ERROR, "pps_tc_offset_div2 out of range: %d\n",
                       tc_offset_div2);
                ret = AVERROR_INVALIDDATA;
                goto err;
            }
            pps->beta_offset = 2 * beta_offset_div2;
            pps->tc_offset   = 2 *   tc_offset_div2;
        }
    }

    pps->scaling_list_data_present_flag = get_bits1(gb);
    if (pps->scaling_list_data_present_flag) {
        set_default_scaling_list_data(&pps->scaling_list);
        ret = scaling_list_data(gb, avctx, &pps->scaling_list, sps);
        if (ret < 0)
            goto err;
    }
    pps->lists_modification_present_flag = get_bits1(gb);
    log2_parallel_merge_level_minus2     = get_ue_golomb_long(gb);
    if (log2_parallel_merge_level_minus2 > sps->log2_ctb_size) {
        av_log(avctx, AV_LOG_ERROR, "log2_parallel_merge_level_minus2 out of range: %d\n",
               log2_parallel_merge_level_minus2);
        ret = AVERROR_INVALIDDATA;
        goto err;
    }
    pps->log2_parallel_merge_level       = log2_parallel_merge_level_minus2 + 2;

    pps->slice_header_extension_present_flag = get_bits1(gb);

    if (get_bits1(gb)) { // pps_extension_present_flag
        int pps_range_extensions_flag = get_bits1(gb);
        skip_bits(gb, 7); // pps_extension_7bits
        if (sps->ptl.general_ptl.profile_idc == FF_PROFILE_HEVC_REXT && pps_range_extensions_flag) {
            if ((ret = pps_range_extensions(gb, avctx, pps, sps)) < 0)
                goto err;
        }
    }

    ret = setup_pps(avctx, pps, sps);
    if (ret < 0)
        goto err;

    if (get_bits_left(gb) < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "Overread PPS by %d bits\n", -get_bits_left(gb));
        ret = AVERROR_INVALIDDATA;
        goto err;
    }

    remove_pps(ps, pps_id);
    ps->pps_list[pps_id] = pps_buf;

    return 0;

err:
    av_buffer_unref(&pps_buf);
    return ret;
}

int ff_hevc_mmal_compute_poc(const HEVCMMALSPS *sps, int pocTid0, int poc_lsb, int nal_unit_type)
{
    int max_poc_lsb  = 1 << sps->log2_max_poc_lsb;
    int prev_poc_lsb = pocTid0 % max_poc_lsb;
    int prev_poc_msb = pocTid0 - prev_poc_lsb;
    int poc_msb;

    if (poc_lsb < prev_poc_lsb && prev_poc_lsb - poc_lsb >= max_poc_lsb / 2)
        poc_msb = prev_poc_msb + max_poc_lsb;
    else if (poc_lsb > prev_poc_lsb && poc_lsb - prev_poc_lsb > max_poc_lsb / 2)
        poc_msb = prev_poc_msb - max_poc_lsb;
    else
        poc_msb = prev_poc_msb;

    // For BLA picture types, POCmsb is set to 0.
    if (nal_unit_type == HEVC_NAL_BLA_W_LP   ||
        nal_unit_type == HEVC_NAL_BLA_W_RADL ||
        nal_unit_type == HEVC_NAL_BLA_N_LP)
        poc_msb = 0;

    return poc_msb + poc_lsb;
}
