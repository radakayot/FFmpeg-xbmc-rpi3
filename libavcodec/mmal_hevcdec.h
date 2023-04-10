/*
 * HEVC video decoder
 *
 * Copyright (C) 2012 - 2013 Guillaume Martres
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

#ifndef AVCODEC_MMAL_HEVCDEC_H
#define AVCODEC_MMAL_HEVCDEC_H

#include "config.h"

#include <stdatomic.h>

#include "libavutil/buffer.h"

#include "avcodec.h"
#include "bswapdsp.h"
#include "cabac.h"
#include "get_bits.h"
#include "mmal_hevcpred.h"
#include "h2645_parse.h"
#include "hevc.h"
#include "mmal_hevc_mv.h"
#include "mmal_hevc_ps.h"
#include "mmal_hevc_sei.h"
#include "mmal_hevcdsp.h"
#include "internal.h"
#include "thread.h"
#include "videodsp.h"

#if ARCH_ARM
#include "arm/mmal_hevc_misc_neon.h"
#endif

#define MAX_NB_THREADS 16
#define SHIFT_CTB_WPP 2

//TODO: check if this is really the maximum
#define MAX_TRANSFORM_DEPTH 5

#define MAX_TB_SIZE 32
#define MAX_QP 51
#define DEFAULT_INTRA_TC_OFFSET 2

#define HEVC_CONTEXTS 199

#define MRG_MAX_NUM_CANDS     5

#define HEVC_MAX_CTB_SIZE (1 << HEVC_MAX_LOG2_CTB_SIZE)  // 64

// Size of DPB array
#define HEVC_DPB_ELS            32

#define L0 0
#define L1 1

#define EPEL_EXTRA_BEFORE 1
#define EPEL_EXTRA_AFTER  2
#define EPEL_EXTRA        3
#define QPEL_EXTRA_BEFORE 3
#define QPEL_EXTRA_AFTER  4
#define QPEL_EXTRA        7

#define EDGE_EMU_BUFFER_STRIDE 80

#include <semaphore.h>
#include "mmal_qpu.h"

// Max jobs per frame thread. Actual usage will be limited by the size
// of the global job pool
// ?? Limits
#define MMAL_MAX_JOBS            2

// This is the number of _extra_ bit threads - we will have
// MMAL_EXTRA_BIT_THREADS+1 threads actually doing the processing
//
// 0 is legitimate and will disable our WPP processing
//#define MMAL_EXTRA_BIT_THREADS 0
#define MMAL_EXTRA_BIT_THREADS   2

// Number of separate threads/passes in worker
// 2 and 3 are the currently valid numbers
// At the moment 3 seems fractionally faster
//#define MMAL_PASSES              2
#define MMAL_PASSES              3

// Print out various usage stats
#define MMAL_TSTATS              0

// Define MMAL_COMPRESS_COEFFS to 1 to send coefficients in compressed form
#define MMAL_COMPRESS_COEFFS     1

// Wait for VPU/QPU to finish in worker pass 0
// If 0 then the wait is in pass 1
//
// One might expect the better place to wait would be in pass 1 however
// testing shows that pass 0 produces overall faster decode.
// Interestingly it is QPU/VPU limited streams that seem to suffer
// from pass 1 waits, CPU limited ones tend to show a very mild gain.
// This define exists so it is easy to test this.
#define MMAL_WORKER_WAIT_PASS_0  0

// Use ARM emulation of QPU pred
// These are for debug only as the emulation makes only limited
// effort to be fast
#define MMAL_QPU_EMU_Y           0
#define MMAL_QPU_EMU_C           0

// Max width & height we are prepared to consider
// Sand frame shape calc becomes confused with large frames
// Some buffer alloc also depends on this
#define HEVC_MMAL_MAX_WIDTH      2048
#define HEVC_MMAL_MAX_HEIGHT     1088


// Min CTB size is 16
#define HEVC_MMAL_MAX_CTBS ((HEVC_MMAL_MAX_WIDTH + 15) / 16) * ((HEVC_MMAL_MAX_HEIGHT + 15) / 16)

/**
 * Value of the luma sample at position (x, y) in the 2D array tab.
 */
#define SAMPLE(tab, x, y) ((tab)[(y) * s->sps->width + (x)])
#define SAMPLE_CTB(tab, x, y) ((tab)[(y) * min_cb_width + (x)])

#define IS_IDR(s) ((s)->nal_unit_type == HEVC_NAL_IDR_W_RADL || (s)->nal_unit_type == HEVC_NAL_IDR_N_LP)
#define IS_BLA(s) ((s)->nal_unit_type == HEVC_NAL_BLA_W_RADL || (s)->nal_unit_type == HEVC_NAL_BLA_W_LP || \
                   (s)->nal_unit_type == HEVC_NAL_BLA_N_LP)
#define IS_IRAP(s) ((s)->nal_unit_type >= 16 && (s)->nal_unit_type <= 23)

enum RPSType {
    ST_CURR_BEF = 0,
    ST_CURR_AFT,
    ST_FOLL,
    LT_CURR,
    LT_FOLL,
    NB_RPS_TYPE,
};

enum SyntaxElement {
    SAO_MERGE_FLAG = 0,
    SAO_TYPE_IDX,
    SAO_EO_CLASS,
    SAO_BAND_POSITION,
    SAO_OFFSET_ABS,
    SAO_OFFSET_SIGN,
    END_OF_SLICE_FLAG,
    SPLIT_CODING_UNIT_FLAG,
    CU_TRANSQUANT_BYPASS_FLAG,
    SKIP_FLAG,
    CU_QP_DELTA,
    PRED_MODE_FLAG,
    PART_MODE,
    PCM_FLAG,
    PREV_INTRA_LUMA_PRED_FLAG,
    MPM_IDX,
    REM_INTRA_LUMA_PRED_MODE,
    INTRA_CHROMA_PRED_MODE,
    MERGE_FLAG,
    MERGE_IDX,
    INTER_PRED_IDC,
    REF_IDX_L0,
    REF_IDX_L1,
    ABS_MVD_GREATER0_FLAG,
    ABS_MVD_GREATER1_FLAG,
    ABS_MVD_MINUS2,
    MVD_SIGN_FLAG,
    MVP_LX_FLAG,
    NO_RESIDUAL_DATA_FLAG,
    SPLIT_TRANSFORM_FLAG,
    CBF_LUMA,
    CBF_CB_CR,
    TRANSFORM_SKIP_FLAG,
    EXPLICIT_RDPCM_FLAG,
    EXPLICIT_RDPCM_DIR_FLAG,
    LAST_SIGNIFICANT_COEFF_X_PREFIX,
    LAST_SIGNIFICANT_COEFF_Y_PREFIX,
    LAST_SIGNIFICANT_COEFF_X_SUFFIX,
    LAST_SIGNIFICANT_COEFF_Y_SUFFIX,
    SIGNIFICANT_COEFF_GROUP_FLAG,
    SIGNIFICANT_COEFF_FLAG,
    COEFF_ABS_LEVEL_GREATER1_FLAG,
    COEFF_ABS_LEVEL_GREATER2_FLAG,
    COEFF_ABS_LEVEL_REMAINING,
    COEFF_SIGN_FLAG,
    LOG2_RES_SCALE_ABS,
    RES_SCALE_SIGN_FLAG,
    CU_CHROMA_QP_OFFSET_FLAG,
    CU_CHROMA_QP_OFFSET_IDX,
};

enum PartMode {
    PART_2Nx2N = 0,
    PART_2NxN  = 1,
    PART_Nx2N  = 2,
    PART_NxN   = 3,
    PART_2NxnU = 4,
    PART_2NxnD = 5,
    PART_nLx2N = 6,
    PART_nRx2N = 7,
};

enum PredMode {
    MODE_INTER = 0,
    MODE_INTRA,
    MODE_SKIP,
};

enum InterPredIdc {
    PRED_L0 = 0,
    PRED_L1,
    PRED_BI,
};

enum PredFlag {
    PF_INTRA = 0,
    PF_L0,
    PF_L1,
    PF_BI,
};

enum SAOType {
    SAO_NOT_APPLIED = 0,
    SAO_BAND,
    SAO_EDGE,
    SAO_APPLIED
};

enum SAOEOClass {
    SAO_EO_HORIZ = 0,
    SAO_EO_VERT,
    SAO_EO_135D,
    SAO_EO_45D,
};

enum ScanType {
    SCAN_DIAG = 0,
    SCAN_HORIZ,
    SCAN_VERT,
};

typedef struct RefPicList {
    struct HEVCMMALFrame *ref[HEVC_MAX_REFS];
    int list[HEVC_MAX_REFS];
    uint8_t isLongTerm[HEVC_MAX_REFS];
    int nb_refs;
} RefPicList;

typedef struct RefPicListTab {
    RefPicList refPicList[2];
} RefPicListTab;

typedef struct MMALCodingUnit {
    unsigned int x;             // Passed to deblock
    unsigned int y;
    unsigned int x_split;
    unsigned int y_split;

    enum PredMode pred_mode;    ///< PredMode
    enum PartMode part_mode;    ///< PartMode

    // Inferred parameters
    uint8_t intra_split_flag;   ///< IntraSplitFlag
    uint8_t max_trafo_depth;    ///< MaxTrafoDepth
    uint8_t cu_transquant_bypass_flag;
} MMALCodingUnit;

typedef struct MMALPredictionUnit {
    uint8_t intra_pred_mode[4];
    uint8_t intra_pred_mode_c[4];
    uint8_t chroma_mode_c[4];
    uint8_t merge_flag;
} MMALPredictionUnit;

typedef struct HEVCMMALTransformUnit {
    int8_t cu_qp_delta;

    // Inferred parameters;
    uint8_t intra_pred_mode;
    uint8_t intra_pred_mode_c;
    uint8_t chroma_mode_c;
    uint8_t is_cu_qp_delta_wanted;
    uint8_t cu_chroma_qp_offset_wanted;
    const int8_t * qp_divmod6[3];
} HEVCMMALTransformUnit;

typedef struct DBParams {
    int8_t beta_offset; // -12 to +12
    int8_t tc_offset;   // -12 to +12
} DBParams;

#define HEVC_FRAME_FLAG_OUTPUT    (1 << 0)
#define HEVC_FRAME_FLAG_SHORT_REF (1 << 1)
#define HEVC_FRAME_FLAG_LONG_REF  (1 << 2)
#define HEVC_FRAME_FLAG_BUMPING   (1 << 3)

struct HEVCMMALJob;

typedef struct HEVCMMALFrame {
    AVFrame *frame;
    ThreadFrame tf;
    ColMvField *col_mvf;
    int poc;
    struct HEVCMMALFrame *collocated_ref;

    AVBufferRef *col_mvf_buf;

    /**
     * A sequence counter, so that old frames are output first
     * after a POC reset
     */
    uint16_t sequence;

    /**
     * A combination of HEVC_FRAME_FLAG_*
     */
    uint8_t flags;

    // Entry no in DPB - can be used as a small unique
    // frame identifier (within the current thread)
    uint8_t dpb_no;
} HEVCMMALFrame;

typedef struct HEVCMMALLocalContext {
    HEVCMMALTransformUnit tu;

    CABACContext cc;

    // Vars that allow us to locate everything from just an lc
    struct HEVCMMALContext * context;  // ??? make const ???
    unsigned int lc_n; // lc list el no

    // Job wait links
    struct HEVCMMALLocalContext * jw_next;
    struct HEVCMMALLocalContext * jw_prev;
    struct HEVCMMALLocalContext * ljw_next;
    struct HEVCMMALLocalContext * ljw_prev;
    struct HEVCMMALJob * volatile jw_job;
    sem_t jw_sem;

    // ?? Wrap in structure ??
    sem_t bt_sem_in;
    sem_t * bt_psem_out;
    volatile int bt_terminate;
    unsigned int ts;
    unsigned int bt_last_line;  // Last line in this bit_thread chunk
    unsigned int bt_line_no;
    unsigned int bt_line_width;
    unsigned int bt_line_inc;

    struct HEVCMMALJob * jb0;
    char unit_done;  // Set once we have dealt with this slice
    char bt_is_tile;
    char last_progress_good;
    char cabac_init_req;

    uint8_t cabac_state[HEVC_CONTEXTS];
    uint8_t stat_coeff[4];
    GetBitContext gb;

    uint8_t ct_depth;
    int8_t qp_y;
    int8_t curr_qp_y;
    int8_t qPy_pred;

// N.B. Used by asm (neon) - do not change
#define AVAIL_S_UR  0
#define AVAIL_S_U   1
#define AVAIL_S_UL  2
#define AVAIL_S_L   3
#define AVAIL_S_DL  4

#define AVAIL_U     (1 << AVAIL_S_U)
#define AVAIL_L     (1 << AVAIL_S_L)
#define AVAIL_UL    (1 << AVAIL_S_UL)
#define AVAIL_UR    (1 << AVAIL_S_UR)
#define AVAIL_DL    (1 << AVAIL_S_DL)

// Intra filters - same number space as avail
#define FILTER_LIGHT    0x40
#define FILTER_STRONG   0x80
#define FILTER_EITHER   (FILTER_LIGHT | FILTER_STRONG)

    uint8_t ctb_avail;
    int     end_of_ctb_x;
    int     end_of_ctb_y;

    MMALCodingUnit cu;
    MMALPredictionUnit pu;

#define BOUNDARY_LEFT_SLICE     (1 << 0)
#define BOUNDARY_LEFT_TILE      (1 << 1)
#define BOUNDARY_UPPER_SLICE    (1 << 2)
#define BOUNDARY_UPPER_TILE     (1 << 3)
    /* properties of the boundary of the current CTB for the purposes
     * of the deblocking filter */
    unsigned int boundary_flags;

#define IPM_TAB_SIZE (HEVC_MAX_CTB_SIZE >> LOG2_MIN_PU_SIZE)
    uint8_t ipm_left[IPM_TAB_SIZE];
    uint8_t ipm_up[IPM_TAB_SIZE];

//#define MVF_STASH_WIDTH       128
#define MVF_STASH_WIDTH       64
#define MVF_STASH_HEIGHT      64
#define MVF_STASH_WIDTH_PU    (MVF_STASH_WIDTH >> LOG2_MIN_PU_SIZE)
#define MVF_STASH_HEIGHT_PU   (MVF_STASH_HEIGHT >> LOG2_MIN_PU_SIZE)
    HEVCMMALMvField mvf_ul[1];
    HEVCMMALMvField mvf_stash[MVF_STASH_WIDTH_PU * MVF_STASH_HEIGHT_PU];

    /* +7 is for subpixel interpolation, *2 for high bit depths */
//    DECLARE_ALIGNED(32, uint8_t, edge_emu_buffer)[(MAX_PB_SIZE + 7) * EDGE_EMU_BUFFER_STRIDE * 2];
    /* The extended size between the new edge emu buffer is abused by SAO */
//    DECLARE_ALIGNED(32, uint8_t, edge_emu_buffer2)[(MAX_PB_SIZE + 7) * EDGE_EMU_BUFFER_STRIDE * 2];
//    DECLARE_ALIGNED(32, int16_t, tmp [MAX_PB_SIZE * MAX_PB_SIZE]);

} HEVCMMALLocalContext;

// Each block can have an intra prediction and an add_residual command
// noof-cmds(2) * max-ctu height(64) / min-transform(4) * planes(3) * MAX_WIDTH

// Sand only has 2 planes (Y/C)
#define MMAL_MAX_PRED_CMDS (2*(HEVC_MAX_CTB_SIZE/4)*2*(HEVC_MMAL_MAX_WIDTH/4))

// Command for intra prediction and transform_add of predictions to coefficients
enum mmal_pred_cmd_e
{
    MMAL_PRED_ADD_RESIDUAL,
    MMAL_PRED_ADD_RESIDUAL_U, // = MMAL_PRED_TRANSFORM_ADD + c_idx
    MMAL_PRED_ADD_RESIDUAL_V, // = MMAL_PRED_TRANSFORM_ADD + c_idx
    MMAL_PRED_ADD_RESIDUAL_C, // Merged U+V
    MMAL_PRED_ADD_DC,
    MMAL_PRED_ADD_DC_U,       // Both U & V are effectively C
    MMAL_PRED_ADD_DC_V,
    MMAL_PRED_INTRA,
    MMAL_PRED_INTRA_C,
    MMAL_PRED_I_PCM,
    MMAL_PRED_CMD_MAX
};

typedef struct HEVCPredCmd {
    uint8_t type;
    uint8_t size;  // log2 "size" used by all variants
    uint8_t avail; // i_pred - but left here as they pack well
    uint8_t dummy;
    union {
        struct {  // TRANSFORM_ADD
            uint8_t * dst;
            const int16_t * buf;
            uint16_t stride;  // Should be good enough for all pic fmts we use
            int16_t dc;
        } ta;
        struct {
            uint8_t * dst;
            uint32_t stride;
            int dc;
        } dc;
        struct {  // INTRA
            uint16_t x;
            uint16_t y;
            enum IntraPredMode mode;
        } i_pred;
        struct {  // I_PCM
            uint16_t x;
            uint16_t y;
            const void * src;
            uint32_t src_len;
        } i_pcm;
    };
} HEVCPredCmd;

union qpu_mc_pred_cmd_s;
struct qpu_mc_pred_y_p_s;
struct qpu_mc_src_s;

typedef struct HEVCMMALInterPredQ
{
    union qpu_mc_pred_cmd_u *qpu_mc_base;
    union qpu_mc_pred_cmd_u *qpu_mc_curr;
    struct qpu_mc_src_s *last_l0;
    struct qpu_mc_src_s *last_l1;
    unsigned int load;
    uint32_t code_setup;
    uint32_t code_sync;
    uint32_t code_exit;
} HEVCMMALInterPredQ;

typedef struct HEVCMMALInterPredEnv
{
    HEVCMMALInterPredQ * q;
    uint8_t n;                  // Number of Qs
    uint8_t n_grp;              // Number of Q in a group
    uint8_t curr;               // Current Q number (0..n-1)
    uint8_t used;               // 0 if nothing in any Q, 1 otherwise
    uint8_t used_grp;           // 0 if nothing in any Q in the current group
    unsigned int max_fill;
    unsigned int min_gap;
    GPU_MEM_PTR_T gptr;
} HEVCMMALInterPredEnv;

typedef struct HEVCMMALIntraPredEnv {
    unsigned int n;        // Number of commands
    HEVCPredCmd * cmds;
} HEVCMMALIntraPredEnv;

typedef struct HEVCMMALCoeffEnv {
    unsigned int n;
#if MMAL_COMPRESS_COEFFS
    unsigned int packed; // Equal to 1 if coefficients should be being packed
    unsigned int packed_n; // Value of n when packed was set equal to 0 (i.e. the amount that is sent compressed).  Only valid if packed==0
#endif
    int16_t * buf;
} HEVCMMALCoeffEnv;

typedef struct HEVCMMALCoeffsEnv {
    HEVCMMALCoeffEnv s[4];
    GPU_MEM_PTR_T gptr;
    void * mptr;
} HEVCMMALCoeffsEnv;

typedef struct HEVCMMALFrameProgressWait {
    int req;
    struct HEVCMMALFrameProgressWait * next;
    sem_t sem;
} HEVCMMALFrameProgressWait;

typedef struct HEVCMMALFrameProgressState {
    struct HEVCMMALFrameProgressWait * first;
    struct HEVCMMALFrameProgressWait * last;
    pthread_mutex_t lock;
} HEVCMMALFrameProgressState;

typedef struct MMALBlk
{
    unsigned int x;
    unsigned int y;
    unsigned int w;
    unsigned int h;
} MMALBlk;

typedef struct HEVCMMALJob {
    struct HEVCMMALJob * next;  // Free chain
    struct HEVCMMALJobCtl * jbc_local;
    const HEVCMMALSPS * sps;       // sps used to set up this job

    int waited;
    int ctu_ts_first;
    int ctu_ts_last;
    MMALBlk bounds;  // Bounding box of job

    struct qpu_mc_pred_y_p_s * last_y8_p;
    struct qpu_mc_src_s * last_y8_l1;
    mmal_cache_flush_env_t * rfe;

    HEVCMMALInterPredEnv chroma_ip;
    HEVCMMALInterPredEnv luma_ip;
    int16_t progress_req[HEVC_DPB_ELS]; // index by dpb_no
    HEVCMMALIntraPredEnv intra;
    HEVCMMALCoeffsEnv coeffs;
    HEVCMMALFrameProgressWait progress_wait;
    sem_t sem;
    mmal_cache_buf_t flush_buf;
} HEVCMMALJob;

struct HEVCMMALContext;

typedef void HEVCMMALWorkerFn(const struct HEVCMMALContext * const s, HEVCMMALJob * const jb);

typedef struct HEVCMMALPassQueue
{
//    int pending;
    volatile int terminate;
    sem_t sem_in;
    sem_t * psem_out;
    unsigned int job_n;
    struct HEVCMMALContext * context; // Context pointer as we get to pass a single "void * this" to the thread
    HEVCMMALWorkerFn * worker;
    pthread_t thread;
    uint8_t pass_n;  // Pass number - debug
    uint8_t started;
} HEVCMMALPassQueue;


struct HEVCMMALJobGlobal;

typedef struct HEVCMMALJobCtl
{
    sem_t sem_out;

    HEVCMMALJob * volatile jb1;  // The job associated with this frame if unallocated - NULL if allocated
    struct HEVCMMALJobGlobal * jbg;

    HEVCMMALLocalContext * lcw_head;
    HEVCMMALLocalContext * lcw_tail;

    pthread_mutex_t in_lock;
    int offload_in;

    HEVCMMALJob *offloadq[MMAL_MAX_JOBS];
} HEVCMMALJobCtl;


typedef struct HEVCMMALJobGlobal
{
    intptr_t ref_count;
    pthread_mutex_t lock;
    HEVCMMALJob * free1;                 // Singly linked list of free jobs
    HEVCMMALLocalContext * wait_head;       // Double linked list of lcs waiting for a job
    HEVCMMALLocalContext * wait_good;  // Last good tail
    HEVCMMALLocalContext * wait_tail;

} HEVCMMALJobGlobal;

#define MMAL_BIT_THREADS (MMAL_EXTRA_BIT_THREADS + 1)

#if MMAL_TSTATS
typedef struct HEVCMMALStats {
    int y_pred1_y8_merge;
    int y_pred1_xy;
    int y_pred1_x0;
    int y_pred1_y0;
    int y_pred1_x0y0;
    int y_pred1_wle8;
    int y_pred1_wgt8;
    int y_pred1_hle16;
    int y_pred1_hgt16;
    int y_pred2_xy;
    int y_pred2_x0;
    int y_pred2_y0;
    int y_pred2_x0y0;
    int y_pred2_hle16;
    int y_pred2_hgt16;
} HEVCMMALStats;
#endif

typedef struct HEVCMMALCabacState
{
    uint8_t rice[4];
    uint8_t state[HEVC_CONTEXTS];
} HEVCMMALCabacState;

#define HEVC_MMAL_BS_STRIDE1_PEL_SHIFT   6   // 64 pels
#define HEVC_MMAL_BS_STRIDE1_PELS        (1U << HEVC_MMAL_BS_STRIDE1_PEL_SHIFT)
#define HEVC_MMAL_BS_STRIDE1_PEL_MASK    (HEVC_MMAL_BS_STRIDE1_PELS - 1)
#define HEVC_MMAL_BS_ELS_PER_BYTE_SHIFT  2   // 4 els per byte
#define HEVC_MMAL_BS_PELS_PER_EL_SHIFT   2   // 4 pels per el
#define HEVC_MMAL_BS_PELS_PER_BYTE_SHIFT (HEVC_MMAL_BS_PELS_PER_EL_SHIFT + HEVC_MMAL_BS_ELS_PER_BYTE_SHIFT)
#define HEVC_MMAL_BS_STRIDE1_BYTE_SHIFT  (HEVC_MMAL_BS_STRIDE1_PEL_SHIFT - HEVC_MMAL_BS_PELS_PER_BYTE_SHIFT)
#define HEVC_MMAL_BS_STRIDE1_BYTES       (1U << HEVC_MMAL_BS_STRIDE1_BYTE_SHIFT)
#define HEVC_MMAL_BS_Y_SHR               3   // 8 vertical pels per row
#define HEVC_MMAL_BS_COL_BYTES_SHR       (HEVC_MMAL_BS_Y_SHR - HEVC_MMAL_BS_STRIDE1_BYTE_SHIFT)

typedef struct HEVCMMALContext {
    const AVClass *c;  // needed by private avoptions
    AVCodecContext *avctx;

    uint8_t             threads_type;
    char qpu_init_ok;

    /** 1 if the independent slice segment header was successfully parsed */
    uint8_t slice_initialized;
    char used_for_ref;  // mmal
    char is_irap;
    char offload_recon;
    uint8_t eos;       ///< current packet contains an EOS/EOB NAL
    uint8_t last_eos;  ///< last packet contains an EOS/EOB NAL
    uint8_t no_backward_pred_flag;
    uint8_t is_decoded;
    uint8_t no_rasl_output_flag;


    /**
     * Sequence counters for decoded and output frames, so that old
     * frames are output first after a POC reset
     */
    uint16_t seq_decode;
    uint16_t seq_output;

    int                 width;
    int                 height;

    HEVCMMALJobCtl * jbc;
    // cabac stash
    // b0       skip flag
    // b1+      ct_depth
    uint8_t * cabac_stash_left;
    uint8_t * cabac_stash_up;

    // Function pointers
#if MMAL_QPU_EMU_Y || MMAL_QPU_EMU_C
    const uint8_t * qpu_dummy_frame_emu;
#endif
#if !MMAL_QPU_EMU_Y || !MMAL_QPU_EMU_C
    uint32_t qpu_dummy_frame_qpu;  // Not a frame - just a bit of memory
#endif
    HEVCMMALQpu qpu;

    HEVCMMALFrameProgressState progress_states[2];

    HEVCMMALCabacState *cabac_save;

    AVFrame *frame;
    AVFrame *output_frame;
    uint8_t *sao_pixel_buffer_h[3];
    uint8_t *sao_pixel_buffer_v[3];

    unsigned int col_mvf_stride;
    AVBufferPool *col_mvf_pool;

    MMALSAOParams *sao;
    DBParams *deblock;
    enum HEVCNALUnitType nal_unit_type;
    int temporal_id;  ///< temporal_id_plus1 - 1
    HEVCMMALFrame *ref;
    int poc;
    int pocTid0;
    int slice_idx; ///< number of the slice being currently decoded
    int max_ra;

    int8_t *qp_y_tab;

    // Deblocking block strength bitmaps
    unsigned int bs_stride2;
    unsigned int bs_size;
    uint8_t *bs_horizontal;
    uint8_t *bs_vertical;
    uint8_t *bsf_stash_up;
    uint8_t *bsf_stash_left;

#if HEVC_MMAL_MAX_CTBS >= 0xffff
#define TAB_SLICE_ADDR_BROKEN ~(uint32_t)0
    uint32_t *tab_slice_address;
#else
#define TAB_SLICE_ADDR_BROKEN ~(uint16_t)0
    uint16_t *tab_slice_address;
#endif

    // Bitfield 1 bit per 8 pels (min pcm size)
    uint8_t *is_pcm;
    // Bitfield 1 bit per 8 pels (min cb size)
    // Only needed for CIP as CIP processing is async to the main thread
    uint8_t *is_intra;

    // PU
    HEVCMMALMvField *mvf_up;
    HEVCMMALMvField *mvf_left;

    const RefPicList **rpl_up;
    const RefPicList **rpl_left;
    RefPicList * refPicList;

    // CTB-level flags affecting loop filter operation
    uint8_t *filter_slice_edges;

    /** used on BE to byteswap the lines for checksumming */
    uint8_t *checksum_buf;
    int      checksum_buf_size;

    const uint8_t *data;

    H2645Packet pkt;
    // type of the first VCL NAL of the current frame
    enum HEVCNALUnitType first_nal_type;

    uint8_t context_initialized;
    int is_nalff;           ///< this flag is != 0 if bitstream is encapsulated
                            ///< as a format defined in 14496-15
    int apply_defdispwin;

    int nal_length_size;    ///< Number of bytes used for nal length (1, 2 or 4)
    int nuh_layer_id;

    struct AVMD5 *md5_ctx;

    RefPicListTab * rpl_tab;
    unsigned int rpl_tab_size;

    uint8_t *is_intra_store;

    MMALSliceHeader sh;

    HEVCMMALParamSets ps;

    HEVCMMALLocalContext    *HEVClc;
    HEVCMMALLocalContext    *HEVClcList[MAX_NB_THREADS];

    HEVCMMALFrame DPB[HEVC_DPB_ELS];

    ///< candidate references for the current frame
    RefPicList rps[5];

    HEVCMMALPredContext hpc;
    HEVCDSPContext hevcdsp;

    HEVCSEIContext sei;

    // Put structures that allocate non-trivial storage at the end
    // These are mostly used indirectly so position in the structure doesn't matter
    HEVCMMALPassQueue passq[MMAL_PASSES];
#if MMAL_EXTRA_BIT_THREADS > 0
    int bt_started;
    // This simply contains thread descriptors - task setup is held elsewhere
    pthread_t bit_threads[MMAL_EXTRA_BIT_THREADS];
#endif
#if MMAL_TSTATS
    HEVCMMALStats tstats;
#endif
} HEVCMMALContext;

/**
 * Mark all frames in DPB as unused for reference.
 */
void ff_hevc_mmal_clear_refs(HEVCMMALContext *s);

/**
 * Drop all frames currently in DPB.
 */
void ff_hevc_mmal_flush_dpb(HEVCMMALContext *s);

/**
 * Construct the reference picture sets for the current frame.
 */
int ff_hevc_mmal_frame_rps(HEVCMMALContext *s);

/**
 * Construct the reference picture list(s) for the current slice.
 */
int ff_hevc_mmal_slice_rpl(HEVCMMALContext *s);


/**
 * Get the number of candidate references for the current frame.
 */
int ff_hevc_mmal_frame_nb_refs(HEVCMMALContext *s);

int ff_hevc_mmal_set_new_ref(HEVCMMALContext *s, AVFrame **frame, int poc);

/**
 * Find next frame in output order and put a reference to it in frame.
 * @return 1 if a frame was output, 0 otherwise
 */
int ff_hevc_mmal_output_frame(HEVCMMALContext *s, AVFrame *frame, int flush);

void ff_hevc_mmal_bump_frame(HEVCMMALContext *s);

void ff_hevc_mmal_unref_frame(HEVCMMALContext *s, HEVCMMALFrame *frame, int flags);

unsigned int ff_hevc_mmal_tb_avail_flags(
    const HEVCMMALContext * const s, const HEVCMMALLocalContext * const lc,
    const unsigned int x, const unsigned int y, const unsigned int w, const unsigned int h);

void ff_hevc_mmal_luma_mv_merge_mode(const HEVCMMALContext * const s, HEVCMMALLocalContext * const lc, int x0, int y0, int nPbW,
                                int nPbH, int log2_cb_size, int part_idx,
                                int merge_idx, HEVCMMALMvField * const mv);
void ff_hevc_mmal_luma_mv_mvp_mode(const HEVCMMALContext * const s, HEVCMMALLocalContext * const lc,
    const unsigned int x0, const unsigned int y0,
    const unsigned int nPbW, const unsigned int nPbH,
    const unsigned int avail,
    HEVCMMALMvField * const mv,
    const unsigned int mvp_lx_flag, const unsigned int LX);
void ff_hevc_mmal_set_qPy(const HEVCMMALContext * const s, HEVCMMALLocalContext * const lc, int xBase, int yBase);
void ff_hevc_mmal_deblocking_boundary_strengths(const HEVCMMALContext * const s, const HEVCMMALLocalContext * const lc,
                                               const unsigned int x0, const unsigned int y0,
                                               const unsigned int log2_trafo_size, const int is_coded_block);
int ff_hevc_mmal_hls_filter_blk(const HEVCMMALContext * const s, const MMALBlk bounds, const int eot);

extern const uint8_t ff_hevc_mmal_qpel_extra_before[4];
extern const uint8_t ff_hevc_mmal_qpel_extra_after[4];
extern const uint8_t ff_hevc_mmal_qpel_extra[4];

int16_t * mmal_alloc_coeff_buf(HEVCMMALJob * const jb, const int buf_no, const int n);

// arm/hevc_misc_neon.S
// Neon coeff zap fn
#if HAVE_NEON
extern void mmal_zap_coeff_vals_neon(int16_t * dst, unsigned int l2ts_m2);
#endif

void ff_hevc_mmal_progress_wait_field(const HEVCMMALContext * const s, HEVCMMALJob * const jb,
                                     const HEVCMMALFrame * const ref, const int val, const int field);

void ff_hevc_mmal_progress_signal_field(HEVCMMALContext * const s, const int val, const int field);

// All of these expect that s->threads_type == FF_THREAD_FRAME

static inline void ff_hevc_mmal_progress_wait_mv(const HEVCMMALContext * const s, HEVCMMALJob * const jb,
                                     const HEVCMMALFrame * const ref, const int y)
{
    if (s->threads_type != 0)
        ff_hevc_mmal_progress_wait_field(s, jb, ref, y, 1);
}

static inline void ff_hevc_mmal_progress_signal_mv(HEVCMMALContext * const s, const int y)
{
    if (s->used_for_ref && s->threads_type != 0)
        ff_hevc_mmal_progress_signal_field(s, y, 1);
}

static inline void ff_hevc_mmal_progress_wait_recon(const HEVCMMALContext * const s, HEVCMMALJob * const jb,
                                     const HEVCMMALFrame * const ref, const int y)
{
    ff_hevc_mmal_progress_wait_field(s, jb, ref, y, 0);
}

static inline void ff_hevc_mmal_progress_signal_recon(HEVCMMALContext * const s, const int y)
{
    if (s->used_for_ref && s->threads_type != 0)
    {
        ff_hevc_mmal_progress_signal_field(s, y, 0);
    }
}

static inline void ff_hevc_mmal_progress_signal_all_done(HEVCMMALContext * const s)
{
    ff_hevc_mmal_progress_signal_field(s, INT_MAX, 0);
    ff_hevc_mmal_progress_signal_field(s, INT_MAX, 1);
}


// Set all done - signal nothing (used in missing refs)
// Works for both mmal & non-mmal
static inline void ff_hevc_mmal_progress_set_all_done(HEVCMMALFrame * const ref)
{
    if (ref->tf.progress != NULL)
    {
        int * const p = (int *)ref->tf.progress->data;
        p[0] = INT_MAX;
        p[1] = INT_MAX;
    }
}

#define HEVC_MMAL_420_ONLY 1
#define HEVC_MMAL_SAND128_ONLY 1

static inline unsigned int ctx_hshift(const HEVCMMALContext * const s, const int cidx)
{
#if HEVC_MMAL_420_ONLY
    return cidx == 0 ? 0 : 1;
#else
    return s->ps.sps->hshift[cidx];
#endif
}

static inline unsigned int ctx_vshift(const HEVCMMALContext * const s, const int cidx)
{
#if HEVC_MMAL_420_ONLY
    return cidx == 0 ? 0 : 1;
#else
    return s->ps.sps->vshift[cidx];
#endif
}

static inline int ctx_cfmt(const HEVCMMALContext * const s)
{
#if HEVC_MMAL_420_ONLY
    return 1;
#else
    return s->ps.sps->chroma_format_idc;
#endif
}

static inline int frame_stride1(const AVFrame * const frame, const int c_idx)
{
#if HEVC_MMAL_SAND128_ONLY
    return 128;
#else
    return frame->linesize[c_idx];
#endif
}

#if HEVC_MMAL_SAND128_ONLY
// Propagate this decision to later zc includes
#define MMAL_ZC_SAND128_ONLY 1
#endif

#ifndef ff_hevc_mmal_copy_vert
static inline void ff_hevc_mmal_copy_vert(uint8_t *dst, const uint8_t *src,
                                         int pixel_shift, int height,
                                         ptrdiff_t stride_dst, ptrdiff_t stride_src)
{
    int i;
    switch (pixel_shift)
    {
        case 2:
            for (i = 0; i < height; i++) {
                *(uint32_t *)dst = *(uint32_t *)src;
                dst += stride_dst;
                src += stride_src;
            }
            break;
        case 1:
            for (i = 0; i < height; i++) {
                *(uint16_t *)dst = *(uint16_t *)src;
                dst += stride_dst;
                src += stride_src;
            }
            break;
        default:
            for (i = 0; i < height; i++) {
                *dst = *src;
                dst += stride_dst;
                src += stride_src;
            }
            break;
    }
}
#endif


#if MVF_STASH_WIDTH == 64
static inline HEVCMMALMvField* mvf_stash_ptr(const HEVCMMALContext *const s, const HEVCMMALLocalContext * const lc,
                               const unsigned int x, const unsigned int y)
{
    const unsigned int mask_cs_hi = (~0U << s->ps.sps->log2_ctb_size);
    return (HEVCMMALMvField*)(lc->mvf_stash + ((y & ~mask_cs_hi) >> LOG2_MIN_PU_SIZE) * MVF_STASH_WIDTH_PU + ((x & ~mask_cs_hi) >> LOG2_MIN_PU_SIZE));
}

static inline HEVCMMALMvField* mvf_ptr(const HEVCMMALContext *const s, const HEVCMMALLocalContext * const lc,
                               const unsigned int x0, const unsigned int y0,
                               const unsigned int x, const unsigned int y)
{
    const unsigned int mask_cs_hi = (~0U << s->ps.sps->log2_ctb_size);
    const unsigned int x0_ctb = x0 & mask_cs_hi;
    const unsigned int y0_ctb = y0 & mask_cs_hi;

    return (HEVCMMALMvField *)((y < y0_ctb) ?
        (x < x0_ctb ? lc->mvf_ul : s->mvf_up + (x >> LOG2_MIN_PU_SIZE)) :
        (x < x0_ctb ? s->mvf_left + (y >> LOG2_MIN_PU_SIZE) :
            lc->mvf_stash +
                ((y & ~mask_cs_hi) >> LOG2_MIN_PU_SIZE) * MVF_STASH_WIDTH_PU +
                ((x & ~mask_cs_hi) >> LOG2_MIN_PU_SIZE)));
}

static inline unsigned int mvf_left_stride(const HEVCMMALContext *const s,
                               const unsigned int x0,
                               const unsigned int x)
{
    const unsigned int mask_cs_hi = (~0U << s->ps.sps->log2_ctb_size);
    const unsigned int x0_ctb = x0 & mask_cs_hi;
    return x < x0_ctb ? 1 : MVF_STASH_WIDTH_PU;
}

#else
static inline HEVCMMALMvField* mvf_stash_ptr(const HEVCMMALContext *const s, const HEVCMMALLocalContext * const lc,
                               const unsigned int x, const unsigned int y)
{
    const unsigned int mask_cs_hi = (~0U << s->ps.sps->log2_ctb_size);
    return (HEVCMMALMvField*)(lc->mvf_stash + ((y & ~mask_cs_hi) >> LOG2_MIN_PU_SIZE) * MVF_STASH_WIDTH_PU + ((x >> LOG2_MIN_PU_SIZE) & (MVF_STASH_WIDTH_PU - 1)));
}

static inline HEVCMMALMvField* mvf_ptr(const HEVCMMALContext *const s, const HEVCMMALLocalContext * const lc,
                               const unsigned int x0, const unsigned int y0,
                               const unsigned int x, const unsigned int y)
{
    const unsigned int mask_cs_hi = (~0U << s->ps.sps->log2_ctb_size);

    const unsigned int x0_ctb = x0 & mask_cs_hi;
    const unsigned int y0_ctb = y0 & mask_cs_hi;

    // If not in the same CTB for Y assume up
    if (y < y0_ctb) {
        // If not in the same CTB for X too assume up-left
        return (HEVCMMALMvField *)(x < x0_ctb ? lc->mvf_ul : s->mvf_up + (x >> LOG2_MIN_PU_SIZE));
    }
    return mvf_stash_ptr(s, lc, x, y);
}

static inline unsigned int mvf_left_stride(const HEVCMMALContext *const s,
                               const unsigned int x0,
                               const unsigned int x)
{
    return MVF_STASH_WIDTH_PU;
}
#endif

#endif /* AVCODEC_MMAL_HEVCDEC_H */
