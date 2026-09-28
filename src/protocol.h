/*
 * protocol.h — hs611-min-ab WebUSB A/B protocol, version 6.
 *
 * The device enumerates as a vendor-class bulk device (256c:6111) with a bulk IN
 * for frames and a bulk OUT for commands. Frames are fixed-length; commands are
 * exactly 64 bytes, zero padded. Everything is little-endian.
 *
 * Frame, 168 bytes:
 *   off  size  field
 *   0    2     magic 'H','S'
 *   2    1     version (6)
 *   3    1     seq (wraps)
 *   4    2     flags (PROTO_FLAG_*)
 *   6    1     backend (PROTO_BACKEND_*)
 *   7    1     estimator (PROTO_ESTIMATOR_*)
 *   8    2     x position, 1/256 channel (1-based)
 *   10   2     y position, 1/256 channel (1-based)
 *   12   1     drive frequency index (1..12)
 *   13   1     burst periods (read burst, 6..32)
 *   14   1     adc samples per channel (1..7)
 *   15   1     adc clock select (0..3)
 *   16   1     x window coils (0 = full scan)
 *   17   1     y window coils (0 = full scan)
 *   18   1     ramp-mitigation mode (PROTO_RAMP_*)
 *   19   1     re-centre mode (PROTO_RECENTER_*)
 *   20   1     scan order (PROTO_SCAN_*)
 *   21   1     warm-up reads per axis
 *   22   1     x peak loop (1..41, 0 = none)
 *   23   1     y peak loop (1..27, 0 = none)
 *   24   4     device time, microseconds (wraps ~59.6 s)
 *   28   4     scan duration, microseconds
 *   32   82    x amplitude[41] (axis-B loops, u16 each)
 *   114  54    y amplitude[27] (axis-A loops, u16 each)
 *
 * Command (host -> device, exactly 64 bytes, zero padded); [0] is the opcode:
 *   0x01 PING            device replies with a frame
 *   0x02 SET_FREQ        [1]=1..12
 *   0x03 SET_FREQ_ARR    [1..2]=u16 LE raw TIMER1 ARR (carrier period in
 *                        72 MHz counts, clamped 100..400); 0 restores the
 *                        SET_FREQ index table. Hardware backend only;
 *                        SET_FREQ clears it (last one wins).
 *   0x04 SET_BACKEND     [1]=0 software / 1 hardware (the repeat-coil
 *                        diagnostic is selected with REPEAT_COIL)
 *   0x05 SET_BURST       [1]=6..32
 *   0x06 SET_SETTLE      [1..4]=A B C D microseconds
 *   0x07 SET_ADC         [1]=1..7 samples, [2]=ADCPSC 0..3 (/2,/4,/6,/8)
 *   0x08 SET_RECOVERY    [1]=0/1 site-D coil recovery
 *   0x09 SET_WINDOW      [1]=x coils 0..8, [2]=y coils 0..8 (0 = full scan)
 *   0x0a SET_RECENTER    [1]=mode, [2]=hysteresis % pitch, [3]=persist frames,
 *                        [4]=step coils, [5]=deadband whole coils;
 *                        0 keeps the current value for [2],[3],[5].
 *   0x0b SET_SCAN_ORDER  [1]=0 asc / 1 desc / 2 outside-in / 3 center-out
 *   0x0c SET_ESTIMATOR   [1]=0 ncentroid / 1 vendor / 2 gaomon / 3 log-gauss
 *   0x0d SET_LOGAUSS     [1]=axis 0=X/1=Y, [2..3]=u16 baseline,
 *                        [4..5]=a1 Q8 (signed), [6..7]=a3 Q8 (signed)
 *   0x0e SET_NCENTROID   [1..2]=u16 additive noise base (counts)
 *   0x0f SET_REACQ       [1..2]=u16 threshold, [3]=coarse stride 0/2..8,
 *                        [4..5]=u16 minimum re-acquire period ms (0 = every
 *                        frame)
 *   0x10 SET_RAMP        [1]=mode 0..7, [2]=prime burst periods, [3]=prime
 *                        repeats, [4..5]=slope i16 per-mille/coil,
 *                        [6]=targeted-reverse radius; 0 keeps current for
 *                        [2],[3],[6]
 *   0x11 SET_PACING      [1]=0 SOF / 1 continuous / 2 auto
 *   0x12 SET_WARMUP      [1]=0..8 discarded reads before each axis scan
 *   0x13 SET_FLAT_TOL    [1..2]=u16 flat-top hold tolerance (0 = off)
 *   0x14 REPEAT_COIL     [1]=0 off, 1..41 repeat that axis-B coil
 *
 * Pacing modes: 0 SOF (one acquisition per USB frame), 1 continuous (free-run),
 * 2 auto (free-run only while the carry-over ramp mode is set).
 */

#ifndef MIN_PROTOCOL_H
#define MIN_PROTOCOL_H

#include <stdint.h>

#define PROTO_MAGIC0        0x48U   /* 'H' */
#define PROTO_MAGIC1        0x53U   /* 'S' */
#define PROTO_VERSION       0x06U

#define PROTO_NX            41U
#define PROTO_NY            27U
#define PROTO_HEADER_LEN    32U
#define PROTO_FRAME_LEN     (PROTO_HEADER_LEN + PROTO_NX * 2U + PROTO_NY * 2U)   /* 168 */
#define PROTO_CMD_LEN       64U

/* Near threshold: a peak above it means the pen is coupling. */
#define PROTO_NEAR_THR      100U

/* Frame field offsets. */
#define PROTO_OFF_MAGIC0    0U
#define PROTO_OFF_MAGIC1    1U
#define PROTO_OFF_VERSION   2U
#define PROTO_OFF_SEQ       3U
#define PROTO_OFF_FLAGS     4U
#define PROTO_OFF_BACKEND   6U
#define PROTO_OFF_ESTIMATOR 7U
#define PROTO_OFF_XPOS      8U
#define PROTO_OFF_YPOS      10U
#define PROTO_OFF_FREQ      12U
#define PROTO_OFF_BURST     13U
#define PROTO_OFF_ADC_N     14U
#define PROTO_OFF_ADC_CLK   15U
#define PROTO_OFF_WIN_X     16U
#define PROTO_OFF_WIN_Y     17U
#define PROTO_OFF_RAMP      18U
#define PROTO_OFF_RECENTER  19U
#define PROTO_OFF_SCAN_ORDER 20U
#define PROTO_OFF_WARMUP    21U
#define PROTO_OFF_PEAK_X    22U
#define PROTO_OFF_PEAK_Y    23U
#define PROTO_OFF_TIME_US   24U
#define PROTO_OFF_SCAN_US   28U
#define PROTO_OFF_AMP_X     32U
#define PROTO_OFF_AMP_Y     (PROTO_OFF_AMP_X + PROTO_NX * 2U)

/* Frame flags. */
#define PROTO_FLAG_PEN        0x0001U   /* peak amplitude above PROTO_NEAR_THR */
#define PROTO_FLAG_WINDOWED   0x0002U   /* windowed scan active */
#define PROTO_FLAG_REACQUIRED 0x0004U   /* full/coarse re-acquire this frame */
#define PROTO_FLAG_SAT_RETRY  0x0008U   /* a saturated window was retried */
#define PROTO_FLAG_FOLLOW     0x0010U   /* follow-peak re-centre active */
#define PROTO_FLAG_STICKY     0x0020U   /* sticky re-centre active */
#define PROTO_FLAG_DESCENDING 0x0040U   /* descending scan order */
#define PROTO_FLAG_CARRYOVER  0x0080U   /* free-run (carry-over) acquisition */

/* Backend. */
#define PROTO_BACKEND_SW      0U
#define PROTO_BACKEND_HW      1U
#define PROTO_BACKEND_REPEAT  2U

/* Estimators (mirror estimator.h). */
#define PROTO_EST_NCENTROID   0U
#define PROTO_EST_VENDOR      1U
#define PROTO_EST_GAOMON      2U
#define PROTO_EST_LOGAUSS     3U

/* Re-centre modes. */
#define PROTO_RECENTER_EDGE     0U
#define PROTO_RECENTER_FOLLOW   1U
#define PROTO_RECENTER_STICKY   2U
#define PROTO_RECENTER_DEADBAND 3U

/* Scan orders. */
#define PROTO_SCAN_ASC        0U
#define PROTO_SCAN_DESC       1U
#define PROTO_SCAN_OUTIN      2U
#define PROTO_SCAN_INOUT      3U

/* Ramp-mitigation modes. */
#define PROTO_RAMP_NONE       0U
#define PROTO_RAMP_CARRYOVER  1U
#define PROTO_RAMP_PRIME_WIN  2U
#define PROTO_RAMP_READ_STEADY 3U
#define PROTO_RAMP_BIDI       4U
#define PROTO_RAMP_SLOPE      5U
#define PROTO_RAMP_TARGET_REV 6U
#define PROTO_RAMP_FRAME_AVG  7U

/* Pacing modes. */
#define PROTO_PACING_SOF        0U
#define PROTO_PACING_CONTINUOUS 1U
#define PROTO_PACING_AUTO       2U

/* Command opcodes. */
#define PROTO_CMD_PING           0x01U
#define PROTO_CMD_SET_FREQ       0x02U
#define PROTO_CMD_SET_FREQ_ARR   0x03U
#define PROTO_CMD_SET_BACKEND    0x04U
#define PROTO_CMD_SET_BURST      0x05U
#define PROTO_CMD_SET_SETTLE     0x06U
#define PROTO_CMD_SET_ADC        0x07U
#define PROTO_CMD_SET_RECOVERY   0x08U
#define PROTO_CMD_SET_WINDOW     0x09U
#define PROTO_CMD_SET_RECENTER   0x0AU
#define PROTO_CMD_SET_SCAN_ORDER 0x0BU
#define PROTO_CMD_SET_ESTIMATOR  0x0CU
#define PROTO_CMD_SET_LOGAUSS    0x0DU
#define PROTO_CMD_SET_NCENTROID  0x0EU
#define PROTO_CMD_SET_REACQ      0x0FU
#define PROTO_CMD_SET_RAMP       0x10U
#define PROTO_CMD_SET_PACING     0x11U
#define PROTO_CMD_SET_WARMUP     0x12U
#define PROTO_CMD_SET_FLAT_TOL   0x13U
#define PROTO_CMD_REPEAT_COIL    0x14U

#endif /* MIN_PROTOCOL_H */
