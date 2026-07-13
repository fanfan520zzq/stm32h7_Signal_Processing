#ifndef RECON_CONFIG_H
#define RECON_CONFIG_H

/* ==================== Reconstruction user parameters ==================== */

/* 1: analyze ADC2/PB1 DUT output directly and bypass H(f).
 * 0: analyze ADC1/PC4 input and apply the learned H(f) table. */
#define RECON_USE_ADC2_DEBUG            1u
/* 1: USART1 is used by the PC test script; 0: keep LCD protocol quiet. */
#define RECON_PC_DEBUG                  1u

/* Sampling and low-frequency path. */
#define RECON_ADC_FAST_FS_HZ            1200000.0f
#define RECON_ADC_LOW_FS_HZ             200000.0f
#define RECON_LOW_FREQ_SWITCH_HZ        3000.0f
#define RECON_USE_MEASURED_ADC_FS       0u

/* Rebuild/relock timing. */
#define RECON_AUTO_REBUILD_CONFIRM      6u
#define RECON_AUTO_REBUILD_ENABLE       1u
#define RECON_RELOCK_BAD_COUNT          12u
#define RECON_RELOCK_ENABLE             0u
#define RECON_INVALID_RETRY_MS          10u
#define RECON_REBUILD_SETTLE_MS         10u
#define RECON_FIRST_LOCK_WARMUP         8u
#define RECON_FIRST_LOCK_GAP_MS         5u
#define RECON_FIRST_LOCK_VOTE_COUNT     5u

/* Normal steady-state tracking gains. */
#define RECON_PLL_TRACK_KP              0.08
#define RECON_PLL_TRACK_KI              0.01
#define RECON_PLL_PHASE_ALPHA           0.20

/* First lock and relock gains. */
#define RECON_PLL_ACQUIRE_KP            0.08
#define RECON_PLL_ACQUIRE_KI            0.01

/* Frequency correction limits for the phase PI controller, in Hz. */
#define RECON_PLL_FREQ_CORR_MAX_HZ      0.50
#define RECON_PLL_INTEGRAL_MAX_HZ       0.20
#define RECON_PLL_LOCK_ERR_RAD          1.00
#define RECON_PLL_UNLOCK_ERR_RAD        2.50
#define RECON_PLL_LOCK_CONFIRM_COUNT    4u
#define RECON_PLL_UNLOCK_CONFIRM_COUNT  8u
#define RECON_PLL_HOLD_ENABLE           0u
#define RECON_PLL_PHASE_CORRECTION_ENABLE 1u
#define RECON_PLL_PHASE_SLOPE_ALPHA     0.20
#define RECON_PLL_SLOPE_CORR_MAX_HZ     0.50
#define RECON_PLL_PHASE_KP_HZ_PER_RAD   0.005
#define RECON_PLL_MAX_PHASE_STEP_RAD    0.80

/* DEBUG_STAGE=14 only. */
#define RECON_PLL_STAGE14_INIT_KP       0.53
#define RECON_PLL_STAGE14_INIT_KI       0.05
#define RECON_PLL_STAGE14_RELOCK_KP     0.50
#define RECON_PLL_STAGE14_RELOCK_KI     0.02
#define RECON_PLL_STAGE14_BAD_COUNT     50u
#define RECON_PLL_STAGE14_LOOP_DELAY_MS 50u

#endif /* RECON_CONFIG_H */
