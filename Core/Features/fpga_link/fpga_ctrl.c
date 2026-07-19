#include "fpga_ctrl.h"
#include "fpga_spi_protocol.h"
#include "spi_driver.h"
#include "module_state.h"
#include <stdio.h>

volatile uint32_t fpga_spi_err_count = 0;

// Wrapper that counts every failed SPI transaction
static int32_t fpga_wr(uint8_t addr, uint16_t data) {
    int32_t r = FPGA_Protocol_Write16(addr, data);
    if (r != ERR_OK) fpga_spi_err_count++;
    return r;
}

static int32_t fpga_rd(uint8_t addr, uint16_t *data) {
    int32_t r = FPGA_Protocol_Read16(addr, data);
    if (r != ERR_OK) fpga_spi_err_count++;
    return r;
}

static int32_t fpga_wr32(uint8_t addr, uint32_t data) {
    int32_t r = FPGA_Protocol_Write32(addr, data);
    if (r != ERR_OK) fpga_spi_err_count++;
    return r;
}

static int32_t fpga_rd32(uint8_t addr, uint32_t *data) {
    int32_t r = FPGA_Protocol_Read32(addr, data);
    if (r != ERR_OK) fpga_spi_err_count++;
    return r;
}

static int32_t fpga_rd64(uint8_t addr, uint64_t *data) {
    int32_t r = FPGA_Protocol_Read64(addr, data);
    if (r != ERR_OK) fpga_spi_err_count++;
    return r;
}

void FPGA_Ctrl_Init(void) {
    SPI_Driver_Init();

    int32_t wres = fpga_wr(FPGA_REG_CTRL_EN, 1);

    // Read back to prove the link and the override actually took effect
    uint16_t id = 0, ctrl = 0;
    int32_t r_id  = fpga_rd(FPGA_REG_ID, &id);
    int32_t r_ctl = fpga_rd(FPGA_REG_CTRL_EN, &ctrl);

    printf("LOG:INFO FPGA Init: ID=0x%04X(res=%ld) CTRL_EN=%u(res=%ld) wr_res=%ld err=%lu\r\n",
           id, (long)r_id, ctrl, (long)r_ctl, (long)wres,
           (unsigned long)fpga_spi_err_count);
}

int32_t FPGA_Ctrl_SetFreq(uint8_t ch, uint32_t freq_hz) {
    int32_t errs = 0;
    if (ch == 1) {
        errs += (fpga_wr(FPGA_REG_CH1_FREQ_L, freq_hz & 0xFFFF) != ERR_OK);
        errs += (fpga_wr(FPGA_REG_CH1_FREQ_H, (freq_hz >> 16) & 0xFFFF) != ERR_OK);
    } else {
        errs += (fpga_wr(FPGA_REG_CH2_FREQ_L, freq_hz & 0xFFFF) != ERR_OK);
        errs += (fpga_wr(FPGA_REG_CH2_FREQ_H, (freq_hz >> 16) & 0xFFFF) != ERR_OK);
    }
    return -errs;
}

int32_t FPGA_Ctrl_SetWave(uint8_t ch, uint8_t wave_type, uint8_t vpp) {
    uint16_t val = ((uint16_t)vpp << 8) | (wave_type & 0x03);
    int32_t r = (ch == 1) ? fpga_wr(FPGA_REG_CH1_WAVE, val)
                          : fpga_wr(FPGA_REG_CH2_WAVE, val);
    return (r == ERR_OK) ? 0 : -1;
}

void FPGA_Ctrl_Commit(void) {
    fpga_wr(FPGA_REG_COMMIT, 1);
}

void FPGA_Ctrl_Sync(void) {
    fpga_wr(FPGA_REG_SYNC, 1);
}

void FPGA_Ctrl_UpdateFromAlgorithm(uint8_t ch, uint8_t wave_type, uint32_t freq_hz) {
    // Default Vpp = 15 (1.5V, safely above the 1Vpp requirement, no DAC wrap)
    FPGA_Ctrl_SetWave(ch, wave_type, 15);
    FPGA_Ctrl_SetFreq(ch, freq_hz);
    FPGA_Ctrl_Commit();
}

int32_t FPGA_Ctrl_SetBothWave(FPGA_OutputWave_t wave_type) {
    uint16_t ch1 = 0U, ch2 = 0U, readback1 = 0U, readback2 = 0U;
    if (wave_type != FPGA_OUTPUT_WAVE_SINE && wave_type != FPGA_OUTPUT_WAVE_TRIANGLE) {
        return ERR_PARAM;
    }
    if (fpga_rd(FPGA_REG_CH1_WAVE, &ch1) != ERR_OK ||
        fpga_rd(FPGA_REG_CH2_WAVE, &ch2) != ERR_OK ||
        fpga_wr(FPGA_REG_CH1_WAVE, (uint16_t)((ch1 & 0xFF00U) | (uint16_t)wave_type)) != ERR_OK ||
        fpga_wr(FPGA_REG_CH2_WAVE, (uint16_t)((ch2 & 0xFF00U) | (uint16_t)wave_type)) != ERR_OK ||
        fpga_wr(FPGA_REG_COMMIT, 1U) != ERR_OK ||
        fpga_rd(FPGA_REG_CH1_WAVE, &readback1) != ERR_OK ||
        fpga_rd(FPGA_REG_CH2_WAVE, &readback2) != ERR_OK) {
        return ERR_HARDWARE;
    }
    if ((readback1 & 0x0003U) != (uint16_t)wave_type ||
        (readback2 & 0x0003U) != (uint16_t)wave_type) {
        return ERR_NOT_READY;
    }
    return ERR_OK;
}

int32_t FPGA_Ctrl_ApplyResult(const SignalSeparationResult *res) {
    if (!res || res->valid_count <= 0) return -1;

    int32_t errs = 0;
    uint32_t applied_freq[2] = {0, 0};

    // CTRL_EN 每次重新断言：FPGA 掉电/未配置会丢配置，单次 boot 写入不可靠
    errs += (fpga_wr(FPGA_REG_CTRL_EN, 1) != ERR_OK);

    // Separate_Signals 按 20k->100k 升序扫描：sig1 必为低频(A)->CH1，sig2 高频(B)->CH2
    for (int i = 0; i < res->valid_count && i < 2; i++) {
        const SignalInfo *s = (i == 0) ? &res->sig1 : &res->sig2;
        uint8_t ch = (uint8_t)(i + 1);

        if (s->type == WAVE_UNKNOWN || s->freq == 0) continue;

        uint8_t wave = (s->type == SIG_TRIANGLE) ? FPGA_WAVE_TRIANGLE : FPGA_WAVE_SINE;

        // vpp_10x = Vpp * 10，钳到 [10, 115]：保证 >=1Vpp 且 amp_adj 不溢出
        uint32_t vpp10 = (uint32_t)(s->amp * 10.0f + 0.5f);
        if (vpp10 < 10)  vpp10 = 10;
        if (vpp10 > 115) vpp10 = 115;

        errs += (FPGA_Ctrl_SetWave(ch, wave, (uint8_t)vpp10) != 0);
        errs += (FPGA_Ctrl_SetFreq(ch, s->freq) != 0);
        applied_freq[i] = s->freq;
    }

    // 两路 shadow 写完，一次 COMMIT 同边界生效
    FPGA_Ctrl_Commit();

    // 读回验证：shadow 值 + 配置序号
    uint16_t seq = 0, f1l = 0, f1h = 0, f2l = 0, f2h = 0;
    errs += (fpga_rd(FPGA_REG_SEQ, &seq) != ERR_OK);
    errs += (fpga_rd(FPGA_REG_CH1_FREQ_L, &f1l) != ERR_OK);
    errs += (fpga_rd(FPGA_REG_CH1_FREQ_H, &f1h) != ERR_OK);
    errs += (fpga_rd(FPGA_REG_CH2_FREQ_L, &f2l) != ERR_OK);
    errs += (fpga_rd(FPGA_REG_CH2_FREQ_H, &f2h) != ERR_OK);

    uint32_t rb1 = ((uint32_t)f1h << 16) | f1l;
    uint32_t rb2 = ((uint32_t)f2h << 16) | f2l;

    printf("LOG:INFO FPGA apply: ch1=%luHz ch2=%luHz rb_ch1=%luHz rb_ch2=%luHz seq=%u errs=%ld crcum=%lu\r\n",
           (unsigned long)applied_freq[0], (unsigned long)applied_freq[1],
           (unsigned long)rb1, (unsigned long)rb2,
           seq, (long)errs, (unsigned long)fpga_spi_err_count);

    return errs;
}

void FPGA_Ctrl_PrintStatus(void) {
    uint16_t id = 0, ctrl = 0, seq = 0;
    int32_t r1 = fpga_rd(FPGA_REG_ID, &id);
    int32_t r2 = fpga_rd(FPGA_REG_CTRL_EN, &ctrl);
    int32_t r3 = fpga_rd(FPGA_REG_SEQ, &seq);
    printf("ACK:FPGA_STATUS id=0x%04X(%ld) ctrl_en=%u(%ld) seq=%u(%ld) err=%lu\r\n",
           id, (long)r1, ctrl, (long)r2, seq, (long)r3,
           (unsigned long)fpga_spi_err_count);
}

int32_t FPGA_Ctrl_GetInfo(FPGA_Info_t *info) {
    int32_t result = ERR_OK;
    if (info == NULL) return ERR_PARAM;
    result |= fpga_rd(FPGA_REG_ID, &info->id);
    result |= fpga_rd(FPGA_REG_PROTOCOL_VERSION, &info->protocol_version);
    result |= fpga_rd(FPGA_REG_CAPABILITIES, &info->capabilities);
    result |= fpga_rd(FPGA_REG_STATUS, &info->status);
    result |= fpga_rd(FPGA_REG_ERROR_COUNT, &info->protocol_error_count);
    result |= fpga_rd(FPGA_REG_BUILD_ID, &info->build_id);
    if (result != ERR_OK) return ERR_HARDWARE;
    if (info->id != 0x2023U || info->protocol_version != 0x0002U ||
        (info->capabilities & FPGA_CAP_STAGE07_REQUIRED) != FPGA_CAP_STAGE07_REQUIRED) {
        return ERR_NOT_READY;
    }
    return ERR_OK;
}

int32_t FPGA_Ctrl_AcquireSnapshot(FPGA_Snapshot_t *snapshot) {
    uint16_t sequence_before = 0U;
    SPI_AnchorTiming_t anchor;
    if (snapshot == NULL) return ERR_PARAM;
    if (fpga_rd(FPGA_REG_SNAP_SEQ, &sequence_before) != ERR_OK) return ERR_HARDWARE;
    if (fpga_wr(FPGA_REG_SNAP_ARM, 1U) != ERR_OK) return ERR_HARDWARE;
    if (SPI_Driver_AnchorPulse(480U, &anchor) != ERR_OK) {
        fpga_spi_err_count++;
        return ERR_HARDWARE;
    }
    snapshot->local_anchor_cycles = anchor.envelope_start +
        ((anchor.cs_low_observed - anchor.envelope_start) / 2U);
    snapshot->local_anchor_uncertainty_cycles =
        anchor.cs_low_observed - anchor.envelope_start;
    if (fpga_rd(FPGA_REG_STATUS, &snapshot->status) != ERR_OK ||
        fpga_rd(FPGA_REG_SNAP_SEQ, &snapshot->sequence) != ERR_OK ||
        fpga_rd(FPGA_REG_ERROR_COUNT, &snapshot->protocol_error_count) != ERR_OK ||
        fpga_rd64(FPGA_REG_SNAP_SAMPLE_COUNTER, &snapshot->sample_counter) != ERR_OK ||
        fpga_rd64(FPGA_REG_SNAP_APPLY_COUNTER, &snapshot->apply_counter) != ERR_OK ||
        fpga_rd32(FPGA_REG_SNAP_PHASE_A, &snapshot->phase_a) != ERR_OK ||
        fpga_rd32(FPGA_REG_SNAP_PHASE_B, &snapshot->phase_b) != ERR_OK ||
        fpga_rd32(FPGA_REG_SNAP_ACTIVE_FTW_A, &snapshot->active_ftw_a) != ERR_OK ||
        fpga_rd32(FPGA_REG_SNAP_ACTIVE_FTW_B, &snapshot->active_ftw_b) != ERR_OK) {
        return ERR_HARDWARE;
    }
    if ((snapshot->status & FPGA_STATUS_SNAP_VALID) == 0U ||
        snapshot->sequence != (uint16_t)(sequence_before + 1U)) {
        return ERR_NOT_READY;
    }
    return ERR_OK;
}

int32_t FPGA_Ctrl_CommitDDSConfig(const FPGA_DDSConfig_t *config,
                                  FPGA_CommitReceipt_t *receipt) {
    uint16_t sequence_before = 0U, sequence_after = 0U;
    uint16_t mode;
    uint32_t readback_a = 0U, readback_b = 0U, readback_offset = 0U;
    FPGA_Snapshot_t snapshot;
    if (config == NULL || receipt == NULL) return ERR_PARAM;
    mode = (uint16_t)config->ratio_n;
    if (config->derived_b_enable) mode |= FPGA_PHASE_MODE_DERIVED_B;
    if (config->raw_ftw_enable) mode |= FPGA_PHASE_MODE_RAW_FTW;

    if (fpga_rd(FPGA_REG_SEQ, &sequence_before) != ERR_OK ||
        fpga_wr32(FPGA_REG_SHADOW_FTW_A, config->ftw_a) != ERR_OK ||
        fpga_wr32(FPGA_REG_SHADOW_FTW_B, config->ftw_b) != ERR_OK ||
        fpga_wr32(FPGA_REG_PHASE_OFFSET_B, config->phase_offset_b) != ERR_OK ||
        fpga_wr(FPGA_REG_PHASE_MODE, mode) != ERR_OK ||
        fpga_rd32(FPGA_REG_SHADOW_FTW_A, &readback_a) != ERR_OK ||
        fpga_rd32(FPGA_REG_SHADOW_FTW_B, &readback_b) != ERR_OK ||
        fpga_rd32(FPGA_REG_PHASE_OFFSET_B, &readback_offset) != ERR_OK) {
        return ERR_HARDWARE;
    }
    if (readback_a != config->ftw_a || readback_b != config->ftw_b ||
        readback_offset != config->phase_offset_b) return ERR_NOT_READY;
    if (fpga_wr(FPGA_REG_COMMIT, 1U) != ERR_OK ||
        fpga_rd(FPGA_REG_SEQ, &sequence_after) != ERR_OK) return ERR_HARDWARE;
    if (sequence_after != (uint16_t)(sequence_before + 1U)) return ERR_NOT_READY;
    if (FPGA_Ctrl_AcquireSnapshot(&snapshot) != ERR_OK) return ERR_HARDWARE;
    receipt->config_sequence = sequence_after;
    receipt->apply_counter = snapshot.apply_counter;
    receipt->active_ftw_a = snapshot.active_ftw_a;
    receipt->active_ftw_b = snapshot.active_ftw_b;
    if (config->raw_ftw_enable &&
        (receipt->active_ftw_a != config->ftw_a ||
         (!config->derived_b_enable && receipt->active_ftw_b != config->ftw_b))) {
        return ERR_NOT_READY;
    }
    return ERR_OK;
}

void FPGA_Ctrl_PrintInfo(void) {
    FPGA_Info_t info = {0};
    int32_t result = FPGA_Ctrl_GetInfo(&info);
    printf("ACK:FPGA_INFO id=0x%04X protocol=0x%04X capabilities=0x%04X status=0x%04X build=0x%04X protocol_errors=%u result=%ld\r\n",
           info.id, info.protocol_version, info.capabilities, info.status, info.build_id,
           info.protocol_error_count, (long)result);
}

void FPGA_Ctrl_PrintSnapshot(void) {
    FPGA_Snapshot_t snapshot = {0};
    int32_t result = FPGA_Ctrl_AcquireSnapshot(&snapshot);
    printf("ACK:FPGA_SNAPSHOT seq=%u anchor=%lu uncertainty=%lu sample=0x%08lX%08lX apply=0x%08lX%08lX phase_a=0x%08lX phase_b=0x%08lX ftw_a=0x%08lX ftw_b=0x%08lX status=0x%04X result=%ld\r\n",
           snapshot.sequence,
           (unsigned long)snapshot.local_anchor_cycles,
           (unsigned long)snapshot.local_anchor_uncertainty_cycles,
           (unsigned long)(snapshot.sample_counter >> 32U),
           (unsigned long)snapshot.sample_counter,
           (unsigned long)(snapshot.apply_counter >> 32U),
           (unsigned long)snapshot.apply_counter,
           (unsigned long)snapshot.phase_a,
           (unsigned long)snapshot.phase_b, (unsigned long)snapshot.active_ftw_a,
           (unsigned long)snapshot.active_ftw_b, snapshot.status, (long)result);
}

void FPGA_Ctrl_RunProtocolSelfTest(uint32_t count) {
    FPGA_Info_t info = {0};
    FPGA_Snapshot_t previous = {0}, current = {0};
    uint32_t snapshot_errors = 0U, sequence_errors = 0U;
    uint32_t counter_errors = 0U, phase_errors = 0U, status_errors = 0U;
    uint16_t seq_before_status = 0U, seq_after_status = 0U, ignored_status = 0U;
    uint16_t saved_mode = 0U;
    uint32_t saved_ftw_a = 0U, saved_ftw_b = 0U, saved_offset = 0U;
    uint32_t raw_errors = 0U;

    if (count == 0U) count = 1U;
    if (count > 1000U) count = 1000U;
    if (FPGA_Ctrl_GetInfo(&info) != ERR_OK ||
        FPGA_Ctrl_AcquireSnapshot(&previous) != ERR_OK) snapshot_errors++;

    if (fpga_rd(FPGA_REG_SNAP_SEQ, &seq_before_status) != ERR_OK ||
        fpga_rd(FPGA_REG_STATUS, &ignored_status) != ERR_OK ||
        fpga_rd(FPGA_REG_SNAP_SEQ, &seq_after_status) != ERR_OK ||
        seq_before_status != seq_after_status) status_errors++;

    for (uint32_t i = 0U; i < count && snapshot_errors == 0U; ++i) {
        if (FPGA_Ctrl_AcquireSnapshot(&current) != ERR_OK) {
            snapshot_errors++;
            break;
        }
        if (current.sequence != (uint16_t)(previous.sequence + 1U)) sequence_errors++;
        if (current.sample_counter <= previous.sample_counter) counter_errors++;
        uint64_t delta_counter = current.sample_counter - previous.sample_counter;
        uint32_t expected_delta = (uint32_t)delta_counter * previous.active_ftw_a;
        if ((uint32_t)(current.phase_a - previous.phase_a) != expected_delta) phase_errors++;
        previous = current;
    }

    if (fpga_rd(FPGA_REG_PHASE_MODE, &saved_mode) != ERR_OK ||
        fpga_rd32(FPGA_REG_SHADOW_FTW_A, &saved_ftw_a) != ERR_OK ||
        fpga_rd32(FPGA_REG_SHADOW_FTW_B, &saved_ftw_b) != ERR_OK ||
        fpga_rd32(FPGA_REG_PHASE_OFFSET_B, &saved_offset) != ERR_OK) raw_errors++;
    if (raw_errors == 0U) {
        FPGA_DDSConfig_t raw_config = {
            previous.active_ftw_a, previous.active_ftw_b, 0U, 1U, 0U, 1U
        };
        FPGA_CommitReceipt_t raw_receipt;
        if (FPGA_Ctrl_CommitDDSConfig(&raw_config, &raw_receipt) != ERR_OK) raw_errors++;

        FPGA_DDSConfig_t restore_config = {
            saved_ftw_a, saved_ftw_b, saved_offset,
            (uint8_t)(saved_mode & FPGA_PHASE_MODE_RATIO_MASK),
            (saved_mode & FPGA_PHASE_MODE_DERIVED_B) != 0U,
            (saved_mode & FPGA_PHASE_MODE_RAW_FTW) != 0U
        };
        FPGA_CommitReceipt_t restore_receipt;
        if (FPGA_Ctrl_CommitDDSConfig(&restore_config, &restore_receipt) != ERR_OK) raw_errors++;
    }

    uint8_t pass = (snapshot_errors == 0U && sequence_errors == 0U &&
                    counter_errors == 0U && phase_errors == 0U && status_errors == 0U &&
                    raw_errors == 0U) ? 1U : 0U;
    printf("ACK:FPGA_PROTOCOL_SELF_TEST count=%lu snapshot_errors=%lu seq_errors=%lu counter_errors=%lu phase_errors=%lu status_errors=%lu raw_errors=%lu pass=%u\r\n",
           (unsigned long)count, (unsigned long)snapshot_errors,
           (unsigned long)sequence_errors, (unsigned long)counter_errors,
           (unsigned long)phase_errors, (unsigned long)status_errors,
           (unsigned long)raw_errors, pass);
}
