#ifndef CALIB_H
#define CALIB_H

// 直通(thru)校准: 两路接同一信号源时, 理论上 H=1∠0; 实测到的偏差
// (两路增益失配 + ~2.6ns 通道间采样偏斜) 记成 H_thru(f), 实测 DUT 时除掉.
void cal_run_thru(void);                                   // 跑一遍同源校准, 填校准表
void cal_run_thru_manual(const float* freqs, int count);
void cal_apply_correction(float f, float* mag, float* phase); // 有校准数据才修正
void cal_clear(void);                                      // 清空校准表
int  cal_is_valid(void);                                   // 是否已有有效校准
void cal_print_table(void);                                // 打印校准表(调试)

#endif // CALIB_H
