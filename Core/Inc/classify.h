#ifndef CLASSIFY_H
#define CLASSIFY_H

// 片上滤波类型判别(发挥1: 显示 LP/HP/BP/BS). 逻辑移植自 plot_bode.py.
typedef enum {
    FILT_UNKNOWN = 0,
    FILT_LP,        // 低通
    FILT_HP,        // 高通
    FILT_BP,        // 带通
    FILT_BS,        // 带阻
    FILT_ALLPASS    // 全通/直通
} FilterType;

typedef struct {
    FilterType type;
    float max_db;
    float min_db;
    float left_db;
    float right_db;
    float notch_depth_db;
    float center_freq;       // 实测峰/谷频率
    float center_phase_deg;
    float geom_center_freq;  // 两侧 -3dB 几何均值(带通/带阻)
    float cutoff_level_db;
    float cutoff_freqs[4];
    int   cutoff_count;
    float bandwidth;
    float q;
} FilterAnalysis;

// 用全局 g_Htable 判类型(扫频跑完后调用)
FilterType sweep_classify(void);
const char* filter_type_name(FilterType t);
int sweep_analyze(FilterAnalysis *out);
void print_filter_analysis(const FilterAnalysis *a);

// 找相对通带 -3dB 的截止频率(对数插值), 写入 out[], 返回个数(<=max_out)
int find_cutoffs_3db(float *out, int max_out);

#endif // CLASSIFY_H
