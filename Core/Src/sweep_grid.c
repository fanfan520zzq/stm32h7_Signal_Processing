#include "sweep_grid.h"
#include "sweep_engine.h"
#include <math.h>

// 对数粗扫 + 自适应细化 (简化框架)
void sweep_grid_execute(float start_f, float end_f) {
    // 粗扫：100Hz 到 500kHz
    // 分段密度：3-60kHz 点更密
    float f = start_f;
    float r_normal = powf(10.0f, 1.0f / 10.0f); // 10点/十倍频程
    float r_dense  = powf(10.0f, 1.0f / 25.0f); // 25点/十倍频程

    // 为了实现简单的有序递推与自适应，可以先做一轮粗扫
    // 真实的递归自适应需要基于前面测得的点做微分判断，这里给出主干流程
    while (f <= end_f) {
        sweep_measure_point(f);
        
        float r = r_normal;
        if (f >= 3000.0f && f <= 60000.0f) {
            r = r_dense;
        }
        f *= r;
    }
    
    // 自适应细化部分：(规范 §7.2)
    // 扫描 g_Htable，寻找斜率过大的区间，在区间中点插入新频点重测。
    // 这里留出递归细化的扩展桩位，具体判断逻辑可依据 H_mag 变化率。
}
