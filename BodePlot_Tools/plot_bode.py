import matplotlib.pyplot as plt
import numpy as np
import os

# 配置中文字体，防止乱码
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False

def mag_to_db(mag):
    if mag < 1e-10:
        mag = 1e-10
    return 20.0 * np.log10(mag)

def interp_log_freq(f1, y1, f2, y2, y_target):
    if f1 <= 0 or f2 <= 0 or y1 == y2:
        return f1
    lf1 = np.log10(f1)
    lf2 = np.log10(f2)
    lf = lf1 + (y_target - y1) * (lf2 - lf1) / (y2 - y1)
    return 10 ** lf

def find_crossings(freqs, mags_db, level_db):
    """H7-friendly crossing search: one pass, log-frequency interpolation."""
    out = []
    for i in range(len(freqs) - 1):
        d1 = mags_db[i] - level_db
        d2 = mags_db[i + 1] - level_db
        if d1 == 0:
            out.append(freqs[i])
        elif d1 * d2 < 0:
            out.append(interp_log_freq(freqs[i], mags_db[i],
                                       freqs[i + 1], mags_db[i + 1],
                                       level_db))
    return out

def find_band_edges(freqs, mags_db, center_idx, level_db):
    """Return the nearest left/right level crossings around a peak or notch."""
    left = None
    right = None

    for i in range(center_idx - 1, -1, -1):
        d1 = mags_db[i] - level_db
        d2 = mags_db[i + 1] - level_db
        if d1 == 0:
            left = freqs[i]
            break
        if d1 * d2 < 0:
            left = interp_log_freq(freqs[i], mags_db[i],
                                   freqs[i + 1], mags_db[i + 1],
                                   level_db)
            break

    for i in range(center_idx, len(freqs) - 1):
        d1 = mags_db[i] - level_db
        d2 = mags_db[i + 1] - level_db
        if d2 == 0:
            right = freqs[i + 1]
            break
        if d1 * d2 < 0:
            right = interp_log_freq(freqs[i], mags_db[i],
                                    freqs[i + 1], mags_db[i + 1],
                                    level_db)
            break

    out = []
    if left is not None:
        out.append(left)
    if right is not None:
        out.append(right)
    return out

def nearest_phase(freqs, phases, f):
    if len(freqs) == 0:
        return 0.0
    idx = int(np.searchsorted(freqs, f))
    if idx <= 0:
        return phases[0]
    if idx >= len(freqs):
        return phases[-1]
    f1, f2 = freqs[idx - 1], freqs[idx]
    p1, p2 = phases[idx - 1], phases[idx]
    if f1 <= 0 or f2 <= 0 or f1 == f2:
        return p1
    t = (np.log10(f) - np.log10(f1)) / (np.log10(f2) - np.log10(f1))
    return p1 + t * (p2 - p1)

def analyze_response(freqs, mags, phases):
    """
    Classifier written in a C/H7-portable style:
    - sort by frequency
    - find edge averages, max, min
    - decide LP/HP/BP/BS from blocked edges and middle dip
    - compute cutoff and center frequency with log-frequency interpolation
    """
    order = np.argsort(freqs)
    freqs = np.asarray(freqs)[order]
    mags = np.asarray(mags)[order]
    phases = np.asarray(phases)[order]
    mags_db = np.array([mag_to_db(m) for m in mags])

    n = len(freqs)
    edge_n = 3
    if n >= 30:
        edge_n = 5
    if n < edge_n:
        edge_n = n

    max_idx = int(np.argmax(mags_db))
    min_idx = int(np.argmin(mags_db))
    max_db = float(mags_db[max_idx])
    min_db = float(mags_db[min_idx])
    left_db = float(np.mean(mags_db[:edge_n]))
    right_db = float(np.mean(mags_db[-edge_n:]))

    left_peak_db = max_db
    right_peak_db = max_db
    if min_idx > 0:
        left_peak_db = float(np.max(mags_db[:min_idx]))
    if min_idx < n - 1:
        right_peak_db = float(np.max(mags_db[min_idx + 1:]))

    # 3 dB edge-block test. This maps directly to C:
    # low side blocked means left edge is below the best passband by 3 dB.
    low_blocked = left_db < max_db - 3.0
    high_blocked = right_db < max_db - 3.0

    # Band-stop needs a real interior dip, not just the first/last point.
    interior_min = (min_idx >= edge_n) and (min_idx < n - edge_n)
    edge_pass_db = min(left_db, right_db)
    notch_pass_db = min(left_peak_db, right_peak_db)
    notch_depth_db = notch_pass_db - min_db
    has_middle_notch = interior_min and (notch_depth_db > 6.0)

    result = {
        "freqs": freqs,
        "mags": mags,
        "phases": phases,
        "mags_db": mags_db,
        "type": "UNKNOWN 未知",
        "type_code": "UNKNOWN",
        "max_db": max_db,
        "min_db": min_db,
        "left_db": left_db,
        "right_db": right_db,
        "notch_depth_db": notch_depth_db,
        "center_freq": None,
        "center_phase": None,
        "geom_center_freq": None,
        "cutoff_level_db": None,
        "cutoff_freqs": [],
        "bandwidth": None,
        "q": None,
    }

    # Band-stop takes priority. Real measured data often rolls off at the far
    # high end too, so an edge-only LP/HP test would misclassify a notch.
    if has_middle_notch:
        result["type"] = "带阻滤波器 (BSF)"
        result["type_code"] = "BS"
        result["center_freq"] = float(freqs[min_idx])
        # For notch filters, -3 dB is measured down from the weaker local
        # passband shoulder around the dip, not from the far right edge.
        level = notch_pass_db - 3.0
        result["cutoff_level_db"] = level
        result["cutoff_freqs"] = find_band_edges(freqs, mags_db, min_idx, level)
    elif low_blocked and high_blocked:
        result["type"] = "带通滤波器 (BPF)"
        result["type_code"] = "BP"
        result["center_freq"] = float(freqs[max_idx])
        level = max_db - 3.0
        result["cutoff_level_db"] = level
        result["cutoff_freqs"] = find_band_edges(freqs, mags_db, max_idx, level)
    elif low_blocked:
        result["type"] = "高通滤波器 (HPF)"
        result["type_code"] = "HP"
        level = max_db - 3.0
        result["cutoff_level_db"] = level
        result["cutoff_freqs"] = find_crossings(freqs, mags_db, level)
    elif high_blocked:
        result["type"] = "低通滤波器 (LPF)"
        result["type_code"] = "LP"
        level = max_db - 3.0
        result["cutoff_level_db"] = level
        result["cutoff_freqs"] = find_crossings(freqs, mags_db, level)
    else:
        result["type"] = "直通 / 全通 (All-Pass)"
        result["type_code"] = "ALLPASS"
        level = max_db - 3.0
        result["cutoff_level_db"] = level
        result["cutoff_freqs"] = find_crossings(freqs, mags_db, level)

    if result["center_freq"] is not None:
        result["center_phase"] = nearest_phase(freqs, phases, result["center_freq"])

    # If a band has two cutoffs, geometric mean is a better center estimate.
    # Keep the measured peak/dip as the primary center because it is more robust
    # when the far passband rolls off or the two shoulders are asymmetric.
    if result["type_code"] in ("BP", "BS") and len(result["cutoff_freqs"]) >= 2:
        f1 = result["cutoff_freqs"][0]
        f2 = result["cutoff_freqs"][-1]
        if f1 > 0 and f2 > f1:
            result["geom_center_freq"] = float(np.sqrt(f1 * f2))
            result["bandwidth"] = float(f2 - f1)
            result["q"] = float(result["geom_center_freq"] / result["bandwidth"]) if result["bandwidth"] > 0 else None

    return result

def plot_bode_from_serial(filepath):
    if not os.path.exists(filepath):
        print(f"找不到数据文件: {filepath}")
        print("请把串口打印文本复制到 serial_data.txt 后再运行。")
        return

    # 解析文本文件，智能提取 CSV 数据部分
    freqs = []
    mags = []
    phases = []
    
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()
        
    start_parsing = False
    for line in lines:
        line = line.strip()
        # 定位到 CSV 的表头，说明接下来全是数据了
        if line.startswith("f_actual,H_mag"):
            start_parsing = True
            continue
            
        if start_parsing:
            if not line:
                continue # 跳过空行
            
            parts = line.split(',')
            # 如果这一行不是由 5 个数据组成，说明 CSV 区域结束了或者有杂乱打印
            if len(parts) != 5:
                continue 
                
            try:
                # 按照你的 printf 逻辑提取：f_actual(0), H_mag(1), H_phase(2)
                f_val = float(parts[0])
                mag_val = float(parts[1])
                phase_val = float(parts[2])
                
                freqs.append(f_val)
                mags.append(mag_val)
                phases.append(phase_val)
            except ValueError:
                pass # 忽略无法转换成数字的乱码行

    if not freqs:
        print("❌ 未能从文件中提取到任何有效数据！请确保你把 'f_actual,H_mag...' 开头的那段全粘进去了。")
        return
        
    analysis = analyze_response(np.array(freqs), np.array(mags), np.array(phases))
    freqs = analysis["freqs"]
    mags = analysis["mags"]
    phases = analysis["phases"]
    mags_db = analysis["mags_db"]
    filter_type = analysis["type"]
    cutoff_db = analysis["cutoff_level_db"]
    cutoff_freqs = analysis["cutoff_freqs"]

    print(f"成功提取到 {len(freqs)} 个频点的数据")
    print(f"滤波器类型: {filter_type}")
    print(f"最大增益: {analysis['max_db']:.2f} dB")
    print(f"左端均值: {analysis['left_db']:.2f} dB, 右端均值: {analysis['right_db']:.2f} dB, 最小值: {analysis['min_db']:.2f} dB")
    print(f"内部陷波深度估计: {analysis['notch_depth_db']:.2f} dB")
    if analysis["center_freq"] is not None:
        print(f"中心频率 f0(实测峰/谷): {analysis['center_freq']:.2f} Hz")
        print(f"中心相位: {analysis['center_phase']:.2f} deg")
    if analysis["geom_center_freq"] is not None:
        print(f"中心频率 f0(两侧-3dB几何均值): {analysis['geom_center_freq']:.2f} Hz")
    if analysis["bandwidth"] is not None:
        print(f"带宽 BW: {analysis['bandwidth']:.2f} Hz")
        print(f"Q 估计: {analysis['q']:.3f}")
    if cutoff_freqs:
        for idx, fc in enumerate(cutoff_freqs):
            print(f"-3dB 截止频率 #{idx+1}: {fc:.2f} Hz")
    else:
        print("未找到明显的 -3dB 截止频率")

    # 开始画图
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title('STM32H7 扫频仪 - 滤波器 Bode 图')

    # 上图：幅频特性 (Magnitude)
    ax1.plot(freqs, mags_db, color='#1f77b4', marker='.', linestyle='-', linewidth=1.5, markersize=8, label='Magnitude')
    
    # === 在图中绘制 -3dB 截止频率辅助线 ===
    ax1.axhline(y=cutoff_db, color='r', linestyle='--', alpha=0.7, label=f'-3dB线 ({cutoff_db:.1f}dB)')
    for fc in cutoff_freqs:
        ax1.axvline(x=fc, color='g', linestyle='--', alpha=0.7)
        # 在线上方标注具体频率数值
        ax1.text(fc, cutoff_db + 1.5, f" {fc:.1f} Hz", color='g', fontweight='bold', fontsize=11, rotation=0)
    if analysis["center_freq"] is not None:
        ax1.axvline(x=analysis["center_freq"], color='m', linestyle=':', alpha=0.8)
        ax1.text(analysis["center_freq"], analysis["max_db"], f" f0={analysis['center_freq']:.1f}Hz",
                 color='m', fontweight='bold', fontsize=11, rotation=0)
    
    ax1.legend(loc='upper right')
    
    ax1.set_ylabel('幅度 Magnitude (dB)', fontsize=12, fontweight='bold')
    ax1.set_title(f'滤波器频响实测结果 [{filter_type}]', fontsize=15, fontweight='bold', pad=15)
    ax1.grid(True, which="both", ls="--", alpha=0.6, color='gray')
    ax1.set_xscale('log') # 频率轴使用对数坐标

    # 下图：相频特性 (Phase)
    ax2.plot(freqs, phases, color='#ff7f0e', marker='.', linestyle='-', linewidth=1.5, markersize=8, label='Phase')
    
    # === 在相频图中也画出截止频率辅助线，并标注对应的相位 ===
    for fc in cutoff_freqs:
        ax2.axvline(x=fc, color='g', linestyle='--', alpha=0.7)
        # 用对数插值算出截止频率处的具体相位值
        # 寻找处于 fc 两侧的点
        idx_right = np.searchsorted(freqs, fc)
        if 0 < idx_right < len(freqs):
            idx_left = idx_right - 1
            f1, f2 = freqs[idx_left], freqs[idx_right]
            p1, p2 = phases[idx_left], phases[idx_right]
            log_f1, log_f2, log_fc = np.log10(f1), np.log10(f2), np.log10(fc)
            pc = p1 + (p2 - p1) * (log_fc - log_f1) / (log_f2 - log_f1)
            
            # 在图中标注相位
            ax2.text(fc, pc + 10, f" {pc:.1f}°", color='g', fontweight='bold', fontsize=11, rotation=0)
    if analysis["center_freq"] is not None:
        ax2.axvline(x=analysis["center_freq"], color='m', linestyle=':', alpha=0.8)

    ax2.set_xlabel('频率 Frequency (Hz)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('相位 Phase (Degrees)', fontsize=12, fontweight='bold')
    ax2.grid(True, which="both", ls="--", alpha=0.6, color='gray')
    ax2.set_xscale('log') # 频率轴使用对数坐标

    # 美化布局
    plt.tight_layout()
    
    # 保存高清图片并显示
    save_path = "Bode_Result.png"
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"图像已保存: {save_path}")
    
    plt.show()

if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    plot_bode_from_serial(os.path.join(here, "serial_data.txt"))
