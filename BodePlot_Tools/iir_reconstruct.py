import os
import numpy as np
import matplotlib.pyplot as plt

from plot_bode import analyze_response


FS_DEFAULT = 1_000_000.0


def load_serial_csv(path):
    freqs, mags, phases = [], [], []
    parsing = False
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if line.startswith("f_actual,H_mag"):
                parsing = True
                continue
            if not parsing:
                continue
            parts = line.split(",")
            if len(parts) != 5:
                continue
            try:
                freqs.append(float(parts[0]))
                mags.append(float(parts[1]))
                phases.append(float(parts[2]))
            except ValueError:
                pass
    return np.array(freqs), np.array(mags), np.deg2rad(np.array(phases))


def fit_iir2_complex(freqs, mags, phases_rad, fs=FS_DEFAULT, max_fit_hz=None):
    if max_fit_hz is None:
        max_fit_hz = fs * 0.45

    order = np.argsort(freqs)
    f = freqs[order]
    h = mags[order] * np.exp(1j * phases_rad[order])
    mask = (f > 0.0) & (f < max_fit_hz) & np.isfinite(h)
    f = f[mask]
    h = h[mask]
    if len(f) < 8:
        raise ValueError("not enough points for second-order IIR fit")

    z1 = np.exp(-1j * 2.0 * np.pi * f / fs)
    z2 = z1 * z1

    # H * (1 + a1 z^-1 + a2 z^-2) = b0 + b1 z^-1 + b2 z^-2
    # Unknown x = [b0, b1, b2, a1, a2].
    a = np.column_stack([
        np.ones_like(z1),
        z1,
        z2,
        -h * z1,
        -h * z2,
    ])
    x, *_ = np.linalg.lstsq(a, h, rcond=None)
    coeff = np.real_if_close(x, tol=1000).real
    b = coeff[:3]
    a_den = np.array([1.0, coeff[3], coeff[4]])
    return b, a_den


def iir2_response(freqs, b, a, fs=FS_DEFAULT):
    z1 = np.exp(-1j * 2.0 * np.pi * freqs / fs)
    z2 = z1 * z1
    num = b[0] + b[1] * z1 + b[2] * z2
    den = a[0] + a[1] * z1 + a[2] * z2
    return num / den


def iir2_filter(x, b, a):
    y = np.zeros_like(x, dtype=float)
    x1 = x2 = 0.0
    y1 = y2 = 0.0
    for i, x0 in enumerate(x):
        y0 = b[0] * x0 + b[1] * x1 + b[2] * x2 - a[1] * y1 - a[2] * y2
        y[i] = y0
        x2, x1 = x1, x0
        y2, y1 = y1, y0
    return y


def vpp(x):
    return float(np.max(x) - np.min(x))


def print_coefficients(b, a):
    poles = np.roots(a)
    zeros = np.roots(b)
    stable = np.all(np.abs(poles) < 1.0)
    print("IIR2 coefficients for y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]")
    print(f"b0={b[0]: .9g}, b1={b[1]: .9g}, b2={b[2]: .9g}")
    print(f"a1={a[1]: .9g}, a2={a[2]: .9g}")
    print(f"zeros={zeros}")
    print(f"poles={poles}")
    print(f"stable={stable}")
    return stable


def run_reconstruction(serial_path, fs=FS_DEFAULT):
    freqs, mags, phases = load_serial_csv(serial_path)
    if len(freqs) == 0:
        raise RuntimeError(f"no CSV data found in {serial_path}")

    analysis = analyze_response(freqs, mags, np.rad2deg(phases))
    print(f"type={analysis['type']}")
    if analysis["center_freq"] is not None:
        print(f"measured f0={analysis['center_freq']:.2f} Hz")

    b, a = fit_iir2_complex(freqs, mags, phases, fs=fs)
    stable = print_coefficients(b, a)

    order = np.argsort(freqs)
    f = freqs[order]
    h_meas = mags[order] * np.exp(1j * phases[order])
    h_fit = iir2_response(f, b, a, fs=fs)
    mag_err_db = 20.0 * np.log10(np.clip(np.abs(h_fit), 1e-10, None)) - \
                 20.0 * np.log10(np.clip(np.abs(h_meas), 1e-10, None))
    phase_err_deg = np.rad2deg(np.unwrap(np.angle(h_fit)) - np.unwrap(np.angle(h_meas)))
    print(f"mag rms err={np.sqrt(np.mean(mag_err_db ** 2)):.3f} dB")
    print(f"mag max err={np.max(np.abs(mag_err_db)):.3f} dB")
    print(f"phase rms err={np.sqrt(np.mean(phase_err_deg ** 2)):.3f} deg")

    t = np.arange(int(fs * 0.004)) / fs
    test_cases = [
        ("10k sine", np.sin(2 * np.pi * 10_000.0 * t)),
        ("10k square", np.where(np.sin(2 * np.pi * 10_000.0 * t) >= 0.0, 1.0, -1.0)),
        ("20k square", np.where(np.sin(2 * np.pi * 20_000.0 * t) >= 0.0, 1.0, -1.0)),
    ]
    outputs = []
    for name, x in test_cases:
        y = iir2_filter(x, b, a)
        outputs.append((name, x, y))
        print(f"{name}: in_vpp={vpp(x):.4f}, out_vpp={vpp(y):.4f}, gain={vpp(y)/max(vpp(x), 1e-12):.4f}")

    fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    axes[0].plot(f, 20 * np.log10(np.clip(np.abs(h_meas), 1e-10, None)), ".", label="measured")
    axes[0].plot(f, 20 * np.log10(np.clip(np.abs(h_fit), 1e-10, None)), "-", label="IIR2 fit")
    axes[0].set_xscale("log")
    axes[0].set_ylabel("Magnitude (dB)")
    axes[0].grid(True, which="both", ls="--", alpha=0.5)
    axes[0].legend()

    axes[1].plot(f, np.rad2deg(np.unwrap(np.angle(h_meas))), ".", label="measured")
    axes[1].plot(f, np.rad2deg(np.unwrap(np.angle(h_fit))), "-", label="IIR2 fit")
    axes[1].set_xscale("log")
    axes[1].set_ylabel("Phase (deg)")
    axes[1].grid(True, which="both", ls="--", alpha=0.5)
    axes[1].legend()

    name, x, y = outputs[1]
    nshow = min(len(t), int(fs * 0.0005))
    axes[2].plot(t[:nshow] * 1000, x[:nshow], label=f"{name} input")
    axes[2].plot(t[:nshow] * 1000, y[:nshow], label="IIR output")
    axes[2].set_xlabel("Time (ms)")
    axes[2].set_ylabel("Amplitude")
    axes[2].grid(True, ls="--", alpha=0.5)
    axes[2].legend()

    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(serial_path), "IIR_Reconstruct_Result.png")
    plt.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"saved {out_path}")
    return stable


def synth_response(kind, freqs, f0=10_000.0, q=3.0, gain=1.0):
    w = 2.0 * np.pi * freqs
    w0 = 2.0 * np.pi * f0
    s = 1j * w
    den = s * s + (w0 / q) * s + w0 * w0
    if kind == "LP":
        h = gain * w0 * w0 / den
    elif kind == "HP":
        h = gain * s * s / den
    elif kind == "BP":
        h = gain * (w0 / q) * s / den
    elif kind == "BS":
        h = gain * (s * s + w0 * w0) / den
    else:
        h = np.ones_like(freqs, dtype=complex)
    return np.abs(h), np.rad2deg(np.angle(h))


def synthetic_classifier_selftest():
    freqs = np.logspace(2, 6, 300)
    for kind in ("LP", "HP", "BP", "BS"):
        q = 0.7 if kind in ("LP", "HP") else 3.0
        mags, phases = synth_response(kind, freqs, q=q)
        r = analyze_response(freqs, mags, phases)
        ok = (r["type_code"] == kind)
        print(f"synthetic {kind}: classified {r['type_code']} f0={r['center_freq']} ok={ok}")


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    synthetic_classifier_selftest()
    run_reconstruction(os.path.join(here, "serial_data.txt"))
