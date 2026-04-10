"""
Compare RQNN filter against the classical filters used in this project.

Mirrors the 9 filters from stationary_sensor_analysis.ipynb:
  1. Low-Pass (Butterworth, 5 Hz)
  2. Band-Pass (Butterworth, 0.5–10 Hz)
  3. Moving Average (window=21)
  4. Savitzky-Golay (window=21, order=3)
  5. Median (kernel=21)
  6. RANSAC
  7. Huber Robust
  8. Cauchy Robust (IRLS)
  9. Clipped ±3σ
  10. ★ RQNN (ours)

Usage:
  python -m qnn_eeg_filtering.compare_filters
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter, medfilt, butter, filtfilt
from scipy.ndimage import uniform_filter1d
from sklearn.linear_model import RANSACRegressor, HuberRegressor

from .rqnn_model import RQNNFilter
from .run_denoising import make_demo_signal, compute_metrics

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "results")


# ---------------------------------------------------------------
# Classical filter implementations
# ---------------------------------------------------------------

def lowpass_filter(signal, fs, cutoff=5.0, order=4):
    b, a = butter(order, cutoff / (fs / 2), btype="low")
    return filtfilt(b, a, signal)


def bandpass_filter(signal, fs, low=0.5, high=10.0, order=4):
    b, a = butter(order, [low / (fs / 2), high / (fs / 2)], btype="band")
    return filtfilt(b, a, signal)


def moving_average(signal, size=21):
    return uniform_filter1d(signal, size=size)


def savgol(signal, window=21, order=3):
    return savgol_filter(signal, window_length=window, polyorder=order)


def median_filter(signal, kernel=21):
    return medfilt(signal, kernel_size=kernel)


def ransac_filter(signal):
    X = np.arange(len(signal)).reshape(-1, 1)
    ransac = RANSACRegressor(random_state=42)
    ransac.fit(X, signal)
    return ransac.predict(X)


def huber_filter(signal):
    X = np.arange(len(signal)).reshape(-1, 1)
    huber = HuberRegressor()
    huber.fit(X, signal)
    return huber.predict(X)


def cauchy_robust_filter(signal, gamma=0.5, iterations=10):
    result = signal.copy()
    for _ in range(iterations):
        residuals = signal - result
        weights = 1.0 / (1.0 + (residuals / gamma) ** 2)
        result = weights * signal + (1 - weights) * result
    return result


def clip_filter(signal, n_sigma=3.0):
    mu = np.mean(signal)
    std = np.std(signal)
    return np.clip(signal, mu - n_sigma * std, mu + n_sigma * std)


# ---------------------------------------------------------------
# Main comparison
# ---------------------------------------------------------------

def run_comparison(
    noisy_signal: np.ndarray,
    clean_signal: np.ndarray,
    fs: float = 200.0,
    rqnn_kwargs: dict = None,
    save_dir: str = None,
):
    """
    Run all filters on the signal and return a comparison dict.

    Parameters
    ----------
    noisy_signal : array (T,)
    clean_signal : array (T,)
    fs : float
    rqnn_kwargs : dict
        Extra kwargs for RQNNFilter.
    save_dir : str or None
        Directory to save plots to.

    Returns
    -------
    results : dict  {name: {"filtered": array, "metrics": dict}}
    """
    if save_dir is None:
        save_dir = RESULTS_DIR
    os.makedirs(save_dir, exist_ok=True)

    filters = {
        "Low-Pass (5 Hz)":          lambda s: lowpass_filter(s, fs),
        "Band-Pass (0.5–10 Hz)":    lambda s: bandpass_filter(s, fs),
        "Moving Average (w=21)":    lambda s: moving_average(s),
        "Savitzky-Golay (w=21,o=3)":lambda s: savgol(s),
        "Median (k=21)":            lambda s: median_filter(s),
        "RANSAC":                   lambda s: ransac_filter(s),
        "Huber Robust":             lambda s: huber_filter(s),
        "Cauchy Robust":            lambda s: cauchy_robust_filter(s),
        "Clipped (±3σ)":            lambda s: clip_filter(s),
    }

    # RQNN
    kw = rqnn_kwargs or {}
    if "x_range" not in kw:
        margin = 2.0 * np.std(noisy_signal)
        kw["x_range"] = (
            float(noisy_signal.min() - margin),
            float(noisy_signal.max() + margin),
        )
    rqnn = RQNNFilter(**kw)

    results = {}

    print("\n" + "=" * 65)
    print("  Filter Comparison")
    print("=" * 65)

    # Classical filters
    for name, fn in filters.items():
        try:
            filt = fn(noisy_signal)
            m = compute_metrics(clean_signal, noisy_signal, filt)
            results[name] = {"filtered": filt, "metrics": m}
            print(f"  ✅ {name:30s}  RMSE={m['RMSE_filtered']:.4f}  "
                  f"SNR↑={m['SNR_improvement_dB']:+.2f} dB  "
                  f"r={m['Correlation']:.4f}")
        except Exception as e:
            print(f"  ❌ {name:30s}  error: {e}")

    # RQNN (default params)
    print(f"\n  🔬 Running RQNN (default) ({rqnn})...")
    rqnn_out = rqnn.filter_signal(noisy_signal, verbose=False)
    m = compute_metrics(clean_signal, noisy_signal, rqnn_out)
    results["★ RQNN (default)"] = {"filtered": rqnn_out, "metrics": m}
    print(f"  ✅ {'★ RQNN (default)':30s}  RMSE={m['RMSE_filtered']:.4f}  "
          f"SNR↑={m['SNR_improvement_dB']:+.2f} dB  "
          f"r={m['Correlation']:.4f}")

    # RQNN (PSO-optimized)
    print(f"\n  🔬 Running PSO-optimized RQNN...")
    from .pso_optimizer import PSOOptimizer
    pso = PSOOptimizer(n_particles=30, n_iterations=40, seed=42)
    seg_len = max(100, len(noisy_signal) // 2)  # optimise on 50% of signal
    best = pso.optimize(
        noisy_signal[:seg_len], clean_signal[:seg_len],
        base_rqnn_kwargs=dict(kw), verbose=True,
    )
    kw_opt = dict(kw)
    kw_opt.update(best)
    rqnn_opt = RQNNFilter(**kw_opt)
    rqnn_opt_out = rqnn_opt.filter_signal(noisy_signal, verbose=False)
    m = compute_metrics(clean_signal, noisy_signal, rqnn_opt_out)
    results["★ RQNN (PSO-tuned)"] = {"filtered": rqnn_opt_out, "metrics": m}
    print(f"  ✅ {'★ RQNN (PSO-tuned)':30s}  RMSE={m['RMSE_filtered']:.4f}  "
          f"SNR↑={m['SNR_improvement_dB']:+.2f} dB  "
          f"r={m['Correlation']:.4f}")

    # --- Summary table ---
    print("\n" + "-" * 75)
    print(f"  {'Filter':32s} {'RMSE':>8s} {'SNR↑(dB)':>10s} {'Corr':>8s}")
    print("-" * 75)
    sorted_results = sorted(results.items(),
                            key=lambda kv: kv[1]["metrics"]["RMSE_filtered"])
    for name, r in sorted_results:
        m = r["metrics"]
        marker = " ◀" if "RQNN" in name else ""
        print(f"  {name:32s} {m['RMSE_filtered']:8.4f} "
              f"{m['SNR_improvement_dB']:+10.2f} "
              f"{m['Correlation']:8.4f}{marker}")
    print("-" * 75)

    # --- Plots ---
    _plot_comparison(noisy_signal, clean_signal, results,
                     save_path=os.path.join(save_dir, "filter_comparison.png"))
    _plot_bar_chart(results,
                    save_path=os.path.join(save_dir, "filter_bar_chart.png"))

    return results


def _plot_comparison(noisy, clean, results, save_path=None):
    """Waveform overlay plot."""
    n = len(results) + 2  # noisy + clean + each filter
    fig, axes = plt.subplots(n, 1, figsize=(16, 2.5 * n), sharex=True)

    t = np.arange(len(noisy))

    # Raw noisy
    axes[0].plot(t, noisy, color="#e74c3c", linewidth=0.5, alpha=0.7)
    axes[0].set_title("Noisy Input", fontweight="bold")
    axes[0].grid(True, alpha=0.2)

    # Clean
    axes[1].plot(t, clean, color="#3498db", linewidth=1.0)
    axes[1].set_title("Clean Reference", fontweight="bold")
    axes[1].grid(True, alpha=0.2)

    colors = plt.cm.tab10(np.linspace(0, 1, len(results)))
    for i, (name, r) in enumerate(results.items()):
        ax = axes[i + 2]
        ax.plot(t, r["filtered"], color=colors[i], linewidth=0.8)
        ax.plot(t, clean, color="#3498db", linewidth=0.5, alpha=0.3)
        m = r["metrics"]
        ax.set_title(
            f"{name}  |  RMSE={m['RMSE_filtered']:.4f}  "
            f"SNR↑={m['SNR_improvement_dB']:+.2f} dB",
            fontweight="bold" if "RQNN" in name else "normal",
        )
        ax.grid(True, alpha=0.2)

    axes[-1].set_xlabel("Sample")
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"\n  Comparison plot saved → {save_path}")
    plt.show()


def _plot_bar_chart(results, save_path=None):
    """Bar chart of RMSE and SNR improvement."""
    names = list(results.keys())
    rmses = [results[n]["metrics"]["RMSE_filtered"] for n in names]
    snrs = [results[n]["metrics"]["SNR_improvement_dB"] for n in names]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

    colors = ["#2ecc71" if "RQNN" in n else "#3498db" for n in names]

    bars1 = ax1.barh(names, rmses, color=colors, edgecolor="white")
    ax1.set_xlabel("RMSE (lower is better)")
    ax1.set_title("Filtering RMSE Comparison", fontweight="bold")
    ax1.invert_yaxis()
    ax1.grid(True, axis="x", alpha=0.3)

    bars2 = ax2.barh(names, snrs, color=colors, edgecolor="white")
    ax2.set_xlabel("SNR Improvement (dB, higher is better)")
    ax2.set_title("SNR Improvement Comparison", fontweight="bold")
    ax2.invert_yaxis()
    ax2.grid(True, axis="x", alpha=0.3)
    ax2.axvline(0, color="gray", linestyle="--", alpha=0.5)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"  Bar chart saved → {save_path}")
    plt.show()


def main():
    """Demo comparison using built-in test signal."""
    print("=" * 60)
    print("  Filter Comparison — Demo")
    print("=" * 60)

    t, clean, noisy, fs = make_demo_signal(duration=2.0, noise_scale=0.5)
    print(f"\nSignal: {len(t)} samples, {fs} Hz")

    run_comparison(noisy, clean, fs=fs)


if __name__ == "__main__":
    main()
