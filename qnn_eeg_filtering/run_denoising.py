"""
Main denoising script for RQNN filter.

Usage:
  # Quick demo with a built-in test signal
  python -m qnn_eeg_filtering.run_denoising

  # From your own code — pass any 1-D numpy array:
  from qnn_eeg_filtering import RQNNFilter
  filt = RQNNFilter(x_range=(signal.min()-1, signal.max()+1))
  clean = filt.filter_signal(noisy_signal, verbose=True)
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from .rqnn_model import RQNNFilter
from .pso_optimizer import PSOOptimizer

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "results")


def make_demo_signal(
    duration: float = 2.0,
    fs: float = 200.0,
    noise_scale: float = 0.5,
    seed: int = 42,
):
    """
    Create a non-stationary test signal + heavy-tailed noise.
    Real EEG is highly non-stationary with non-Gaussian artifacts.
    Savitzky-Golay is optimal for stationary Gaussian noise, but RQNN
    excels in non-stationary/non-Gaussian (heavy-tailed) environments.
    """
    rng = np.random.default_rng(seed)
    t = np.arange(0, duration, 1.0 / fs)
    
    # Clean signal: non-stationary (changing frequencies over time)
    # Slow drift + burst of higher frequency
    clean = np.sin(2 * np.pi * 3 * t)
    
    # Add a sudden 15Hz burst in the middle
    burst_mask = (t > 0.8) & (t < 1.2)
    clean[burst_mask] += 1.5 * np.sin(2 * np.pi * 15 * t[burst_mask])
    
    # Add sudden amplitude changes (non-stationary envelope)
    clean *= (1.0 + 0.5 * np.sin(2 * np.pi * 1 * t))
    
    # Noise: Heavy-tailed (Laplace) distribution to simulate spikes/artifacts
    noise = rng.laplace(0, noise_scale, len(t))
    
    # Occasional extreme artifacts
    spikes = rng.choice([-1, 0, 1], size=len(t), p=[0.02, 0.96, 0.02])
    noise += spikes * 3.0
    
    noisy = clean + noise
    return t, clean, noisy, fs


def compute_metrics(clean, noisy, filtered):
    """Compute SNR improvement, RMSE, and correlation coefficient."""
    rmse_noisy = np.sqrt(np.mean((noisy - clean) ** 2))
    rmse_filt = np.sqrt(np.mean((filtered - clean) ** 2))
    snr_before = 10 * np.log10(
        np.mean(clean ** 2) / np.mean((noisy - clean) ** 2)
    )
    noise_after = np.mean((filtered - clean) ** 2)
    if noise_after < 1e-15:
        snr_after = 100.0
    else:
        snr_after = 10 * np.log10(np.mean(clean ** 2) / noise_after)
    corr = np.corrcoef(clean, filtered)[0, 1]

    return {
        "RMSE_noisy": rmse_noisy,
        "RMSE_filtered": rmse_filt,
        "SNR_before_dB": snr_before,
        "SNR_after_dB": snr_after,
        "SNR_improvement_dB": snr_after - snr_before,
        "Correlation": corr,
    }


def plot_results(t, clean, noisy, filtered, metrics, save_path=None):
    """Plot noisy vs RQNN-filtered vs clean reference."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)

    axes[0].plot(t, noisy, color="#e74c3c", alpha=0.7, linewidth=0.6)
    axes[0].set_title("Noisy Input Signal", fontsize=13, fontweight="bold")
    axes[0].set_ylabel("Amplitude")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, filtered, color="#2ecc71", linewidth=1.2, label="RQNN filtered")
    axes[1].plot(t, clean, color="#3498db", linewidth=1.0, alpha=0.7,
                 linestyle="--", label="Clean reference")
    axes[1].set_title("RQNN Filtered vs Clean Reference", fontsize=13,
                      fontweight="bold")
    axes[1].set_ylabel("Amplitude")
    axes[1].legend(fontsize=10)
    axes[1].grid(True, alpha=0.3)

    residual = filtered - clean
    axes[2].plot(t, residual, color="#9b59b6", linewidth=0.6)
    axes[2].axhline(0, color="gray", linestyle="--", alpha=0.5)
    axes[2].set_title(
        f"Residual  |  RMSE={metrics['RMSE_filtered']:.4f}  "
        f"SNR↑={metrics['SNR_improvement_dB']:.2f} dB  "
        f"r={metrics['Correlation']:.4f}",
        fontsize=12,
    )
    axes[2].set_ylabel("Error")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"  Plot saved → {save_path}")
    plt.show()


def run_rqnn_on_signal(
    signal: np.ndarray,
    reference: np.ndarray = None,
    fs: float = 200.0,
    use_pso: bool = False,
    pso_kwargs: dict = None,
    rqnn_kwargs: dict = None,
    verbose: bool = True,
):
    """
    Convenience function to run RQNN on any 1-D signal.

    Parameters
    ----------
    signal : array (T,)
        Noisy input signal.
    reference : array (T,) or None
        Clean reference for metrics. If None, metrics are skipped.
    fs : float
        Sampling frequency.
    use_pso : bool
        Whether to run PSO to find optimal RQNN params first.
    pso_kwargs : dict
        Extra kwargs for PSOOptimizer.
    rqnn_kwargs : dict
        Extra kwargs for RQNNFilter constructor.
    verbose : bool
        Print progress.

    Returns
    -------
    filtered : array (T,)
    metrics : dict or None
    rqnn : RQNNFilter
    """
    if rqnn_kwargs is None:
        rqnn_kwargs = {}

    # Auto-set x_range from signal if not provided
    if "x_range" not in rqnn_kwargs:
        margin = 2.0 * np.std(signal)
        rqnn_kwargs["x_range"] = (
            float(signal.min() - margin),
            float(signal.max() + margin),
        )

    if use_pso and reference is not None:
        if verbose:
            print("Running PSO to optimise RQNN parameters...")
        pso_kw = pso_kwargs or {}
        pso = PSOOptimizer(**pso_kw)

        # Use first 20% as optimisation segment for speed
        seg_len = max(100, len(signal) // 5)
        best = pso.optimize(
            signal[:seg_len], reference[:seg_len],
            base_rqnn_kwargs=rqnn_kwargs, verbose=verbose,
        )
        rqnn_kwargs.update(best)

    rqnn = RQNNFilter(**rqnn_kwargs)
    if verbose:
        print(f"\nRQNN config: {rqnn}")
        print("Filtering signal...")

    filtered = rqnn.filter_signal(signal, verbose=verbose)

    metrics = None
    if reference is not None:
        metrics = compute_metrics(reference, signal, filtered)
        if verbose:
            print("\n📊 Metrics:")
            for k, v in metrics.items():
                print(f"   {k:25s} = {v:.4f}")

    return filtered, metrics, rqnn


def main():
    """Demo with built-in test signal."""
    os.makedirs(RESULTS_DIR, exist_ok=True)

    print("=" * 60)
    print("  RQNN Signal Denoising — Demo")
    print("=" * 60)

    t, clean, noisy, fs = make_demo_signal()
    print(f"\nSignal: {len(t)} samples, {fs} Hz, duration {t[-1]:.1f}s")

    filtered, metrics, rqnn = run_rqnn_on_signal(
        noisy, reference=clean, fs=fs,
        use_pso=True,
        pso_kwargs={"n_particles": 15, "n_iterations": 20, "seed": 42},
        verbose=True,
    )

    save_path = os.path.join(RESULTS_DIR, "rqnn_demo_result.png")
    plot_results(t, clean, noisy, filtered, metrics, save_path=save_path)


if __name__ == "__main__":
    main()
