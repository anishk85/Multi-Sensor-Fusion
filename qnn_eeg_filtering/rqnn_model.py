"""
Recurrent Quantum Neural Network (RQNN) Filter.

Implements the Schrödinger-wave-equation-based signal filter from:
Gandhi, V., Prasad, G., Coyle, D., Behera, L., & McGinnity, T. M. (2014).
"Quantum Neural Network-Based EEG Filtering for a Brain-Computer Interface."
IEEE TNNLS, 25(2), 278-288.

Architecture:
  1. A 1-D spatial lattice x ∈ [x_min, x_max] with N nodes.
  2. Gaussian kernel functions g_i(x,t) centred on lattice nodes.
  3. Potential field V(x,t) = Σ W_i(t) * g_i(x,t)  — learned via
     unsupervised Hebbian rule.
  4. Wave function ψ(x,t) evolved via the time-dependent Schrödinger
     equation using Crank-Nicolson discretisation.
  5. PDF f(x,t) = |ψ(x,t)|².
  6. Signal estimate ŷ(t) = E[x | f] (expectation) or argmax f (MLE).
  7. Unsupervised Hebbian weight update driven by innovation ν = y − ŷ.
"""

import numpy as np
from scipy.linalg import solve_banded
from typing import Tuple


class RQNNFilter:
    """
    Recurrent Quantum Neural Network filter for 1-D signal denoising.

    Parameters
    ----------
    n_lattice : int
        Number of spatial lattice nodes (default 200).
    x_range : tuple (x_min, x_max)
        Spatial range — should cover the expected signal amplitude
        range with some margin.
    sigma : float
        Gaussian kernel width (default 0.3).
    hbar : float
        Reduced Planck constant analogue (default 1.0).
    mass : float
        Mass parameter in the Schrödinger equation (default 1.0).
    dt : float
        Time step for Crank-Nicolson evolution (default 0.005).
    beta : float
        Learning rate for Hebbian weight update (default 0.5).
    beta_d : float
        De-learning (forgetting) rate (default 0.005).
    estimate_mode : str
        'expectation' for E[x|f] or 'mle' for argmax f(x,t).
    """

    def __init__(
        self,
        n_lattice: int = 200,
        x_range: Tuple[float, float] = (-5.0, 5.0),
        sigma: float = 0.3,
        hbar: float = 1.0,
        mass: float = 1.0,
        dt: float = 0.005,
        beta: float = 0.5,
        beta_d: float = 0.005,
        gamma: float = 2.0,
        bias_strength: float = 0.4,
        estimate_mode: str = "expectation",
    ):
        self.n_lattice = n_lattice
        self.x_min, self.x_max = x_range
        self.sigma = sigma
        self.hbar = hbar
        self.mass = mass
        self.dt = dt
        self.beta = beta
        self.beta_d = beta_d
        self.gamma = gamma  # observation-driven potential strength
        self.bias_strength = bias_strength  # wave packet excitation strength
        self.estimate_mode = estimate_mode

        # Build spatial grid and internal structures
        self._build_lattice()

    # ------------------------------------------------------------------
    # Initialisation helpers
    # ------------------------------------------------------------------

    def _build_lattice(self):
        """Build/rebuild all lattice-dependent structures."""
        self.x = np.linspace(self.x_min, self.x_max, self.n_lattice)
        self.dx = self.x[1] - self.x[0]
        self.centres = self.x.copy()
        self.W = np.zeros(self.n_lattice)
        self.G = self._build_kernel_matrix()
        self._init_wavefunction()

    def _init_wavefunction(self, centre: float = None):
        """Initialise ψ as a normalised Gaussian wave packet."""
        if centre is None:
            centre = (self.x_min + self.x_max) / 2.0
        sig0 = (self.x_max - self.x_min) / 8.0
        psi = np.exp(-(self.x - centre) ** 2 / (2 * sig0 ** 2)).astype(
            np.complex128
        )
        norm = np.sqrt(np.trapz(np.abs(psi) ** 2, self.x))
        self.psi = psi / max(norm, 1e-15)

    def _build_kernel_matrix(self) -> np.ndarray:
        """G[i, j] = exp(-(x_j - c_i)^2 / (2 σ^2))."""
        diff = self.x[np.newaxis, :] - self.centres[:, np.newaxis]
        return np.exp(-diff ** 2 / (2.0 * self.sigma ** 2))

    # ------------------------------------------------------------------
    # Crank-Nicolson evolution
    # ------------------------------------------------------------------

    def _evolve_psi(self, V: np.ndarray):
        """
        One Crank-Nicolson step for the time-dependent Schrödinger eqn.

        H = T + V,  T = -ℏ²/(2m) ∂²/∂x²

        (I + i·α H) ψ^{n+1} = (I - i·α H) ψ^n
        where α = dt / (2ℏ).
        """
        N = self.n_lattice
        alpha = self.dt / (2.0 * self.hbar)
        r = self.hbar / (2.0 * self.mass * self.dx ** 2)  # kinetic coeff

        # --- RHS: b = (I - i·α H) ψ^n ---
        # Kinetic part (tridiagonal Laplacian)
        kinetic = np.zeros(N, dtype=np.complex128)
        kinetic[1:-1] = r * (self.psi[2:] - 2 * self.psi[1:-1] + self.psi[:-2])
        kinetic[0] = r * (self.psi[1] - 2 * self.psi[0])
        kinetic[-1] = r * (-2 * self.psi[-1] + self.psi[-2])

        H_psi = -kinetic + V * self.psi
        rhs = self.psi - 1j * alpha * H_psi

        # --- LHS: solve (I + i·α H) ψ^{n+1} = rhs ---
        main_diag = 1.0 + 1j * alpha * (2 * r + V)
        off_val = 1j * alpha * r

        # Use scipy banded solver (LAPACK dgbsv — compiled C, ~100× faster)
        ab = np.zeros((3, N), dtype=np.complex128)
        ab[0, 1:] = off_val      # upper diagonal
        ab[1, :] = main_diag     # main diagonal
        ab[2, :-1] = off_val     # lower diagonal

        self.psi = solve_banded((1, 1), ab, rhs)

        # Re-normalise
        norm = np.sqrt(np.trapz(np.abs(self.psi) ** 2, self.x))
        if norm > 1e-15:
            self.psi /= norm

    # ------------------------------------------------------------------
    # Core computational methods
    # ------------------------------------------------------------------

    def _potential(self, y_obs: float = None) -> np.ndarray:
        """
        V(x, t) = V_learned(x) + V_obs(x)

        V_learned = Σ_i W_i * g_i(x)  (Hebbian-learned)
        V_obs     = γ * (x - y_obs)²   (observation-driven confining well)

        The observation-driven term is the key mechanism from the paper:
        the input signal directly modulates the potential field, creating
        a confining well that guides the wave packet toward the signal.
        """
        V = self.W @ self.G
        if y_obs is not None:
            # Harmonic confining potential centred at observation
            V += self.gamma * (self.x - y_obs) ** 2
        return V

    def _pdf(self) -> np.ndarray:
        """f(x, t) = |ψ(x, t)|²."""
        pdf = np.abs(self.psi) ** 2
        # Normalise as a probability distribution
        total = np.trapz(pdf, self.x)
        if total > 1e-15:
            pdf /= total
        return pdf

    def _estimate(self, pdf: np.ndarray) -> float:
        """Extract signal estimate from the PDF."""
        if self.estimate_mode == "mle":
            return self.x[np.argmax(pdf)]
        else:
            return float(np.trapz(self.x * pdf, self.x))

    def _update_weights(self, innovation: float, y_est: float):
        """
        Hebbian weight update:
          ΔW_i = β · ν · g_i(ŷ) − β_d · W_i

        where ν = y_observed − ŷ is the innovation.
        """
        g_at_est = np.exp(
            -(y_est - self.centres) ** 2 / (2.0 * self.sigma ** 2)
        )
        self.W += self.beta * innovation * g_at_est - self.beta_d * self.W

    def _bias_wavepacket(self, y_obs: float, strength: float = 0.3):
        """
        Apply a soft bias to the wave packet toward the observed value.
        This models the 'excitation' of the neural lattice by the input
        signal, as described in the paper.
        """
        bias = np.exp(
            -(self.x - y_obs) ** 2 / (2.0 * self.sigma ** 2)
        ).astype(np.complex128)
        # Mix the bias into psi
        self.psi = (1 - strength) * self.psi + strength * bias * np.sign(
            self.psi.real + 1e-15
        )
        # Re-normalise
        norm = np.sqrt(np.trapz(np.abs(self.psi) ** 2, self.x))
        if norm > 1e-15:
            self.psi /= norm

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def reset(self):
        """Reset internal state for a new signal."""
        self.W = np.zeros(self.n_lattice)
        self._init_wavefunction()

    def filter_signal(
        self,
        signal: np.ndarray,
        n_substeps: int = 3,
        verbose: bool = False,
    ) -> np.ndarray:
        """
        Filter a 1-D signal sample-by-sample.

        Parameters
        ----------
        signal : array-like, shape (T,)
            The noisy input signal.
        n_substeps : int
            Number of Crank-Nicolson sub-steps per sample.
        verbose : bool
            Print progress every 10 %.

        Returns
        -------
        y_est : np.ndarray, shape (T,)
            The denoised (filtered) signal.
        """
        self.reset()
        signal = np.asarray(signal, dtype=np.float64)
        T = len(signal)
        y_est = np.empty(T)

        for t in range(T):
            y_obs = signal[t]

            # --- Bias wave packet toward observation ---
            self._bias_wavepacket(y_obs, strength=self.bias_strength)

            # --- Compute observation-driven potential and evolve ψ ---
            V = self._potential(y_obs=y_obs)
            for _ in range(n_substeps):
                self._evolve_psi(V)

            # --- Extract estimate ---
            pdf = self._pdf()
            y_hat = self._estimate(pdf)
            y_est[t] = y_hat

            # --- Update weights ---
            innovation = y_obs - y_hat
            self._update_weights(innovation, y_hat)

            if verbose and (t + 1) % max(1, T // 10) == 0:
                pct = 100 * (t + 1) / T
                print(
                    f"  RQNN progress: {pct:.0f}%  |  "
                    f"innovation={innovation:.4f}"
                )

        return y_est

    def get_params(self) -> dict:
        """Return current hyperparameters as a dict."""
        return {
            "n_lattice": self.n_lattice,
            "x_range": (self.x_min, self.x_max),
            "sigma": self.sigma,
            "hbar": self.hbar,
            "mass": self.mass,
            "dt": self.dt,
            "beta": self.beta,
            "beta_d": self.beta_d,
            "gamma": self.gamma,
            "bias_strength": self.bias_strength,
            "estimate_mode": self.estimate_mode,
        }

    def set_params(self, **kwargs):
        """Update hyperparameters and rebuild internal structures."""
        rebuild = False
        for k, v in kwargs.items():
            if k == "x_range":
                self.x_min, self.x_max = v
                rebuild = True
            elif k == "n_lattice":
                self.n_lattice = v
                rebuild = True
            elif hasattr(self, k):
                setattr(self, k, v)
                if k in ("sigma", "hbar", "mass", "dt"):
                    rebuild = True
            else:
                raise ValueError(f"Unknown parameter: {k}")

        if rebuild:
            self._build_lattice()

    def __repr__(self):
        return (
            f"RQNNFilter(N={self.n_lattice}, σ={self.sigma:.3f}, "
            f"ℏ={self.hbar:.3f}, m={self.mass:.3f}, "
            f"γ={self.gamma:.3f}, "
            f"β={self.beta:.4f}, β_d={self.beta_d:.4f})"
        )
