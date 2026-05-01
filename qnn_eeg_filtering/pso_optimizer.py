"""
Particle Swarm Optimization (PSO) for RQNN hyperparameter tuning.

Optimises RQNN parameters (mass, sigma, beta, beta_d, hbar) by
minimising RMSE between the RQNN-filtered output and a reference
(clean or smoothed) signal, following the approach in Gandhi et al.
(IEEE TNNLS 2014).
"""

import numpy as np
from typing import Dict, Tuple, Optional, Callable
from .rqnn_model import RQNNFilter


def _rmse(a: np.ndarray, b: np.ndarray) -> float:
    return np.sqrt(np.mean((a - b) ** 2))


def _snr_improvement(clean: np.ndarray, noisy: np.ndarray,
                     filtered: np.ndarray) -> float:
    """SNR improvement in dB."""
    noise_before = np.mean((noisy - clean) ** 2)
    noise_after = np.mean((filtered - clean) ** 2)
    if noise_after < 1e-15:
        return 100.0
    return 10.0 * np.log10(noise_before / noise_after)



def _rmse_zero_lag(est: np.ndarray, ref: np.ndarray) -> float:
    rmse = np.sqrt(np.mean((est - ref) ** 2))
    cc = np.correlate(est - np.mean(est), ref - np.mean(ref), mode='full')
    lag = abs(np.argmax(cc) - (len(est) - 1))
    # Reduced from 10.0 — original penalty dominated RMSE, forcing zero-lag
    # at the cost of filter quality. 0.3 penalises lag without suppressing noise reduction.
    return rmse + lag * 0.3


def _fitness_optimal_smooth(est: np.ndarray, ref: np.ndarray) -> float:
    # 1. Base tracking (RMSE against reference/noisy signal)
    rmse = np.sqrt(np.mean((est - ref) ** 2))

    # 2. Smoothness: penalise high-frequency jitter in the estimate
    roughness = np.sqrt(np.mean(np.diff(est) ** 2))

    # 3. Phase delay: soft penalty so lag matters but doesn't swamp RMSE
    cc = np.correlate(est - np.mean(est), ref - np.mean(ref), mode='full')
    lag = abs(np.argmax(cc) - (len(est) - 1))

    # Weights: RMSE (shape) + roughness (noise rejection) + lag (responsiveness)
    # Reduced lag weight 1.0→0.3: lag of 1 sample no longer dominates over RMSE.
    return rmse + (roughness * 1.5) + (lag * 0.3)

class PSOOptimizer:
    """
    PSO to find optimal RQNN hyperparameters.

    Parameters
    ----------
    param_bounds : dict
        Keys are RQNN parameter names, values are (low, high) tuples.
        Default bounds are provided if not specified.
    n_particles : int
        Swarm size (default 20).
    n_iterations : int
        Number of PSO iterations (default 30).
    w : float
        Inertia weight (default 0.7).
    c1 : float
        Cognitive coefficient (default 1.5).
    c2 : float
        Social coefficient (default 1.5).
    fitness_fn : str
        'rmse' (default) or 'snr'.
    seed : int or None
        Random seed for reproducibility.
    """

    DEFAULT_BOUNDS = {
        "mass":           (0.5, 15.0),   # expanded — prior runs hit 5.0 ceiling
        "sigma":          (0.05, 3.0),   # expanded — prior runs hit both 0.1 floor and 2.0 ceiling
        "beta":           (0.05, 2.0),   # expanded — prior runs hit 1.0 ceiling
        "beta_d":         (0.001, 0.1),  # expanded — prior runs hit 0.05 ceiling
        "hbar":           (0.5, 6.0),    # expanded — prior runs hit 3.0 ceiling
        "gamma":          (0.1, 20.0),   # expanded — prior runs hit 10.0 ceiling
        "bias_strength":  (0.05, 0.95),  # expanded — prior runs hit 0.8 ceiling
    }

    def __init__(
        self,
        param_bounds: Optional[Dict[str, Tuple[float, float]]] = None,
        n_particles: int = 50,
        n_iterations: int = 100,
        w: float = 0.9,
        w_min: float = 0.4,
        c1: float = 1.5,
        c2: float = 1.5,
        fitness_fn: str = "optimal_smooth",
        seed: Optional[int] = None,
    ):
        self.bounds = param_bounds or self.DEFAULT_BOUNDS
        self.param_names = list(self.bounds.keys())
        self.n_dim = len(self.param_names)
        self.n_particles = n_particles
        self.n_iterations = n_iterations
        self.w = w
        self.w_min = w_min
        self.c1 = c1
        self.c2 = c2
        self.fitness_fn = fitness_fn
        self.rng = np.random.default_rng(seed)

        # Results
        self.best_params: Dict[str, float] = {}
        self.best_fitness: float = np.inf
        self.history: list = []

    def _evaluate(
        self,
        params: np.ndarray,
        noisy_signal: np.ndarray,
        reference_signal: np.ndarray,
        base_rqnn_kwargs: dict,
    ) -> float:
        """
        Run RQNN with given params and return fitness (lower = better).
        """
        kwargs = dict(base_rqnn_kwargs)
        for i, name in enumerate(self.param_names):
            kwargs[name] = float(params[i])

        try:
            filt = RQNNFilter(**kwargs)
            y_est = filt.filter_signal(noisy_signal)
            if self.fitness_fn == "snr":
                # Negate because PSO minimises
                return -_snr_improvement(reference_signal, noisy_signal, y_est)
            elif self.fitness_fn == "rmse_zero_lag":
                return _rmse_zero_lag(y_est, reference_signal)
            elif self.fitness_fn == "optimal_smooth":
                return _fitness_optimal_smooth(y_est, reference_signal)
            else:
                return _rmse(y_est, reference_signal)
        except Exception:
            return 1e10  # penalty for invalid params

    def optimize(
        self,
        noisy_signal: np.ndarray,
        reference_signal: np.ndarray,
        base_rqnn_kwargs: Optional[dict] = None,
        verbose: bool = True,
    ) -> Dict[str, float]:
        """
        Run PSO optimisation.

        Parameters
        ----------
        noisy_signal : array (T,)
            The noisy input signal to filter.
        reference_signal : array (T,)
            Clean / smoothed reference for fitness evaluation.
        base_rqnn_kwargs : dict
            Fixed RQNN constructor kwargs (n_lattice, x_range, etc.)
            that are NOT being optimised.
        verbose : bool
            Print progress.

        Returns
        -------
        best_params : dict
            Optimal parameter values keyed by name.
        """
        if base_rqnn_kwargs is None:
            base_rqnn_kwargs = {}

        lows = np.array([self.bounds[k][0] for k in self.param_names])
        highs = np.array([self.bounds[k][1] for k in self.param_names])

        # --- Initialise swarm ---
        positions = self.rng.uniform(lows, highs, (self.n_particles, self.n_dim))
        velocities = self.rng.uniform(
            -(highs - lows) * 0.1,
            (highs - lows) * 0.1,
            (self.n_particles, self.n_dim),
        )

        p_best_pos = positions.copy()
        p_best_fit = np.full(self.n_particles, np.inf)
        g_best_pos = positions[0].copy()
        g_best_fit = np.inf

        self.history = []

        if verbose:
            print(f"PSO: {self.n_particles} particles × {self.n_iterations} iters")
            print(f"  Optimising: {self.param_names}")

        w_max = self.w

        for it in range(self.n_iterations):
            # Linear inertia decay: explore early (w_max), exploit late (w_min)
            w = w_max - (w_max - self.w_min) * it / max(self.n_iterations - 1, 1)

            for p in range(self.n_particles):
                fit = self._evaluate(
                    positions[p], noisy_signal, reference_signal, base_rqnn_kwargs
                )

                if fit < p_best_fit[p]:
                    p_best_fit[p] = fit
                    p_best_pos[p] = positions[p].copy()

                if fit < g_best_fit:
                    g_best_fit = fit
                    g_best_pos = positions[p].copy()

            self.history.append(g_best_fit)

            if verbose:
                print(f"  Iter {it+1:3d}/{self.n_iterations} | "
                      f"w={w:.3f} | best={g_best_fit:.6f}")

            # --- Update velocities and positions ---
            r1 = self.rng.uniform(0, 1, (self.n_particles, self.n_dim))
            r2 = self.rng.uniform(0, 1, (self.n_particles, self.n_dim))

            velocities = (
                w * velocities
                + self.c1 * r1 * (p_best_pos - positions)
                + self.c2 * r2 * (g_best_pos - positions)
            )

            positions = positions + velocities

            # Clamp to bounds
            positions = np.clip(positions, lows, highs)

        # Store results
        self.best_fitness = g_best_fit
        self.best_params = {
            name: float(g_best_pos[i]) for i, name in enumerate(self.param_names)
        }

        if verbose:
            print(f"\n  ✅ PSO complete. Best params:")
            for k, v in self.best_params.items():
                print(f"     {k:8s} = {v:.6f}")

        return self.best_params
