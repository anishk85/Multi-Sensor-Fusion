#!/usr/bin/env python3
"""
Retune RQNN hyperparameters per channel using PSO on real bag data.

Reads raw sensor signals from a rosbag, runs PSO (with updated bounds +
inertia decay) on each channel, and saves optimal params to
rqnn_pretrained_params.json.

Usage:
    python3 scripts/retune_rqnn_pso.py --bag /path/to/bag [--out config/rqnn_pretrained_params.json]
"""
import argparse
import json
import sys
import os
import numpy as np
from pathlib import Path
from scipy.signal import savgol_filter

sys.path.insert(0, '/home/anish/Multi-sensor-fusion')
from qnn_eeg_filtering import RQNNFilter, PSOOptimizer

CONFIG_OUT = (
    '/home/anish/Multi-sensor-fusion/src/husarion_ugv_description/'
    'config/rqnn_pretrained_params.json'
)


# ---------------------------------------------------------------------------
# Bag reading helpers
# ---------------------------------------------------------------------------

def read_bag(bag_path: str) -> dict:
    """Extract per-channel numpy arrays from rosbag."""
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import get_typestore, Stores

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    raw = {
        'imu_ax': [], 'imu_ay': [], 'imu_az': [],
        'imu_gx': [], 'imu_gy': [], 'imu_gz': [],
        'wheel_vx': [], 'wheel_wz': [],
        'visual_vx': [], 'visual_wz': [],
        'lidar_vx': [],  'lidar_wz': [],
    }

    TOPIC_MAP = {
        '/imu/data':                    'imu',
        '/diff_drive_controller/odom':  'wheel',
        '/odometry/visual':             'visual',
        '/odometry/lidar':              'lidar',
    }

    with Reader(Path(bag_path)) as reader:
        conns = [c for c in reader.connections if c.topic in TOPIC_MAP]
        for conn, _, rawdata in reader.messages(connections=conns):
            key = TOPIC_MAP[conn.topic]
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)

            if key == 'imu':
                raw['imu_ax'].append(msg.linear_acceleration.x)
                raw['imu_ay'].append(msg.linear_acceleration.y)
                raw['imu_az'].append(msg.linear_acceleration.z)
                raw['imu_gx'].append(msg.angular_velocity.x)
                raw['imu_gy'].append(msg.angular_velocity.y)
                raw['imu_gz'].append(msg.angular_velocity.z)
            elif key == 'wheel':
                raw['wheel_vx'].append(msg.twist.twist.linear.x)
                raw['wheel_wz'].append(msg.twist.twist.angular.z)
            elif key == 'visual':
                raw['visual_vx'].append(msg.twist.twist.linear.x)
                raw['visual_wz'].append(msg.twist.twist.angular.z)
            elif key == 'lidar':
                raw['lidar_vx'].append(msg.twist.twist.linear.x)
                raw['lidar_wz'].append(msg.twist.twist.angular.z)

    return {k: np.array(v, dtype=np.float64) for k, v in raw.items() if len(v) > 50}


# ---------------------------------------------------------------------------
# Per-channel sampling rates (approximate, from hardware)
# ---------------------------------------------------------------------------
CHANNEL_FS = {
    'imu_ax': 154.0, 'imu_ay': 154.0, 'imu_az': 154.0,
    'imu_gx': 154.0, 'imu_gy': 154.0, 'imu_gz': 154.0,
    'wheel_vx': 38.6, 'wheel_wz': 38.6,
    'visual_vx': 14.3, 'visual_wz': 14.3,
    'lidar_vx': 3.9,   'lidar_wz': 3.9,
}


def make_reference(signal: np.ndarray, fs: float) -> np.ndarray:
    """Savitzky-Golay smoothed reference (adaptive window)."""
    # Window: ~50ms worth of samples, must be odd and >= 5
    win = max(5, int(fs * 0.05))
    if win % 2 == 0:
        win += 1
    poly = min(3, win - 2)
    try:
        return savgol_filter(signal, window_length=win, polyorder=poly)
    except Exception:
        return signal.copy()


# ---------------------------------------------------------------------------
# Main tuning loop
# ---------------------------------------------------------------------------

def tune_channel(name: str, signal: np.ndarray, fs: float, verbose: bool = True) -> dict:
    margin = 2.0 * np.std(signal)
    x_range = (float(signal.min() - margin), float(signal.max() + margin))
    dt = float(1.0 / fs)

    reference = make_reference(signal, fs)

    # Use first 30% for PSO speed — enough to capture signal statistics
    seg = max(200, len(signal) // 3)
    noisy_seg = signal[:seg]
    ref_seg = reference[:seg]

    base_kw = {'x_range': x_range, 'dt': dt, 'n_lattice': 150}

    pso = PSOOptimizer(seed=42)  # uses new defaults: 50 particles, 100 iters
    best = pso.optimize(noisy_seg, ref_seg, base_rqnn_kwargs=base_kw, verbose=verbose)

    result = dict(best)
    result['x_range_min'] = x_range[0]
    result['x_range_max'] = x_range[1]
    result['dt'] = dt
    result['n_lattice'] = 150

    # Validate: run on full signal and check RMSE improved
    try:
        filt = RQNNFilter(**{k: v for k, v in result.items()
                             if k not in ('x_range_min', 'x_range_max')
                             and k != 'n_lattice'},
                          x_range=x_range, n_lattice=150)
        y_hat = filt.filter_signal(signal)
        rmse_raw = np.sqrt(np.mean((signal - reference) ** 2))
        rmse_filt = np.sqrt(np.mean((y_hat - reference) ** 2))
        improvement = (rmse_raw - rmse_filt) / max(rmse_raw, 1e-10) * 100
        print(f'  [{name}] RMSE improvement vs raw: {improvement:+.1f}%  '
              f'(raw={rmse_raw:.4f}  filtered={rmse_filt:.4f})')
    except Exception as e:
        print(f'  [{name}] Validation failed: {e}')

    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True, help='Path to rosbag directory')
    parser.add_argument('--out', default=CONFIG_OUT, help='Output JSON path')
    parser.add_argument('--channels', nargs='*', default=None,
                        help='Subset of channels to retune (default: all)')
    parser.add_argument('--quiet', action='store_true')
    args = parser.parse_args()

    print(f'Reading bag: {args.bag}')
    data = read_bag(args.bag)
    print(f'Loaded channels: {list(data.keys())}')

    # Load existing params to preserve any channels not in this bag
    existing = {}
    if os.path.exists(args.out):
        with open(args.out) as f:
            existing = json.load(f)

    channels = args.channels or list(data.keys())
    results = dict(existing)

    for ch in channels:
        if ch not in data:
            print(f'  [{ch}] Not in bag — skipping')
            continue
        sig = data[ch]
        fs = CHANNEL_FS.get(ch, 50.0)
        print(f'\n{"="*60}')
        print(f'  Tuning: {ch}  ({len(sig)} samples @ {fs:.1f} Hz)')
        print(f'  Signal: min={sig.min():.3f}  max={sig.max():.3f}  σ={sig.std():.4f}')
        print(f'{"="*60}')
        results[ch] = tune_channel(ch, sig, fs, verbose=not args.quiet)

    with open(args.out, 'w') as f:
        json.dump(results, f, indent=2)
    print(f'\nSaved {len(results)} channel profiles → {args.out}')


if __name__ == '__main__':
    main()
