"""
Standalone CLI simulation runner for the Spencer Modified Bouc-Wen MR Damper.

Usage:
    python python/simulate.py --voltage 1.0 --freq 10.0 --amplitude 0.008 --duration 1.0
"""

import argparse
import sys
from pathlib import Path
import numpy as np

repo_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(repo_root))

from python.mr_damper import ModifiedBoucWenMRDamper, MRDamperParameters


def main():
    parser = argparse.ArgumentParser(description="Simulate Spencer Modified Bouc-Wen MR Damper")
    parser.add_argument("--voltage", "-v", type=float, default=1.0, help="Control voltage [V] (default: 1.0)")
    parser.add_argument("--freq", "-f", type=float, default=10.0, help="Excitation frequency [Hz] (default: 10.0)")
    parser.add_argument("--amplitude", "-a", type=float, default=0.008, help="Stroke amplitude [m] (default: 0.008 m = 8 mm)")
    parser.add_argument("--duration", "-d", type=float, default=1.0, help="Simulation duration [s] (default: 1.0)")
    parser.add_argument("--dt", type=float, default=1e-4, help="Integration time step [s] (default: 1e-4)")
    args = parser.parse_args()

    omega = 2.0 * np.pi * args.freq
    damper = ModifiedBoucWenMRDamper()

    print(f"--- Simulating Spencer Modified Bouc-Wen MR Damper ---")
    print(f"Voltage:   {args.voltage:.2f} V")
    print(f"Frequency: {args.freq:.1f} Hz (omega = {omega:.2f} rad/s)")
    print(f"Amplitude: {args.amplitude*1000.0:.1f} mm (peak-to-peak: {args.amplitude*2000.0:.1f} mm)")
    print(f"Duration:  {args.duration:.2f} s")

    res = damper.simulate_trajectory(
        t_span=(0.0, args.duration),
        disp_func=lambda t: args.amplitude * np.sin(omega * t - np.pi / 2.0),
        vel_func=lambda t: args.amplitude * omega * np.cos(omega * t - np.pi / 2.0),
        volt_func=lambda t: args.voltage,
        dt=args.dt
    )

    # Steady state analysis (last cycle)
    t_end = args.duration
    idx_ss = res['time'] >= (t_end - 1.0 / args.freq)
    f_ss = res['force'][idx_ss]
    v_ss = res['velocity'][idx_ss]
    t_ss = res['time'][idx_ss]

    max_force = np.max(f_ss)
    min_force = np.min(f_ss)
    e_diss = np.trapezoid(f_ss * v_ss, t_ss)

    print(f"\n--- Steady-State Cycle Results ---")
    print(f"Maximum Force:       {max_force:8.2f} N")
    print(f"Minimum Force:       {min_force:8.2f} N")
    print(f"Peak-to-Peak Force:  {max_force - min_force:8.2f} N")
    print(f"Dissipated Energy:   {e_diss:8.3f} J/cycle")
    print(f"Equivalent Damping:  {e_diss / (np.pi * omega * args.amplitude**2):8.2f} N*s/m")


if __name__ == '__main__':
    main()
