"""
Benchmark script: Generates Force-Displacement (F-D) and Force-Velocity (F-V)
hysteresis curves for the Spencer Modified Bouc-Wen MR Damper.

Saves high-resolution plots to docs/assets/ for documentation and research verification.
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# Add python directory to path
repo_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(repo_root))

from python.mr_damper import ModifiedBoucWenMRDamper, MRDamperParameters


def run_voltage_sweep():
    damper = ModifiedBoucWenMRDamper()
    
    # Excitation parameters (from MRD_FDFV.slx system_root.xml)
    f = 10.0                 # Frequency [Hz]
    omega = 2.0 * np.pi * f  # [rad/s]
    X0 = 0.008               # Amplitude [m] (8 mm stroke)
    
    voltages = [0.0, 0.5, 1.0, 1.5, 2.0]
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
    
    # Integration span (run 5 cycles, capture the last 2 cycles for steady-state)
    t_cycles = 5
    t_end = t_cycles / f
    dt = 1e-4
    
    results = {}
    for v in voltages:
        res = damper.simulate_trajectory(
            t_span=(0.0, t_end),
            disp_func=lambda t: X0 * np.sin(omega * t - np.pi / 2.0),
            vel_func=lambda t: X0 * omega * np.cos(omega * t - np.pi / 2.0),
            volt_func=lambda t: v,
            dt=dt
        )
        # Extract steady-state cycles (last 2 cycles)
        t_steady_idx = res['time'] >= (t_end - 2.0 / f)
        results[v] = {
            'disp_mm': res['displacement'][t_steady_idx] * 1000.0,
            'vel_mps': res['velocity'][t_steady_idx],
            'force_N': res['force'][t_steady_idx],
            'time_s': res['time'][t_steady_idx],
            'res_full': res
        }

    # Set up styling
    plt.rcParams.update({
        'font.size': 11,
        'axes.labelsize': 12,
        'axes.titlesize': 13,
        'xtick.labelsize': 11,
        'ytick.labelsize': 11,
        'legend.fontsize': 10,
        'figure.titlesize': 14,
        'lines.linewidth': 2.0,
        'grid.alpha': 0.35
    })

    assets_dir = repo_root / "docs" / "assets"
    assets_dir.mkdir(parents=True, exist_ok=True)

    # 1. Force - Displacement (F-D) Plot
    fig_fd, ax_fd = plt.subplots(figsize=(8, 6), dpi=300)
    for v, c in zip(voltages, colors):
        ax_fd.plot(results[v]['disp_mm'], results[v]['force_N'], label=f'V = {v:.1f} V', color=c)
    ax_fd.set_title(r"$\mathbf{Force\ vs.\ Displacement\ (F-D)}$" + f" at f = {f:.0f} Hz, $X_0$ = {X0*1000:.0f} mm")
    ax_fd.set_xlabel("Displacement [mm]")
    ax_fd.set_ylabel("Damper Force [N]")
    ax_fd.grid(True)
    ax_fd.legend(loc='upper left', frameon=True)
    fig_fd.tight_layout()
    fig_fd.savefig(assets_dir / "fd_hysteresis_loops.png")
    plt.close(fig_fd)

    # 2. Force - Velocity (F-V) Plot
    fig_fv, ax_fv = plt.subplots(figsize=(8, 6), dpi=300)
    for v, c in zip(voltages, colors):
        ax_fv.plot(results[v]['vel_mps'], results[v]['force_N'], label=f'V = {v:.1f} V', color=c)
    ax_fv.set_title(r"$\mathbf{Force\ vs.\ Velocity\ (F-V)}$" + f" at f = {f:.0f} Hz, $X_0$ = {X0*1000:.0f} mm")
    ax_fv.set_xlabel("Piston Velocity [m/s]")
    ax_fv.set_ylabel("Damper Force [N]")
    ax_fv.grid(True)
    ax_fv.legend(loc='upper left', frameon=True)
    fig_fv.tight_layout()
    fig_fv.savefig(assets_dir / "fv_hysteresis_loops.png")
    plt.close(fig_fv)

    # 3. Dynamic Time Response Plot (for V = 1.0 V)
    fig_t, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 7), sharex=True, dpi=300)
    res_1v = results[1.0]['res_full']
    t_plot = res_1v['time']
    t_mask = t_plot <= 0.3  # first 3 cycles

    ax1.plot(t_plot[t_mask], res_1v['displacement'][t_mask] * 1000.0, color='#1f77b4', label="x(t) [mm]")
    ax1.set_ylabel("Displacement [mm]")
    ax1.grid(True)
    ax1.legend(loc='upper right')

    ax2.plot(t_plot[t_mask], res_1v['velocity'][t_mask], color='#ff7f0e', label=r"$\dot{x}(t)$ [m/s]")
    ax2.set_ylabel("Velocity [m/s]")
    ax2.grid(True)
    ax2.legend(loc='upper right')

    ax3.plot(t_plot[t_mask], res_1v['force'][t_mask], color='#2ca02c', label=r"$F_{MR}(t)$ [N]")
    ax3.set_ylabel("Damper Force [N]")
    ax3.set_xlabel("Time [s]")
    ax3.grid(True)
    ax3.legend(loc='upper right')

    fig_t.suptitle("Dynamic Transient & Steady-State Response at V = 1.0 V, f = 10 Hz", fontweight='bold')
    fig_t.tight_layout()
    fig_t.savefig(assets_dir / "time_response.png")
    plt.close(fig_t)

    # 4. Multi-frequency comparison (at V = 1.0 V)
    freqs = [1.0, 2.5, 5.0, 10.0]
    freq_colors = ['#17becf', '#bcbd22', '#e377c2', '#7f7f7f']
    fig_freq, (ax_f1, ax_f2) = plt.subplots(1, 2, figsize=(13, 5), dpi=300)

    for frq, fc in zip(freqs, freq_colors):
        w = 2.0 * np.pi * frq
        t_sim = 4.0 / frq
        r = damper.simulate_trajectory(
            t_span=(0.0, t_sim),
            disp_func=lambda t: X0 * np.sin(w * t - np.pi / 2.0),
            vel_func=lambda t: X0 * w * np.cos(w * t - np.pi / 2.0),
            volt_func=lambda t: 1.0,
            dt=1e-4 / (frq / 10.0)
        )
        ss_idx = r['time'] >= (t_sim - 2.0 / frq)
        ax_f1.plot(r['displacement'][ss_idx] * 1000.0, r['force'][ss_idx], label=f'f = {frq:.1f} Hz', color=fc)
        ax_f2.plot(r['velocity'][ss_idx], r['force'][ss_idx], label=f'f = {frq:.1f} Hz', color=fc)

    ax_f1.set_title("F-D Hysteresis Loop vs. Frequency (V = 1.0 V)")
    ax_f1.set_xlabel("Displacement [mm]")
    ax_f1.set_ylabel("Damper Force [N]")
    ax_f1.grid(True)
    ax_f1.legend(loc='upper left')

    ax_f2.set_title("F-V Hysteresis Loop vs. Frequency (V = 1.0 V)")
    ax_f2.set_xlabel("Piston Velocity [m/s]")
    ax_f2.set_ylabel("Damper Force [N]")
    ax_f2.grid(True)
    ax_f2.legend(loc='upper left')

    fig_freq.tight_layout()
    fig_freq.savefig(assets_dir / "frequency_dependence.png")
    plt.close(fig_freq)

    print("Successfully generated all hysteresis and response benchmark plots in docs/assets/")


if __name__ == "__main__":
    run_voltage_sweep()
