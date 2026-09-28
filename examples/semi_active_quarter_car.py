"""
Semi-Active Vehicle Suspension Control Benchmark:
2-DOF Quarter-Car Model comparing Passive vs. Skyhook Control using
the Spencer Modified Bouc-Wen MR Damper.

ISO 8855 Coordinate System: Positive z is upward.
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

repo_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(repo_root))

from python.mr_damper import ModifiedBoucWenMRDamper, MRDamperParameters


def run_quarter_car_simulation():
    # Quarter-Car Physical Parameters (ISO SI units)
    m_s = 320.0       # Sprung mass (chassis body corner) [kg]
    m_u = 40.0        # Unsprung mass (wheel/hub assembly) [kg]
    k_s = 22000.0     # Suspension main coil spring stiffness [N/m]
    k_t = 190000.0    # Tire vertical radial stiffness [N/m]

    damper = ModifiedBoucWenMRDamper()

    # Road profile: Discrete bump excitation (ISO road obstacle)
    bump_height = 0.05   # 50 mm bump [m]
    bump_length = 1.0    # 1 meter length [m]
    v_car = 45.0 / 3.6   # 45 km/h forward velocity [m/s]
    t_bump = bump_length / v_car
    t_start_bump = 0.5
    t_end = 2.5
    dt = 2e-4

    def road_profile(t: float) -> Tuple[float, float]:
        if t_start_bump <= t <= t_start_bump + t_bump:
            tau = (t - t_start_bump) / t_bump
            # Smooth haversine bump profile
            zr = 0.5 * bump_height * (1.0 - np.cos(2.0 * np.pi * tau))
            zr_dot = 0.5 * bump_height * (2.0 * np.pi / t_bump) * np.sin(2.0 * np.pi * tau)
            return zr, zr_dot
        return 0.0, 0.0

    # Control policies:
    # 1. Passive Off (V = 0.0 V)
    # 2. Passive On (V = 2.0 V)
    # 3. Continuous Skyhook Control:
    #    If z_s_dot * (z_s_dot - z_u_dot) >= 0 -> V = V_max
    #    Else -> V = 0.0
    cases = [
        {"name": "Passive Soft (0.0 V)", "color": "#1f77b4", "mode": "passive_0"},
        {"name": "Passive Hard (2.0 V)", "color": "#d62728", "mode": "passive_2"},
        {"name": "Skyhook Semi-Active", "color": "#2ca02c", "mode": "skyhook"}
    ]

    sim_results = {}

    for case in cases:
        mode = case["mode"]

        # Combined State Vector: [z_s, z_s_dot, z_u, z_u_dot, y, z, u]
        # Size = 7 states
        def eom(t, state):
            z_s, z_s_dot, z_u, z_u_dot, y_d, z_d, u_d = state
            zr, _ = road_profile(t)

            # Damper relative motion (piston displacement & velocity)
            # x = z_s - z_u (extension positive)
            x = z_s - z_u
            x_dot = z_s_dot - z_u_dot

            # Controller voltage determination
            if mode == "passive_0":
                v_cmd = 0.0
            elif mode == "passive_2":
                v_cmd = 2.0
            elif mode == "skyhook":
                # Classical 2-state Skyhook heuristic
                if z_s_dot * x_dot >= 0.0:
                    v_cmd = 2.0
                else:
                    v_cmd = 0.0

            # MR Damper derivatives and instantaneous force
            mr_derivs, f_mr = damper.compute_derivatives(t, np.array([y_d, z_d, u_d]), x, x_dot, v_cmd)

            # Equations of Motion:
            # Sprung mass:   m_s * z_s_ddot = -k_s * (z_s - z_u) - F_MR
            # Unsprung mass: m_u * z_u_ddot =  k_s * (z_s - z_u) + F_MR - k_t * (z_u - z_r)
            f_spring = k_s * (z_s - z_u)
            f_tire = k_t * (z_u - zr)

            z_s_ddot = (-f_spring - f_mr) / m_s
            z_u_ddot = (f_spring + f_mr - f_tire) / m_u

            return [z_s_dot, z_s_ddot, z_u_dot, z_u_ddot, mr_derivs[0], mr_derivs[1], mr_derivs[2]]

        t_eval = np.linspace(0.0, t_end, int(np.round(t_end / dt)) + 1)
        init_state = np.zeros(7, dtype=np.float64)

        sol = solve_ivp(eom, [0.0, t_end], init_state, t_eval=t_eval, method='Radau', rtol=1e-5, atol=1e-8)
        
        # Calculate sprung mass acceleration
        z_s = sol.y[0]
        z_s_dot = sol.y[1]
        z_u = sol.y[2]
        z_u_dot = sol.y[3]
        y_d = sol.y[4]
        z_d = sol.y[5]
        u_d = sol.y[6]

        x = z_s - z_u
        x_dot = z_s_dot - z_u_dot

        c0 = damper.params.c0_a + damper.params.c0_b * u_d
        c1 = damper.params.c1_a + damper.params.c1_b * u_d
        alpha = damper.params.alpha_a + damper.params.alpha_b * u_d
        y_dot = (alpha * z_d + c0 * x_dot + damper.params.k0 * (x - y_d)) / (c0 + c1)
        f_mr = c1 * y_dot + damper.params.k1 * (x - damper.params.x0)
        z_s_ddot = (-k_s * x - f_mr) / m_s

        sim_results[case["name"]] = {
            "time": sol.t,
            "z_s": z_s,
            "z_s_dot": z_s_dot,
            "z_s_ddot": z_s_ddot,
            "susp_travel": x * 1000.0,  # mm
            "color": case["color"],
            "force": f_mr
        }

    # Plot performance comparison
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True, dpi=300)

    # Road profile reference
    t_arr = sim_results["Skyhook Semi-Active"]["time"]
    road_z = np.array([road_profile(ti)[0] * 1000.0 for ti in t_arr])

    ax1.plot(t_arr, road_z, 'k--', label="Road Profile [mm]", alpha=0.5)
    for name, data in sim_results.items():
        ax1.plot(data["time"], data["z_s"] * 1000.0, label=name, color=data["color"], lw=1.8)
    ax1.set_ylabel("Body Disp. [mm]")
    ax1.set_title("Sprung Mass Vertical Displacement (Ride Comfort)", fontweight='bold')
    ax1.grid(True, alpha=0.4)
    ax1.legend(loc='upper right', frameon=True)

    for name, data in sim_results.items():
        ax2.plot(data["time"], data["z_s_ddot"], label=name, color=data["color"], lw=1.8)
    ax2.set_ylabel("Body Accel. [$m/s^2$]")
    ax2.set_title("Sprung Mass Vertical Acceleration", fontweight='bold')
    ax2.grid(True, alpha=0.4)

    for name, data in sim_results.items():
        ax3.plot(data["time"], data["susp_travel"], label=name, color=data["color"], lw=1.8)
    ax3.set_ylabel("Susp. Travel [mm]")
    ax3.set_xlabel("Time [s]")
    ax3.set_title("Suspension Deflection ($z_s - z_u$)", fontweight='bold')
    ax3.grid(True, alpha=0.4)

    fig.tight_layout()
    out_fig = repo_root / "docs" / "assets" / "quarter_car_comparison.png"
    fig.savefig(out_fig)
    plt.close(fig)
    print(f"Quarter-car comparison saved to {out_fig}")

    # Print summary performance metrics
    print("\n--- Quarter-Car ISO Vibration Performance Summary ---")
    for name, data in sim_results.items():
        rms_accel = np.sqrt(np.mean(data["z_s_ddot"]**2))
        peak_disp = np.max(np.abs(data["z_s"])) * 1000.0
        print(f"{name:25s} | RMS Body Accel: {rms_accel:5.2f} m/s^2 | Peak Body Disp: {peak_disp:5.1f} mm")


if __name__ == "__main__":
    from typing import Tuple
    run_quarter_car_simulation()
