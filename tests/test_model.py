"""
Unit tests for the Modified Bouc-Wen MR Damper simulation model.
Verifies thermodynamic passivity, parameter consistency, and numerical stability.
"""

import sys
from pathlib import Path
import numpy as np
import pytest

repo_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(repo_root))

from python.mr_damper import ModifiedBoucWenMRDamper, MRDamperParameters


def test_default_parameters():
    params = MRDamperParameters()
    assert params.c0_a == 784.0
    assert params.c0_b == 1803.0
    assert params.k0 == 3610.0
    assert params.c1_a == 14649.0
    assert params.c1_b == 34622.0
    assert params.k1 == 840.0
    assert params.x0 == 0.0245
    assert params.alpha_a == 12441.0
    assert params.alpha_b == 38430.0
    assert params.gamma == 136320.0
    assert params.beta == 2059020.0
    assert params.A == 58.0
    assert params.n == 2.0
    assert params.eta == 190.0


def test_monotonic_force_scaling_with_voltage():
    """Peak damper force must strictly increase with applied coil control voltage."""
    damper = ModifiedBoucWenMRDamper()
    f = 10.0
    omega = 2.0 * np.pi * f
    X0 = 0.008
    t_end = 0.5

    peak_forces = []
    voltages = [0.0, 0.5, 1.0, 1.5, 2.0]

    for v in voltages:
        res = damper.simulate_trajectory(
            t_span=(0.0, t_end),
            disp_func=lambda t: X0 * np.sin(omega * t - np.pi / 2.0),
            vel_func=lambda t: X0 * omega * np.cos(omega * t - np.pi / 2.0),
            volt_func=lambda t: v,
            dt=2e-4
        )
        # Steady state peak force
        idx = res['time'] >= (t_end - 1.0 / f)
        peak = np.max(np.abs(res['force'][idx]))
        peak_forces.append(peak)

    # Verify strictly monotonic increase
    for i in range(len(peak_forces) - 1):
        assert peak_forces[i+1] > peak_forces[i], f"Force at {voltages[i+1]}V ({peak_forces[i+1]:.1f}N) not greater than at {voltages[i]}V ({peak_forces[i]:.1f}N)"


def test_energy_dissipation_passivity():
    """
    Thermodynamic Passivity test:
    The work done by the damper over one complete steady-state cycle (hysteresis loop area)
    must be positive (energy dissipation): E_diss = oint F_MR dx > 0.
    """
    damper = ModifiedBoucWenMRDamper()
    f = 10.0
    omega = 2.0 * np.pi * f
    X0 = 0.008
    t_end = 0.6
    dt = 1e-4

    res = damper.simulate_trajectory(
        t_span=(0.0, t_end),
        disp_func=lambda t: X0 * np.sin(omega * t - np.pi / 2.0),
        vel_func=lambda t: X0 * omega * np.cos(omega * t - np.pi / 2.0),
        volt_func=lambda t: 1.0,
        dt=dt
    )

    # Extract exact last period [t_end - 1/f, t_end]
    idx = (res['time'] >= (t_end - 1.0 / f)) & (res['time'] <= t_end)
    f_cycle = res['force'][idx]
    v_cycle = res['velocity'][idx]
    t_cycle = res['time'][idx]

    # Dissipated energy per cycle: integral(F * x_dot * dt)
    energy_dissipated = np.trapezoid(f_cycle * v_cycle, t_cycle)
    assert energy_dissipated > 0.0, f"Damper must dissipate positive energy, got {energy_dissipated} J"


def test_coil_first_order_lag():
    """Verify coil dynamics follow the analytical step response u(t) = V * (1 - exp(-eta * t))."""
    damper = ModifiedBoucWenMRDamper()
    eta = damper.params.eta
    v_step = 1.5

    res = damper.simulate_trajectory(
        t_span=(0.0, 0.05),
        disp_func=lambda t: 0.0,
        vel_func=lambda t: 0.0,
        volt_func=lambda t: v_step,
        dt=1e-4
    )

    t = res['time']
    u_sim = res['u']
    u_exact = v_step * (1.0 - np.exp(-eta * t))

    max_error = np.max(np.abs(u_sim - u_exact))
    assert max_error < 1e-3, f"Coil voltage filter mismatch: max error = {max_error}"


if __name__ == '__main__':
    pytest.main([__file__, "-v"])
