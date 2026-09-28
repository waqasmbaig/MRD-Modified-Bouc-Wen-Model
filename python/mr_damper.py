"""
Modified Bouc-Wen Phenomenological Model for Magnetorheological (MR) Dampers.

Reference:
    Spencer, B. F., Dyke, S. J., Sain, M. K., & Carlson, J. D. (1997).
    "Phenomenological Model for Magnetorheological Dampers."
    Journal of Engineering Mechanics, ASCE, 123(3), 230-238.
    https://doi.org/10.1061/(ASCE)0733-9399(1997)123:3(230)

Author: Open-Source Vehicle Dynamics & Smart Structures Reference
All internal calculations adhere strictly to SI units (m, s, N, V, rad).
"""

from dataclasses import dataclass
from typing import Tuple, Optional, Callable
import numpy as np
from scipy.integrate import solve_ivp


@dataclass
class MRDamperParameters:
    """
    Physical parameter container for Spencer's Modified Bouc-Wen MR Damper Model.
    All values are in SI base units.
    """
    # Viscous damping parameters
    c0_a: float = 784.0        # Zero-voltage damping [N*s/m]
    c0_b: float = 1803.0       # Field-dependent damping coefficient [N*s/(m*V)]
    
    # Dashpot stiffness
    k0: float = 3610.0         # Post-yield stiffness [N/m]
    
    # Gas accumulator parameters
    c1_a: float = 14649.0      # Gas accumulator damping [N*s/m]
    c1_b: float = 34622.0      # Field-dependent accumulator damping [N*s/(m*V)]
    k1: float = 840.0          # Gas accumulator stiffness [N/m]
    x0: float = 0.0245         # Initial accumulator displacement offset [m]
    
    # Bouc-Wen hysteresis parameters
    alpha_a: float = 12441.0   # Base hysteretic force coefficient [N/m]
    alpha_b: float = 38430.0   # Field-dependent hysteretic force gain [N/(m*V)]
    gamma: float = 136320.0    # Hysteresis shape parameter [m^-2]
    beta: float = 2059020.0    # Hysteresis shape parameter [m^-2]
    A: float = 58.0            # Hysteresis amplitude scale factor [-]
    n: float = 2.0             # Smoothness exponent of yield transition [-]
    
    # Electromagnetic coil dynamics
    eta: float = 190.0         # Coil first-order time response rate [s^-1] (tau = 1/eta ~ 5.26 ms)


class ModifiedBoucWenMRDamper:
    """
    Implementation of the phenomenological Spencer Modified Bouc-Wen MR Damper.

    Internal State Vector:
        state = [y, z, u]
            y: Internal damper displacement [m]
            z: Evolutionary hysteretic variable [m]
            u: Effective filtered control voltage [V]
    """

    def __init__(self, params: Optional[MRDamperParameters] = None):
        self.params = params if params is not None else MRDamperParameters()
        self.state = np.zeros(3, dtype=np.float64)  # [y, z, u]

    def reset(self, initial_state: Optional[np.ndarray] = None) -> None:
        """Reset internal states to initial values (default all zeros)."""
        if initial_state is not None:
            self.state = np.array(initial_state, dtype=np.float64)
        else:
            self.state = np.zeros(3, dtype=np.float64)

    def compute_derivatives(
        self,
        t: float,
        state: np.ndarray,
        x: float,
        x_dot: float,
        v_cmd: float
    ) -> Tuple[np.ndarray, float]:
        """
        Compute the state derivatives and instantaneous damping force.

        Args:
            t: Current time [s]
            state: Array of [y, z, u]
            x: Piston displacement [m]
            x_dot: Piston velocity [m/s]
            v_cmd: Commanded coil input voltage [V]

        Returns:
            derivs: np.ndarray of [dy/dt, dz/dt, du/dt]
            F_MR: Total damping force [N]
        """
        p = self.params
        y, z, u = state[0], state[1], state[2]

        # 1. First-order voltage dynamic lag: du/dt = -eta * (u - V_cmd)
        u_dot = -p.eta * (u - v_cmd)

        # 2. Voltage-dependent parameters
        alpha = p.alpha_a + p.alpha_b * u
        c0 = p.c0_a + p.c0_b * u
        c1 = p.c1_a + p.c1_b * u

        # 3. Intermediate node velocity (dy/dt) from force equilibrium:
        #    (c0 + c1) * y_dot = alpha * z + c0 * x_dot + k0 * (x - y)
        y_dot = (alpha * z + c0 * x_dot + p.k0 * (x - y)) / (c0 + c1)

        # 4. Hysteretic evolutionary rate (dz/dt):
        vel_rel = x_dot - y_dot
        abs_z = abs(z)
        z_n = abs_z ** p.n
        z_n_minus_1 = abs_z ** (p.n - 1.0) if abs_z > 1e-18 else 0.0

        z_dot = (
            p.A * vel_rel
            - p.beta * vel_rel * z_n
            - p.gamma * abs(vel_rel) * z_n_minus_1 * z
        )

        # 5. Total output damping force:
        #    F_MR = c1 * y_dot + k1 * (x - x0)
        f_mr = c1 * y_dot + p.k1 * (x - p.x0)

        derivs = np.array([y_dot, z_dot, u_dot], dtype=np.float64)
        return derivs, f_mr

    def simulate_trajectory(
        self,
        t_span: Tuple[float, float],
        disp_func: Callable[[float], float],
        vel_func: Callable[[float], float],
        volt_func: Callable[[float], float],
        dt: float = 1e-4,
        initial_state: Optional[np.ndarray] = None
    ) -> dict:
        """
        Simulate the MR damper response over a continuous time interval.

        Args:
            t_span: (t_start, t_end) in seconds
            disp_func: Function x(t) returning displacement [m]
            vel_func: Function x_dot(t) returning velocity [m/s]
            volt_func: Function V(t) returning voltage [V]
            dt: Maximum integration time step [s]
            initial_state: Initial [y0, z0, u0]

        Returns:
            Dictionary containing time, displacement, velocity, force, and states.
        """
        if initial_state is None:
            initial_state = np.zeros(3, dtype=np.float64)

        num_points = max(2, int(np.round((t_span[1] - t_span[0]) / dt)) + 1)
        t_eval = np.linspace(t_span[0], t_span[1], num_points)

        def ode_func(t, state):
            x = disp_func(t)
            x_dot = vel_func(t)
            v = volt_func(t)
            derivs, _ = self.compute_derivatives(t, state, x, x_dot, v)
            return derivs

        # Radau implicit solver ensures stiff integration stability
        sol = solve_ivp(
            ode_func,
            t_span,
            initial_state,
            t_eval=t_eval,
            method='Radau',
            rtol=1e-6,
            atol=1e-9
        )

        y = sol.y[0]
        z = sol.y[1]
        u = sol.y[2]
        t = sol.t

        x = np.array([disp_func(ti) for ti in t])
        x_dot = np.array([vel_func(ti) for ti in t])

        # Compute output force array
        c0 = self.params.c0_a + self.params.c0_b * u
        c1 = self.params.c1_a + self.params.c1_b * u
        alpha = self.params.alpha_a + self.params.alpha_b * u
        y_dot = (alpha * z + c0 * x_dot + self.params.k0 * (x - y)) / (c0 + c1)
        f_mr = c1 * y_dot + self.params.k1 * (x - self.params.x0)

        return {
            'time': t,
            'displacement': x,
            'velocity': x_dot,
            'force': f_mr,
            'y': y,
            'z': z,
            'u': u,
            'y_dot': y_dot
        }
