import torch
import torch.nn as nn
import casadi as cs
import l4casadi as l4c
import numpy as np

def relative_absolute_error(ref, y, eps=1e-12):
    """Sum of relative absolute tracking errors: sum(|ref - y| / (|ref| + eps))."""
    # Use sum2 for horizontal row vectors
    return cs.sum2(cs.fabs(ref - y) / (cs.fabs(ref) + eps))

def smoothness_cost(u, u_prev, u_min, u_max, eps=1e-12):
    """Normalized squared input jumps: sum(((u_k - u_k-1) / range)^2)."""
    u_range = (u_max - u_min) + eps
    # First step uses u_prev from the plant
    cost = ((u[0] - u_prev) / u_range)**2
    # Subsequent steps in the horizon
    for i in range(1, u.shape[1]):
        cost += ((u[i] - u[i-1]) / u_range)**2
    return cost

def consumption_cost(u, u_max, eps=1e-12):
    """Normalized cumulative consumption: sum(u / u_max)."""
    return cs.sum2(u) / (u_max + eps)

class L4CasADiWrapper(nn.Module):
    def __init__(self, model):
        super().__init__()
        self.model = model
    def forward(self, x):
        if x.ndim == 1: x = x.unsqueeze(0)
        elif x.ndim == 2 and x.shape[1] == 1:
            x = x.t()
        return self.model(x)

class MicroalgaeMPC:
    def __init__(self, model_path="dynamic_model_l4casadi.pt"):
        # 1. Dimensions and Opti setup
        self.N = 3; self.nx = 5; self.nu = 4; self.nd = 7
        self.opti = cs.Opti()
        
        # 2. Parameters and Variables
        self.p_u_prev = self.opti.parameter(self.nu)
        self.p_x0 = self.opti.parameter(self.nx)
        self.p_dist = self.opti.parameter(self.nd)
        self.p_refs = self.opti.parameter(3) # [pH, DO, Temp]

        self.X = self.opti.variable(self.nx, self.N + 1)
        self.U = self.opti.variable(self.nu, self.N)

        # 3. Benchmark Normalization
        norm = {
            'CO2_max': 20 / 1000 / 60, 'CO2_min': 0.0,
            'air_max': 200 / 1000 / 60, 'air_min': 0.0,
            'Qw_max': 6e-4,             'Qw_min': 0.0,
            'Tin_max': 80.0,            'Tin_min': 0.0
        }

        # 4. Benchmark Weights
        w = {
            'pH': {'sp': 0.00119647, 's': 0.00261993, 'c': 0.0000162036},
            'DO': {'sp': 0.0000338547, 's': 0.000144217, 'c': 0.0000000366847},
            'T':  {'sp': 0.000183301, 's1': 0.00305396, 's2': 0.00846202, 'c': 0.00000525393}
        }

        # 5. Build Objective Components
        # Tracking (JSP)
        jsp_pH = relative_absolute_error(self.p_refs[0], self.X[0, 1:])
        jsp_DO = relative_absolute_error(self.p_refs[1], self.X[1, 1:])
        jsp_T  = relative_absolute_error(self.p_refs[2], self.X[4, 1:])

        # Smoothness (JS)
        js_pH    = smoothness_cost(self.U[0, :], self.p_u_prev[0], norm['CO2_min'], norm['CO2_max'])
        js_DO    = smoothness_cost(self.U[1, :], self.p_u_prev[1], norm['air_min'], norm['air_max'])
        js_qw_T  = smoothness_cost(self.U[2, :], self.p_u_prev[2], norm['Qw_min'], norm['Qw_max'])
        js_tin_T = smoothness_cost(self.U[3, :], self.p_u_prev[3], norm['Tin_min'], norm['Tin_max'])

        # Consumption (JC)
        jc_pH = consumption_cost(self.U[0, :], norm['CO2_max'])
        jc_DO = consumption_cost(self.U[1, :], norm['air_max'])
        jc_T  = consumption_cost(self.U[2, :], norm['Qw_max'])

        # 6. Total Cost
        j_pH = w['pH']['sp']*jsp_pH + w['pH']['s']*js_pH + w['pH']['c']*jc_pH
        j_DO = w['DO']['sp']*jsp_DO + w['DO']['s']*js_DO + w['DO']['c']*jc_DO
        j_T  = w['T']['sp']*jsp_T + w['T']['s1']*js_qw_T + w['T']['s2']*js_tin_T + w['T']['c']*jc_T

        self.opti.minimize(j_pH + j_DO + j_T)

        # 7. Solver Setup
        opts = {
            'ipopt': {
                'print_level': 0,
                'max_iter': 50,
                'tol': 1e-3,
                'acceptable_tol': 1e-2,
                'acceptable_iter': 5,
                'hessian_approximation': 'limited-memory',
            }
        }
        self.opti.solver('ipopt', opts)

    def solve(self, x_curr, d_curr, refs, u_prev):
        self.opti.set_value(self.p_x0, x_curr)
        self.opti.set_value(self.p_dist, d_curr)
        self.opti.set_value(self.p_refs, refs)
        self.opti.set_value(self.p_u_prev, u_prev)

        if hasattr(self, 'last_sol'):
            old_u = self.last_sol.value(self.U)
            new_u_guess = np.hstack([old_u[:, 1:], old_u[:, -1:]])
            self.opti.set_initial(self.U, new_u_guess)
            
            old_x = self.last_sol.value(self.X)
            new_x_guess = np.hstack([old_x[:, 1:], old_x[:, -1:]])
            self.opti.set_initial(self.X, new_x_guess)
        
        try:
            sol = self.opti.solve()
            self.last_sol = sol
            return sol.value(self.U[:, 0]).tolist()
        except RuntimeError:
            return self.opti.debug.value(self.U[:, 0]).tolist()

# --- Wrapper Logic ---
_solver_instance = None
_last_u = [0.0] * 4 # Keeps track of history across MATLAB calls

def get_mpc_action(x, d, r):
    global _solver_instance, _last_u
    if _solver_instance is None:
        _solver_instance = MicroalgaeMPC()
    
    # Solve and update our global 'last_u' so the penalty works next time!
    action = _solver_instance.solve(np.array(x), np.array(d), np.array(r), np.array(_last_u))
    _last_u = action 
    return action