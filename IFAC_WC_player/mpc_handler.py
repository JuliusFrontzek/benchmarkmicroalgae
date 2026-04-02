import torch
import torch.nn as nn
import casadi as cs
import l4casadi as l4c
import numpy as np

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
        # 1. Load Model
        raw_model = torch.jit.load(model_path, map_location='cpu')
        raw_model.eval()
        self.loaded_model = L4CasADiWrapper(raw_model)
        self.l4c_model = l4c.L4CasADi(self.loaded_model, device='cpu')

        # 2. Dimensions
        self.N = 3; self.nx = 5; self.nu = 4; self.nd = 7

        # 3. Setup Opti Environment
        self.opti = cs.Opti()
        
        # Optimization variables
        self.X = self.opti.variable(self.nx, self.N + 1)  # States
        self.U = self.opti.variable(self.nu, self.N)      # Controls
        
        # Parameters (values provided at solve time)
        self.p_x0 = self.opti.parameter(self.nx)          # Initial state
        self.p_dist = self.opti.parameter(self.nd)        # Disturbances
        self.p_refs = self.opti.parameter(3)             # [pH, DO, Temp]

        # 4. Objective & Constraints
        cost = 0
        self.opti.subject_to(self.X[:, 0] == self.p_x0)   # Initial condition

        for k in range(self.N):
            # Cost function: State tracking + Control effort
            cost += 10 * (self.X[0, k] - self.p_refs[0])**2  # pH
            cost += 0.1 * (self.X[1, k] - self.p_refs[1])**2 # DO
            cost += 5 * (self.X[4, k] - self.p_refs[2])**2   # Temp
            cost += 0.01 * cs.sumsqr(self.U[:, k])           # Regularization
            
            # Dynamics Constraint
            # L4CasADi works seamlessly within the Opti loop
            input_concat = cs.vertcat(self.X[:, k], self.U[:, k], self.p_dist)
            x_next = self.l4c_model(input_concat).T
            self.opti.subject_to(self.X[:, k+1] == x_next)

        self.opti.minimize(cost)

        # 5. Solver Setup
        opts = {
            'ipopt': {
                'print_level': 0,
                'hessian_approximation': 'limited-memory',
                'max_iter': 100 
            }
        }
        self.opti.solver('ipopt', opts)

    def solve(self, x_curr, d_curr, refs):
        # Set parameter values
        self.opti.set_value(self.p_x0, x_curr)
        self.opti.set_value(self.p_dist, d_curr)
        self.opti.set_value(self.p_refs, refs)
        
        try:
            sol = self.opti.solve()
            return sol.value(self.U[:, 0]).tolist()
        except RuntimeError:
            # Fallback if solver fails to converge
            return self.opti.debug.value(self.U[:, 0]).tolist()

# Global instance to persist across MATLAB calls
_solver_instance = None

def get_mpc_action(x, d, r):
    global _solver_instance
    if _solver_instance is None:
        _solver_instance = MicroalgaeMPC()
    return _solver_instance.solve(np.array(x), np.array(d), np.array(r))