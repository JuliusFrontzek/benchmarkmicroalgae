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
        # 1. Setup Dimensions first!
        self.N = 3; self.nx = 5; self.nu = 4; self.nd = 7
        
        # 2. Setup Opti Environment
        self.opti = cs.Opti()
        
        # 3. NOW you can define parameters
        self.p_u_prev = self.opti.parameter(self.nu)
        self.p_x0 = self.opti.parameter(self.nx)
        self.p_dist = self.opti.parameter(self.nd)
        self.p_refs = self.opti.parameter(3)

        # 4. Load Model
        raw_model = torch.jit.load(model_path, map_location='cpu')
        raw_model.eval()
        self.loaded_model = L4CasADiWrapper(raw_model)
        self.l4c_model = l4c.L4CasADi(self.loaded_model, device='cpu')

        # 5. Optimization variables
        self.X = self.opti.variable(self.nx, self.N + 1)
        self.U = self.opti.variable(self.nu, self.N)
        
        # 6. Objective & Constraints
        cost = 0
        self.opti.subject_to(self.X[:, 0] == self.p_x0)

        for k in range(self.N):
            # Setpoint cost
            cost += 10 * (self.X[0, k] - self.p_refs[0])**2  # pH
            cost += 0.1 * (self.X[1, k] - self.p_refs[1])**2 # DO
            cost += 5 * (self.X[4, k] - self.p_refs[2])**2   # Temp

            # Control effort cost
            cost += 0.01 * cs.sumsqr(self.U[:, k])

            # Delta U cost (Slew rate penalty)
            if k == 0:
                diff = self.U[:, k] - self.p_u_prev
            else:
                diff = self.U[:, k] - self.U[:, k-1]
            
            cost += 1.0 * cs.sumsqr(diff)
            
            # Dynamics Constraint
            input_concat = cs.vertcat(self.X[:, k], self.U[:, k], self.p_dist)
            x_next = self.l4c_model(input_concat).T
            self.opti.subject_to(self.X[:, k+1] == x_next)

        self.opti.minimize(cost)

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