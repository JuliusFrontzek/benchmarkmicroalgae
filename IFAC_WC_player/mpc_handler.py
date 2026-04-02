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


# Scaling Constants
X_MEAN = [7.9163324339067564, 120.71290085378622, 0.14845685279384763, 0.4793606812697203, 27.88878210112914]
X_STD = [0.4744382035392219, 30.026734704243562, 0.004497946395715579, 0.05335923784587826, 3.8031253028626337]

U_MEAN = [0.0001388407717334788, 0.0031480429203711463, 0.0004426091468117169, 33.93987094388777]
U_STD = [0.0001233936408617958, 0.0029700696386538274, 0.00022966608129498743, 15.163315708503845]

D_MEAN = [267.46088324652777, 561.0259486975558, 26.01501011075797, 57.046535056498314, 2.214619182198748, 0.034915123456790126, 0.03125]
D_STD = [306.6240325161852, 643.1745706067707, 4.447301442396769, 21.617538471642067, 1.6714980397152348, 0.18356485941099596, 0.17399263633843817]

# --- MPC Class ---
class MicroalgaeMPC:
    def __init__(self, model_path="dynamic_model_l4casadi.pt"):
        # 1. Dimensions and Opti setup
        self.N = 12; self.nx = 5; self.nu = 4; self.nd = 7
        self.opti = cs.Opti()
        
        # Load L4CasADi Model
        # The NN input is expected to be [nx + nu + nd] = 16 elements
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        nn_model = torch.load(
            model_path, 
            map_location=device, 
            weights_only=False
        )
        self.l4c_model = l4c.L4CasADi(L4CasADiWrapper(nn_model))

        # 2. Parameters
        self.p_u_prev = self.opti.parameter(self.nu)
        self.p_x0 = self.opti.parameter(self.nx)
        # FIX: Disturbance is now a matrix (nd rows, N columns)
        self.p_dist = self.opti.parameter(self.nd, self.N)
        self.p_refs = self.opti.parameter(3) # [pH, DO, Temp]

        # 3. Decision Variables
        self.X = self.opti.variable(self.nx, self.N + 1)
        self.U = self.opti.variable(self.nu, self.N)

        # 4. Dynamics Constraints (Crucial for utilizing p_dist trajectory)
        self.opti.subject_to(self.X[:, 0] == self.p_x0)
        for k in range(self.N):
            # 1. Scale inputs for the NN
            x_scaled = (self.X[:, k] - X_MEAN) / X_STD
            u_scaled = (self.U[:, k] - U_MEAN) / U_STD
            d_scaled = (self.p_dist[:, k] - D_MEAN) / D_STD
            
            # 2. Call NN to get Delta (Scaled)
            inputs = cs.vertcat(x_scaled, u_scaled, d_scaled)
            delta_scaled = self.l4c_model(inputs).T
            
            # 3. Unscale Delta: delta_raw = delta_scaled * sigma_x
            delta_raw = delta_scaled * X_STD
            
            # 4. Residual connection: x_next = x_curr + delta_raw
            self.opti.subject_to(self.X[:, k+1] == self.X[:, k] + delta_raw)

        # 5. Normalization and Weights
        norm = {
            'CO2_max': 20/1000/60, 'CO2_min': 0.0,
            'air_max': 200/1000/60, 'air_min': 0.0,
            'Qw_max': 6e-4, 'Qw_min': 0.0,
            'Tin_max': 80.0, 'Tin_min': 0.0
        }
        w = {
            'pH': {'sp': 1196, 's': 2.6, 'c': 0.001},
            'DO': {'sp': 34, 's': 0.1, 'c': 0.00001},
            'T':  {'sp': 183, 's1': 3.0, 's2': 8.462, 'c': 0.0001}
        }

        # 6. Build Objective
        jsp_pH = relative_absolute_error(self.p_refs[0], self.X[0, 1:])
        jsp_DO = relative_absolute_error(self.p_refs[1], self.X[1, 1:])
        jsp_T  = relative_absolute_error(self.p_refs[2], self.X[4, 1:])

        js_pH = smoothness_cost(self.U[0, :], self.p_u_prev[0], norm['CO2_min'], norm['CO2_max'])
        js_DO = smoothness_cost(self.U[1, :], self.p_u_prev[1], norm['air_min'], norm['air_max'])
        js_qw_T = smoothness_cost(self.U[2, :], self.p_u_prev[2], norm['Qw_min'], norm['Qw_max'])
        js_tin_T = smoothness_cost(self.U[3, :], self.p_u_prev[3], norm['Tin_min'], norm['Tin_max'])

        jc_pH = consumption_cost(self.U[0, :], norm['CO2_max'])
        jc_DO = consumption_cost(self.U[1, :], norm['air_max'])
        jc_T  = consumption_cost(self.U[2, :], norm['Qw_max'])

        self.opti.minimize(
            w['pH']['sp']*jsp_pH + w['pH']['s']*js_pH + w['pH']['c']*jc_pH +
            w['DO']['sp']*jsp_DO + w['DO']['s']*js_DO + w['DO']['c']*jc_DO +
            w['T']['sp']*jsp_T + w['T']['s1']*js_qw_T + w['T']['s2']*js_tin_T + w['T']['c']*jc_T
        )

        # 7. Solver Setup
        self.opti.solver('ipopt', {'ipopt': {'print_level': 0, 'max_iter': 50, 'tol': 1e-3}})

    def solve(self, x_curr, d_matrix, refs, u_prev):
        self.opti.set_value(self.p_x0, x_curr)
        self.opti.set_value(self.p_dist, d_matrix)
        self.opti.set_value(self.p_refs, refs)
        self.opti.set_value(self.p_u_prev, u_prev)

        # Warm start logic
        if hasattr(self, 'last_sol'):
            self.opti.set_initial(self.U, self.last_sol.value(self.U))
            self.opti.set_initial(self.X, self.last_sol.value(self.X))
        
        try:
            sol = self.opti.solve()
            self.last_sol = sol
            return sol.value(self.U[:, 0]).tolist()
        except:
            return self.opti.debug.value(self.U[:, 0]).tolist()

# --- Singleton Logic ---
_solver_instance = None
_last_u = [0.0] * 4

def get_mpc_action(x, d_matrix, r, u_prev_ext):
    global _solver_instance, _last_u
    if _solver_instance is None:
        _solver_instance = MicroalgaeMPC()
    
    # We use the previous control signal to calculate smoothness cost
    action = _solver_instance.solve(np.array(x), np.array(d_matrix), np.array(r), np.array(u_prev_ext))
    _last_u = action 
    return action