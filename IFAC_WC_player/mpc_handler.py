import torch
import torch.nn as nn
import casadi as cs
import l4casadi as l4c
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

# Module-level state history buffer (filled by get_mpc_action each call)
_state_history = []
_MAX_HISTORY = 60

# Cached raw PyTorch model for numpy rollouts (no CasADi dependency)
_raw_nn_model = None


def _load_raw_model(model_path="dynamic_model_l4casadi.pt"):
    global _raw_nn_model
    if _raw_nn_model is None:
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        _raw_nn_model = torch.load(model_path, map_location=device, weights_only=False)
        _raw_nn_model.eval()
    return _raw_nn_model


def _nn_rollout_numpy(nn_model, x0, u_seq, d_matrix):
    """Open-loop N-step rollout using raw PyTorch (no CasADi).

    Args:
        nn_model: raw torch.nn.Module
        x0: (5,) current state
        u_seq: (4, N) control inputs per step
        d_matrix: (7, N) disturbances per step

    Returns:
        X_pred: (5, N+1) state trajectory including x0
    """
    X_mean = np.array(X_MEAN)
    X_std  = np.array(X_STD)
    U_mean = np.array(U_MEAN)
    U_std  = np.array(U_STD)
    D_mean = np.array(D_MEAN)
    D_std  = np.array(D_STD)

    N = u_seq.shape[1]
    X_pred = np.zeros((5, N + 1))
    X_pred[:, 0] = x0.copy()

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    with torch.no_grad():
        x = x0.copy().astype(np.float32)
        for k in range(N):
            x_scaled = (x - X_mean) / X_std
            u_scaled = (u_seq[:, k] - U_mean) / U_std
            d_scaled = (d_matrix[:, k] - D_mean) / D_std
            inp = np.concatenate([x_scaled, u_scaled, d_scaled]).astype(np.float32)
            inp_t = torch.tensor(inp).unsqueeze(0).to(device)
            delta_scaled = nn_model(inp_t).cpu().numpy()[0]
            delta_raw = delta_scaled * X_std
            x = x + delta_raw
            X_pred[:, k + 1] = x
    return X_pred


def plot_nn_predictions(x_history, x_curr, u_fixed, d_matrix,
                        refs=None, save_path="nn_prediction_debug.png",
                        model_path="dynamic_model_l4casadi.pt"):
    """Diagnostic plot: past observed states + NN open-loop future predictions.

    Args:
        x_history: (T_hist, 5) past states, or [] / None if no history
        x_curr: (5,) current state
        u_fixed: (4,) control input held constant across all N prediction steps
        d_matrix: (7, N) disturbance forecast
        refs: optional [pH_ref, DO_ref, T_ref] for setpoint lines
        save_path: where to save the figure
        model_path: path to the NN weights file
    """
    x_history = np.array(x_history) if (x_history is not None and len(x_history) > 0) else np.empty((0, 5))
    x_curr    = np.array(x_curr, dtype=float)
    u_fixed   = np.array(u_fixed, dtype=float)
    d_matrix  = np.array(d_matrix, dtype=float)
    N = d_matrix.shape[1]

    nn_model = _load_raw_model(model_path)
    u_seq = np.tile(u_fixed[:, None], (1, N))
    X_pred = _nn_rollout_numpy(nn_model, x_curr, u_seq, d_matrix)

    dt = 5  # minutes per step
    t_future = np.arange(0, (N + 1) * dt, dt)

    has_hist = x_history.shape[0] > 0
    if has_hist:
        T_hist = x_history.shape[0]
        t_hist = np.arange(-T_hist * dt, 0, dt)
        # full past + current concatenated for a continuous line up to t=0
        x_past_full = np.vstack([x_history, x_curr])
        t_past_full  = np.append(t_hist, 0)
    else:
        t_past_full = np.array([0])
        x_past_full = x_curr[None, :]

    state_labels = ['pH', 'DO (%)', 'Depth (m)', 'Xalg (g/L)', 'Temperature (°C)']
    sp_indices   = {0: refs[0] if refs else None,
                    1: refs[1] if refs else None,
                    4: refs[2] if refs else None}

    fig, axes = plt.subplots(5, 1, figsize=(10, 14), sharex=True)
    fig.suptitle('NN Model: Past Observations vs. Predicted Future Trajectory', fontsize=13)

    for i, ax in enumerate(axes):
        ax.plot(t_past_full, x_past_full[:, i], color='steelblue', lw=1.8, label='Observed')
        ax.plot(t_future, X_pred[i, :], color='tomato', lw=1.8, linestyle='--', label='NN prediction')
        ax.axvline(0, color='black', lw=0.8, linestyle=':')
        sp = sp_indices.get(i)
        if sp is not None:
            ax.axhline(sp, color='grey', lw=1.2, linestyle=':', label=f'Setpoint ({sp})')
        ax.set_ylabel(state_labels[i])
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time relative to now (min)')
    plt.tight_layout()
    plt.savefig(save_path, dpi=120)
    plt.close(fig)
    print(f"[NN Diagnostic] Prediction plot saved to: {os.path.abspath(save_path)}")
    return os.path.abspath(save_path)


def relative_absolute_error(ref, y, eps=1e-12):
    return cs.sum2(cs.fabs(ref - y) / (cs.fabs(ref) + eps))

def smoothness_cost(u, u_prev, u_min, u_max, eps=1e-12):
    u_range = (u_max - u_min) + eps
    cost = ((u[0] - u_prev) / u_range)**2
    for i in range(1, u.shape[1]):
        cost += ((u[i] - u[i-1]) / u_range)**2
    return cost

def consumption_cost(u, u_max, eps=1e-12):
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


# Scaling Constants (derived from training data)
X_MEAN = [7.9163324339067564, 120.71290085378622, 0.14845685279384763, 0.4793606812697203, 27.88878210112914]
X_STD = [0.4744382035392219, 30.026734704243562, 0.004497946395715579, 0.05335923784587826, 3.8031253028626337]

U_MEAN = [0.0001388407717334788, 0.0031480429203711463, 0.0004426091468117169, 33.93987094388777]
U_STD = [0.0001233936408617958, 0.0029700696386538274, 0.00022966608129498743, 15.163315708503845]

D_MEAN = [267.46088324652777, 561.0259486975558, 26.01501011075797, 57.046535056498314, 2.214619182198748, 0.034915123456790126, 0.03125]
D_STD = [306.6240325161852, 643.1745706067707, 4.447301442396769, 21.617538471642067, 1.6714980397152348, 0.18356485941099596, 0.17399263633843817]

# Actuator physical limits
U_LB = [0.0,          0.0,           0.0,          0.0]
U_UB = [20/1000/60,   500/1000/60,   36/1000/60,   80.0]

# Constant heat-exchanger flux used by the temperature-only MPC. Fixing Qhx and
# optimising only Tin removes the bilinear Qhx*(Tin-T) degeneracy. Max flow gives
# the most heat-exchange authority and sits ~0.7 std above the training mean
# (U_MEAN[2]=4.4e-4, U_STD[2]=2.3e-4), so it stays in-distribution for the NN.
QHX_CONST = U_UB[2]

# Prediction horizon for the simple temperature MPC, in 5-minute steps. Single
# source of truth: the MATLAB controller fetches it (so the forecast matrix width
# always matches) and SimpleTempMPC sizes its NLP from it. Change it here only.
# N=6 -> 30 min lookahead.
SIMPLE_TEMP_MPC_N = 6

# Prediction horizon for the simple pH MPC, in 5-minute steps. Same single-
# source-of-truth pattern as SIMPLE_TEMP_MPC_N: the MATLAB controller fetches it
# so the forecast matrix width always matches the NLP. N=6 -> 30 min lookahead.
SIMPLE_PH_MPC_N = 6


# --- Full MPC Class ---
class MicroalgaeMPC:
    def __init__(self, model_path="dynamic_model_l4casadi.pt"):
        self.N = 12
        self.nx = 5
        self.nu = 4
        self.nd = 7
        
        self.opti = cs.Opti()
        self._fail_count = 0

        nn_model = torch.load(model_path, map_location='cpu', weights_only=False)
        self.l4c_model = l4c.L4CasADi(L4CasADiWrapper(nn_model))

        # Parameters
        self.p_u_prev = self.opti.parameter(self.nu)
        self.p_x0 = self.opti.parameter(self.nx)
        self.p_dist = self.opti.parameter(self.nd, self.N)
        self.p_refs = self.opti.parameter(3)  # [pH, DO, Temp]

        # Decision Variables
        self.X = self.opti.variable(self.nx, self.N + 1)
        self.U = self.opti.variable(self.nu, self.N)

        # Input constraints (physical actuator limits)
        for i in range(self.nu):
            self.opti.subject_to(self.U[i, :] >= U_LB[i])
            self.opti.subject_to(self.U[i, :] <= U_UB[i])

        # Dynamics constraints
        self.opti.subject_to(self.X[:, 0] == self.p_x0)
        for k in range(self.N):
            x_scaled = (self.X[:, k] - X_MEAN) / X_STD
            u_scaled = (self.U[:, k] - U_MEAN) / U_STD
            d_scaled = (self.p_dist[:, k] - D_MEAN) / D_STD
            inputs = cs.vertcat(x_scaled, u_scaled, d_scaled)
            delta_scaled = self.l4c_model(inputs).T
            delta_raw = delta_scaled * X_STD
            self.opti.subject_to(self.X[:, k+1] == self.X[:, k] + delta_raw)

        # Cost weights and normalization
        norm = {
            'CO2_max': U_UB[0], 'CO2_min': U_LB[0],
            'air_max': U_UB[1], 'air_min': U_LB[1],
            'Qw_max':  U_UB[2], 'Qw_min':  U_LB[2],
            'Tin_max': U_UB[3], 'Tin_min': U_LB[3],
        }
        w = {
            'pH': {'sp': 1196, 's': 2.6,  'c': 0.001},
            'DO': {'sp': 34,   's': 0.1,  'c': 0.00001},
            'T':  {'sp': 183,  's1': 3.0, 's2': 8.462, 'c': 0.0001}
        }

        # Objective
        jsp_pH = relative_absolute_error(self.p_refs[0], self.X[0, 1:])
        jsp_DO = relative_absolute_error(self.p_refs[1], self.X[1, 1:])
        jsp_T  = relative_absolute_error(self.p_refs[2], self.X[4, 1:])

        js_pH    = smoothness_cost(self.U[0, :], self.p_u_prev[0], norm['CO2_min'], norm['CO2_max'])
        js_DO    = smoothness_cost(self.U[1, :], self.p_u_prev[1], norm['air_min'], norm['air_max'])
        js_qw_T  = smoothness_cost(self.U[2, :], self.p_u_prev[2], norm['Qw_min'],  norm['Qw_max'])
        js_tin_T = smoothness_cost(self.U[3, :], self.p_u_prev[3], norm['Tin_min'], norm['Tin_max'])

        jc_pH = consumption_cost(self.U[0, :], norm['CO2_max'])
        jc_DO = consumption_cost(self.U[1, :], norm['air_max'])
        jc_T  = consumption_cost(self.U[2, :], norm['Qw_max'])

        self.opti.minimize(
            w['pH']['sp']*jsp_pH + w['pH']['s']*js_pH + w['pH']['c']*jc_pH +
            w['DO']['sp']*jsp_DO + w['DO']['s']*js_DO + w['DO']['c']*jc_DO +
            w['T']['sp']*jsp_T + w['T']['s1']*js_qw_T + w['T']['s2']*js_tin_T + w['T']['c']*jc_T
        )

        self.opti.solver('ipopt', {
            'ipopt': {
                'print_level': 0,
                'max_iter': 500,
                'tol': 1e-2,
                'hessian_approximation': 'limited-memory',
                'mu_strategy': 'adaptive',
            }
        })

    def solve(self, x_curr, d_matrix, refs, u_prev):
        key = (tuple(np.round(x_curr, 6)), tuple(np.round(refs, 6)))
        if hasattr(self, '_cache') and self._cache.get('key') == key:
            return self._cache['result']

        self.opti.set_value(self.p_x0, x_curr)
        self.opti.set_value(self.p_dist, d_matrix)
        self.opti.set_value(self.p_refs, refs)
        self.opti.set_value(self.p_u_prev, u_prev)

        if hasattr(self, 'last_sol'):
            prev_U = self.last_sol.value(self.U)
            prev_X = self.last_sol.value(self.X)
            self.opti.set_initial(self.U, np.hstack([prev_U[:, 1:], prev_U[:, -1:]]))
            self.opti.set_initial(self.X, np.hstack([prev_X[:, 1:], prev_X[:, -1:]]))

        try:
            sol = self.opti.solve()
            self.last_sol = sol
            result = sol.value(self.U[:, 0]).tolist()
            self._cache = {'key': key, 'result': result}
            return result
        except Exception as e:
            self._fail_count += 1
            print(f"[MPC] Solver failed (total failures: {self._fail_count}): {e}")
            if hasattr(self, 'last_sol'):
                return self.last_sol.value(self.U[:, 0]).tolist()
            return list(u_prev)


# --- Temperature-Only MPC (diagnostic) ---
class TemperatureOnlyMPC:
    """
    Controls only Qhx (u[2]) and Tin_hx (u[3]) to track a temperature setpoint.
    CO2 and air are treated as fixed zeros so the NN sees a consistent input.
    Useful for verifying that the NN temperature model is working correctly.
    """
    def __init__(self, model_path="dynamic_model_l4casadi.pt"):
        self.N = 12; self.nx = 5; self.nu = 4; self.nd = 7
        self.opti = cs.Opti()
        self._fail_count = 0

        nn_model = torch.load(model_path, map_location='cpu', weights_only=False)
        self.l4c_model = l4c.L4CasADi(L4CasADiWrapper(nn_model))

        # Parameters
        self.p_u_co2  = self.opti.parameter()        # fixed CO2 (from PI controller)
        self.p_u_air  = self.opti.parameter()        # fixed air (from PI controller)
        self.p_u_prev_qhx = self.opti.parameter()   # previous Qhx for smoothness
        self.p_u_prev_tin = self.opti.parameter()   # previous Tin for smoothness
        self.p_x0 = self.opti.parameter(self.nx)
        self.p_dist = self.opti.parameter(self.nd, self.N)
        self.p_T_ref = self.opti.parameter()  # temperature setpoint [°C]

        # Decision Variables: only Qhx and Tin
        self.Qhx = self.opti.variable(1, self.N)
        self.Tin = self.opti.variable(1, self.N)
        self.X = self.opti.variable(self.nx, self.N + 1)

        # Input constraints
        self.opti.subject_to(self.Qhx >= U_LB[2])
        self.opti.subject_to(self.Qhx <= U_UB[2])
        self.opti.subject_to(self.Tin >= U_LB[3])
        self.opti.subject_to(self.Tin <= U_UB[3])

        # Dynamics constraints — CO2 and air are fixed parameters, not optimised
        self.opti.subject_to(self.X[:, 0] == self.p_x0)
        for k in range(self.N):
            # Assemble full input vector with fixed CO2/air
            u_k = cs.vertcat(self.p_u_co2, self.p_u_air, self.Qhx[0, k], self.Tin[0, k])
            x_scaled = (self.X[:, k] - X_MEAN) / X_STD
            u_scaled = (u_k - U_MEAN) / U_STD
            d_scaled = (self.p_dist[:, k] - D_MEAN) / D_STD
            inputs = cs.vertcat(x_scaled, u_scaled, d_scaled)
            delta_scaled = self.l4c_model(inputs).T
            delta_raw = delta_scaled * X_STD
            self.opti.subject_to(self.X[:, k+1] == self.X[:, k] + delta_raw)

        # Objective: temperature tracking + smoothness
        w_sp = 500.0   # setpoint tracking weight
        w_s1 = 3.0     # Qhx smoothness
        w_s2 = 8.462   # Tin smoothness

        jsp_T = cs.sum2(cs.fabs(self.p_T_ref - self.X[4, 1:]))

        qhx_range = U_UB[2] - U_LB[2]
        tin_range = U_UB[3] - U_LB[3]
        js_qhx = ((self.Qhx[0, 0] - self.p_u_prev_qhx) / qhx_range)**2
        js_tin = ((self.Tin[0, 0] - self.p_u_prev_tin) / tin_range)**2
        for i in range(1, self.N):
            js_qhx += ((self.Qhx[0, i] - self.Qhx[0, i-1]) / qhx_range)**2
            js_tin  += ((self.Tin[0, i] - self.Tin[0, i-1])  / tin_range)**2

        self.opti.minimize(w_sp * jsp_T + w_s1 * js_qhx + w_s2 * js_tin)

        self.opti.solver('ipopt', {
            'ipopt': {
                'print_level': 0,
                'max_iter': 500,
                'tol': 1e-2,
                'hessian_approximation': 'limited-memory',
                'mu_strategy': 'adaptive',
            }
        })

    def solve(self, x_curr, d_matrix, T_ref, u_co2, u_air, u_prev_qhx, u_prev_tin):
        self.opti.set_value(self.p_x0, x_curr)
        self.opti.set_value(self.p_dist, d_matrix)
        self.opti.set_value(self.p_T_ref, T_ref)
        self.opti.set_value(self.p_u_co2, u_co2)
        self.opti.set_value(self.p_u_air, u_air)
        self.opti.set_value(self.p_u_prev_qhx, u_prev_qhx)
        self.opti.set_value(self.p_u_prev_tin, u_prev_tin)

        if hasattr(self, 'last_sol'):
            prev_Qhx = self.last_sol.value(self.Qhx)
            prev_Tin = self.last_sol.value(self.Tin)
            prev_X = self.last_sol.value(self.X)
            self.opti.set_initial(self.Qhx, np.hstack([prev_Qhx[:, 1:], prev_Qhx[:, -1:]]))
            self.opti.set_initial(self.Tin,  np.hstack([prev_Tin[:, 1:],  prev_Tin[:, -1:]]))
            self.opti.set_initial(self.X,    np.hstack([prev_X[:, 1:],    prev_X[:, -1:]]))

        try:
            sol = self.opti.solve()
            self.last_sol = sol
            return [sol.value(self.Qhx[0, 0]), sol.value(self.Tin[0, 0])]
        except Exception as e:
            self._fail_count += 1
            print(f"[TempMPC] Solver failed (total failures: {self._fail_count}): {e}")
            if hasattr(self, 'last_sol'):
                return [self.last_sol.value(self.Qhx[0, 0]), self.last_sol.value(self.Tin[0, 0])]
            return [u_prev_qhx, u_prev_tin]


# --- Simple Temp MPC (N=3, CasADi/L4CasADi, single-shooting) ---
class SimpleTempMPC:
    """
    N-step temperature MPC using CasADi/L4CasADi + IPOPT (single shooting).
    The horizon N is set by the module constant SIMPLE_TEMP_MPC_N (5 min/step).
    Optimises only Tin[0:N] for a smooth (quadratic) temperature tracking cost
    plus Tin smoothness. The heat-exchanger flux Qhx is held at QHX_CONST to
    avoid the bilinear Qhx*(Tin-T) degeneracy. CO2 and air are fixed (not
    optimised). The scheduled harvest/dilution disturbances (d rows 6,7) are
    fed in by the caller so the controller can pre-heat before cold-water
    injections.
    """
    def __init__(self, model_path="dynamic_model_l4casadi.pt"):
        self.N = SIMPLE_TEMP_MPC_N
        self._fail_count = 0

        nn_model_cpu = torch.load(model_path, map_location='cpu', weights_only=False)
        self.l4c_model = l4c.L4CasADi(L4CasADiWrapper(nn_model_cpu), name='simple_mpc')

        opti = cs.Opti()

        # Decision variable: inlet-temperature trajectory only (Qhx fixed)
        Tin = opti.variable(1, self.N)

        # Parameters
        p_x0         = opti.parameter(5)
        p_dist       = opti.parameter(7, self.N)
        p_T_ref      = opti.parameter()
        p_u_co2      = opti.parameter()
        p_u_air      = opti.parameter()
        p_u_prev_tin = opti.parameter()

        # Box constraints
        opti.subject_to(Tin >= U_LB[3])
        opti.subject_to(Tin <= U_UB[3])

        # Single-shooting: compose N NN calls symbolically (Qhx held constant)
        x_k = p_x0
        T_preds = []
        for k in range(self.N):
            u_k  = cs.vertcat(p_u_co2, p_u_air, QHX_CONST, Tin[0, k])
            x_sc = (x_k          - cs.DM(X_MEAN)) / cs.DM(X_STD)
            u_sc = (u_k          - cs.DM(U_MEAN)) / cs.DM(U_STD)
            d_sc = (p_dist[:, k] - cs.DM(D_MEAN)) / cs.DM(D_STD)
            delta_sc = self.l4c_model(cs.vertcat(x_sc, u_sc, d_sc)).T  # (5,1)
            x_k = x_k + delta_sc * cs.DM(X_STD)
            T_preds.append(x_k[4])

        # Cost: smooth quadratic tracking over N steps + Tin smoothness
        w_sp = 1.0; w_s = 0.5
        tin_range = U_UB[3] - U_LB[3]
        jsp_T = cs.sum1(cs.vertcat(*[(p_T_ref - T_k) ** 2 for T_k in T_preds]))
        js_tin = ((Tin[0, 0] - p_u_prev_tin) / tin_range) ** 2
        for i in range(1, self.N):
            js_tin += ((Tin[0, i] - Tin[0, i-1]) / tin_range) ** 2
        opti.minimize(w_sp * jsp_T + w_s * js_tin)

        opti.solver('ipopt', {'ipopt': {
            'print_level': 0,  # quiet; _fail_count print surfaces problems
            'max_iter': 200,
            'tol': 1e-2,
            'acceptable_tol': 1e-1,
            'acceptable_iter': 5,
            'hessian_approximation': 'limited-memory',
            'mu_strategy': 'adaptive',
        }})

        self.opti = opti
        self.Tin = Tin
        self.p_x0 = p_x0; self.p_dist = p_dist; self.p_T_ref = p_T_ref
        self.p_u_co2 = p_u_co2; self.p_u_air = p_u_air
        self.p_u_prev_tin = p_u_prev_tin

    def solve(self, x_curr, d_matrix, T_ref, u_co2=0.0, u_air=0.0,
              u_prev_qhx=None, u_prev_tin=None):
        # u_prev_qhx is accepted for call-signature compatibility but unused:
        # the flux is fixed at QHX_CONST and only Tin is optimised.
        if u_prev_tin is None: u_prev_tin = float(T_ref)

        self.opti.set_value(self.p_x0, x_curr)
        self.opti.set_value(self.p_dist, d_matrix)
        self.opti.set_value(self.p_T_ref, T_ref)
        self.opti.set_value(self.p_u_co2, u_co2)
        self.opti.set_value(self.p_u_air, u_air)
        self.opti.set_value(self.p_u_prev_tin, u_prev_tin)

        if hasattr(self, 'last_sol'):
            prev_Tin = self.last_sol.value(self.Tin).reshape(1, self.N)
            self.opti.set_initial(self.Tin, np.hstack([prev_Tin[:, 1:], prev_Tin[:, -1:]]))
        else:
            self.opti.set_initial(self.Tin, np.full((1, self.N), u_prev_tin))

        try:
            sol = self.opti.solve()
            self.last_sol = sol
            return [QHX_CONST, float(sol.value(self.Tin[0, 0]))]
        except Exception as e:
            self._fail_count += 1
            print(f"[SimpleTempMPC] Solver failed ({self._fail_count}): {e}")
            if hasattr(self, 'last_sol'):
                return [QHX_CONST, float(self.last_sol.value(self.Tin[0, 0]))]
            return [QHX_CONST, float(T_ref)]


# --- Simple pH MPC (CasADi/L4CasADi, single-shooting) ---
class SimplePHMPC:
    """
    N-step pH MPC using CasADi/L4CasADi + IPOPT (single shooting).
    The horizon N is set by the module constant SIMPLE_PH_MPC_N (5 min/step).
    Optimises only the CO2 injection CO2[0:N] to track the benchmark pH
    setpoint. Air, heat-exchanger flux Qhx and inlet temperature Tin are held
    fixed (defaults: training means, to keep the NN rollout in-distribution).

    Cost: the control-action penalty (consumption) and control-action smoothness
    follow the main MicroalgaeMPC cost (same smoothness_cost / consumption_cost
    helpers and pH weights). Setpoint tracking uses a smooth quadratic (as in
    SimpleTempMPC) rather than the main class's relative_absolute_error: the
    abs() kink at the setpoint crossing makes the smooth IPOPT NLP thrash.
    """
    def __init__(self, model_path="dynamic_model_l4casadi.pt"):
        self.N = SIMPLE_PH_MPC_N
        self._fail_count = 0

        nn_model_cpu = torch.load(model_path, map_location='cpu', weights_only=False)
        self.l4c_model = l4c.L4CasADi(L4CasADiWrapper(nn_model_cpu), name='simple_ph_mpc')

        opti = cs.Opti()

        # Decision variable: CO2-injection trajectory only (Air/Qhx/Tin fixed).
        # Optimised in NORMALISED units [0, 1] and mapped to physical CO2 via
        # U_UB[0] (U_LB[0]=0). Physical CO2 is O(1e-4), which is badly scaled
        # for IPOPT; the normalised variable keeps the NLP well-conditioned.
        CO2n = opti.variable(1, self.N)
        CO2  = CO2n * U_UB[0]   # 1xN physical CO2 row (m^3/s)

        # Parameters
        p_x0         = opti.parameter(5)
        p_dist       = opti.parameter(7, self.N)
        p_pH_ref     = opti.parameter()
        p_u_air      = opti.parameter()
        p_u_qhx      = opti.parameter()
        p_u_tin      = opti.parameter()
        p_u_prev_co2 = opti.parameter()

        # Box constraints (normalised)
        opti.subject_to(CO2n >= 0.0)
        opti.subject_to(CO2n <= 1.0)

        # Single-shooting: compose N NN calls symbolically (Air/Qhx/Tin held constant)
        x_k = p_x0
        pH_preds = []
        for k in range(self.N):
            u_k  = cs.vertcat(CO2[0, k], p_u_air, p_u_qhx, p_u_tin)
            x_sc = (x_k          - cs.DM(X_MEAN)) / cs.DM(X_STD)
            u_sc = (u_k          - cs.DM(U_MEAN)) / cs.DM(U_STD)
            d_sc = (p_dist[:, k] - cs.DM(D_MEAN)) / cs.DM(D_STD)
            delta_sc = self.l4c_model(cs.vertcat(x_sc, u_sc, d_sc)).T  # (5,1)
            x_k = x_k + delta_sc * cs.DM(X_STD)
            pH_preds.append(x_k[0])

        # Cost: control smoothness + consumption follow the main MicroalgaeMPC
        # cost (same helpers + pH weights); tracking is a smooth quadratic.
        w = {'sp': 1196, 's': 2.6, 'c': 0.001}
        pH_row = cs.horzcat(*pH_preds)                                  # 1xN, like X[0, 1:]
        jsp = cs.sum2((p_pH_ref - pH_row) ** 2)                         # setpoint tracking
        js  = smoothness_cost(CO2, p_u_prev_co2, U_LB[0], U_UB[0])      # control smoothness
        jc  = consumption_cost(CO2, U_UB[0])                            # control magnitude
        opti.minimize(w['sp'] * jsp + w['s'] * js + w['c'] * jc)

        opti.solver('ipopt', {'ipopt': {
            'print_level': 0,  # quiet; _fail_count print surfaces problems
            'max_iter': 200,
            'tol': 1e-2,
            'acceptable_tol': 1e-1,
            'acceptable_iter': 5,
            'hessian_approximation': 'limited-memory',
            'mu_strategy': 'adaptive',
        }})

        self.opti = opti
        self.CO2n = CO2n; self.CO2 = CO2
        self.p_x0 = p_x0; self.p_dist = p_dist; self.p_pH_ref = p_pH_ref
        self.p_u_air = p_u_air; self.p_u_qhx = p_u_qhx; self.p_u_tin = p_u_tin
        self.p_u_prev_co2 = p_u_prev_co2

    def solve(self, x_curr, d_matrix, pH_ref, u_air=None, u_qhx=None,
              u_tin=None, u_prev_co2=0.0):
        # Non-optimised actuators: use the actually-applied values from the
        # co-running DO/Temp controllers when supplied, so the pH prediction is
        # not biased by assuming training-mean air/temperature. Missing values
        # (None / NaN, e.g. before the other controllers have run) fall back to
        # the training means to keep the NN rollout in-distribution.
        def _val(v, default):
            if v is None: return default
            v = float(v)
            return default if not np.isfinite(v) else v
        u_air = _val(u_air, U_MEAN[1])
        u_qhx = _val(u_qhx, U_MEAN[2])
        u_tin = _val(u_tin, U_MEAN[3])

        self.opti.set_value(self.p_x0, x_curr)
        self.opti.set_value(self.p_dist, d_matrix)
        self.opti.set_value(self.p_pH_ref, pH_ref)
        self.opti.set_value(self.p_u_air, u_air)
        self.opti.set_value(self.p_u_qhx, u_qhx)
        self.opti.set_value(self.p_u_tin, u_tin)
        self.opti.set_value(self.p_u_prev_co2, u_prev_co2)

        if hasattr(self, 'last_sol'):
            prev_CO2n = self.last_sol.value(self.CO2n).reshape(1, self.N)
            self.opti.set_initial(self.CO2n, np.hstack([prev_CO2n[:, 1:], prev_CO2n[:, -1:]]))
        else:
            self.opti.set_initial(self.CO2n, np.full((1, self.N), u_prev_co2 / U_UB[0]))

        try:
            sol = self.opti.solve()
            self.last_sol = sol
            return [float(sol.value(self.CO2[0, 0]))]
        except Exception as e:
            self._fail_count += 1
            print(f"[SimplePHMPC] Solver failed ({self._fail_count}): {e}")
            if hasattr(self, 'last_sol'):
                return [float(self.last_sol.value(self.CO2[0, 0]))]
            return [0.0]


# --- Singleton Logic ---
_solver_instance = None
_temp_solver_instance = None
_simple_temp_solver_instance = None
_simple_ph_solver_instance = None


def get_mpc_action(x, d_matrix, r, u_prev_ext):
    global _solver_instance, _state_history
    x_np = np.array(x)
    _state_history.append(x_np.copy())
    if len(_state_history) > _MAX_HISTORY:
        _state_history.pop(0)
    if _solver_instance is None:
        _solver_instance = MicroalgaeMPC()
    return _solver_instance.solve(x_np, np.array(d_matrix), np.array(r), np.array(u_prev_ext))


def get_prediction_plot(x_curr, u_fixed, d_matrix, refs=None,
                        save_path="nn_prediction_debug.png",
                        model_path="dynamic_model_l4casadi.pt"):
    """MATLAB-callable wrapper: plot NN predictions using the stored history buffer.

    Call from MATLAB via:
        pyrun("from mpc_handler import get_prediction_plot; p = get_prediction_plot(x, u, d, r)", ...
              "p", x=x_curr, u=u_prev, d=d_future, r=r_curr)
    """
    hist = np.array(_state_history[:-1]) if len(_state_history) > 1 else np.empty((0, 5))
    return plot_nn_predictions(
        hist, np.array(x_curr), np.array(u_fixed), np.array(d_matrix),
        refs=list(np.array(refs)) if refs is not None else None,
        save_path=save_path, model_path=model_path
    )


def get_temp_mpc_action(x, d_matrix, T_ref, u_co2, u_air, u_prev_qhx, u_prev_tin):
    global _temp_solver_instance
    if _temp_solver_instance is None:
        _temp_solver_instance = TemperatureOnlyMPC()
    return _temp_solver_instance.solve(
        np.array(x), np.array(d_matrix),
        float(T_ref), float(u_co2), float(u_air),
        float(u_prev_qhx), float(u_prev_tin)
    )


def get_simple_temp_mpc_action(x, d_matrix, T_ref, u_prev_qhx=0.0, u_prev_tin=30.0):
    global _simple_temp_solver_instance
    if _simple_temp_solver_instance is None:
        _simple_temp_solver_instance = SimpleTempMPC()
    return _simple_temp_solver_instance.solve(
        np.array(x).ravel(), np.array(d_matrix), float(T_ref),
        u_prev_qhx=float(u_prev_qhx), u_prev_tin=float(u_prev_tin)
    )


def get_simple_ph_mpc_action(x, d_matrix, pH_ref, u_prev_co2=0.0,
                             u_air=None, u_qhx=None, u_tin=None):
    global _simple_ph_solver_instance
    if _simple_ph_solver_instance is None:
        _simple_ph_solver_instance = SimplePHMPC()
    return _simple_ph_solver_instance.solve(
        np.array(x).ravel(), np.array(d_matrix), float(pH_ref),
        u_air=u_air, u_qhx=u_qhx, u_tin=u_tin, u_prev_co2=float(u_prev_co2)
    )
