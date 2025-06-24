import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import RegularGridInterpolator, griddata

# ------------------------------------------------------------------
# 0. Flux‑map (measured) preparation – simple linear map demo
# ------------------------------------------------------------------
Pn = 4
Ld_nom, Lq_nom = 0.3e-3, 0.5e-3
psi_f_nom = 0.08

IdAxis = np.linspace(-150, 150, 61)
IqAxis = np.linspace(-200, 400, 61)
Id_mesh, Iq_mesh = np.meshgrid(IdAxis, IqAxis, indexing="ij")

PhiDMap = Ld_nom * Id_mesh + psi_f_nom
PhiQMap = Lq_nom * Iq_mesh

phi_d_lut = RegularGridInterpolator((IdAxis, IqAxis), PhiDMap,
                                    bounds_error=False, fill_value=None)
phi_q_lut = RegularGridInterpolator((IdAxis, IqAxis), PhiQMap,
                                    bounds_error=False, fill_value=None)

points = np.array([PhiDMap.ravel(), PhiQMap.ravel()]).T
values_id = Id_mesh.ravel()
values_iq = Iq_mesh.ravel()

psi_d_axis = np.linspace(PhiDMap.min(), PhiDMap.max(), 61)
psi_q_axis = np.linspace(PhiQMap.min(), PhiQMap.max(), 61)
grid_psi_d, grid_psi_q = np.meshgrid(psi_d_axis, psi_q_axis, indexing="ij")

IdTable = griddata(points, values_id, (grid_psi_d, grid_psi_q), method="nearest")
IqTable = griddata(points, values_iq, (grid_psi_d, grid_psi_q), method="nearest")

id_lut = RegularGridInterpolator((psi_d_axis, psi_q_axis), IdTable,
                                 bounds_error=False, fill_value=None)
iq_lut = RegularGridInterpolator((psi_d_axis, psi_q_axis), IqTable,
                                 bounds_error=False, fill_value=None)

# ------------------------------------------------------------------
# 1. Plant using flux dynamics + map inverse
# ------------------------------------------------------------------
class FluxMapPlant:
    def __init__(self, Rs=0.05, Ts=1e-4):
        self.Rs = Rs
        self.phi_d = 0.0
        self.phi_q = 0.0
        self.Ts = Ts

    def step(self, v_d, v_q, omega):
        i_d = float(id_lut([self.phi_d, self.phi_q]))
        i_q = float(iq_lut([self.phi_d, self.phi_q]))

        dphi_d = v_d - self.Rs * i_d + omega * self.phi_q
        dphi_q = v_q - self.Rs * i_q - omega * self.phi_d

        self.phi_d += self.Ts * dphi_d
        self.phi_q += self.Ts * dphi_q

        torque = 1.5 * Pn * (self.phi_d * i_q - self.phi_q * i_d)
        return self.phi_d, self.phi_q, i_d, i_q, torque

# ------------------------------------------------------------------
# 2. EKF specialised for flux‑map model (states: phi_d, phi_q, Rs)
# ------------------------------------------------------------------
class FluxMapEKF:
    def __init__(self, Ts, Rs_init=0.04):
        self.Ts = Ts
        self.x = np.array([0.0, 0.0, Rs_init])  # phi_d, phi_q, Rs
        self.P = np.diag([1e-4, 1e-4, 1e-4])
        self.Q = np.diag([1e-6, 1e-6, 1e-8])
        self.R = np.diag([0.1, 0.1])**2  # current sensor noise std^2

    def _f(self, x, u):
        phi_d, phi_q, Rs = x
        v_d, v_q, omega = u
        i_d = float(id_lut([phi_d, phi_q]))
        i_q = float(iq_lut([phi_d, phi_q]))
        dphi_d = v_d - Rs * i_d + omega * phi_q
        dphi_q = v_q - Rs * i_q - omega * phi_d
        return np.array([
            phi_d + self.Ts * dphi_d,
            phi_q + self.Ts * dphi_q,
            Rs  # random walk
        ])

    def _jacobian_F(self, x, u, eps=1e-6):
        n = len(x)
        F = np.zeros((n, n))
        f0 = self._f(x, u)
        for k in range(n):
            dx = x.copy()
            dx[k] += eps
            F[:, k] = (self._f(dx, u) - f0) / eps
        return F

    def predict(self, u):
        Fk = self._jacobian_F(self.x, u)
        self.x = self._f(self.x, u)
        self.P = Fk @ self.P @ Fk.T + self.Q

    def update(self, z):
        # measurement: currents
        def h(x):
            phi_d, phi_q, _ = x
            return np.array([
                float(id_lut([phi_d, phi_q])),
                float(iq_lut([phi_d, phi_q]))
            ])
        # Jacobian H via finite diff
        n = len(self.x)
        H = np.zeros((2, n))
        h0 = h(self.x)
        eps = 1e-6
        for k in range(n):
            dx = self.x.copy()
            dx[k] += eps
            H[:, k] = (h(dx) - h0) / eps

        # Kalman update
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        y = z - h0
        self.x = self.x + K @ y
        self.P = self.P - K @ H @ self.P

    # helpers
    @property
    def phi_d(self): return self.x[0]
    @property
    def phi_q(self): return self.x[1]
    @property
    def Rs(self):    return self.x[2]
    @property
    def i_d(self):   return float(id_lut([self.phi_d, self.phi_q]))
    @property
    def i_q(self):   return float(iq_lut([self.phi_d, self.phi_q]))
    @property
    def torque(self):
        return 1.5 * Pn * (self.phi_d * self.i_q - self.phi_q * self.i_d)

# ------------------------------------------------------------------
# 3. Simulation
# ------------------------------------------------------------------
def simulate():
    plant = FluxMapPlant(Rs=0.05, Ts=1e-4)
    ekf   = FluxMapEKF(Ts=1e-4, Rs_init=0.04)

    v_d, v_q = 20.0, 100.0
    omega = 2 * np.pi * 150
    steps = 30000  # 3 s

    history = {"t": [], "i_d_true": [], "i_d_est": [],
               "i_q_true": [], "i_q_est": [],
               "phi_d_true": [], "phi_d_est": [],
               "phi_q_true": [], "phi_q_est": [],
               "Rs_true": [], "Rs_est": [],
               "Te_true": [], "Te_est": []}

    obs_std = np.sqrt(np.diag(ekf.R))
    for k in range(steps):
        t = k * plant.Ts
        # plant step
        phi_d, phi_q, i_d, i_q, Te = plant.step(v_d, v_q, omega)

        # measurement with noise
        z = np.array([i_d, i_q]) + np.random.normal(0, obs_std, 2)

        # EKF
        ekf.predict([v_d, v_q, omega])
        ekf.update(z)

        # log
        history["t"].append(t)
        history["i_d_true"].append(i_d)
        history["i_d_est"].append(ekf.i_d)
        history["i_q_true"].append(i_q)
        history["i_q_est"].append(ekf.i_q)
        history["phi_d_true"].append(phi_d)
        history["phi_d_est"].append(ekf.phi_d)
        history["phi_q_true"].append(phi_q)
        history["phi_q_est"].append(ekf.phi_q)
        history["Rs_true"].append(plant.Rs)
        history["Rs_est"].append(ekf.Rs)
        history["Te_true"].append(Te)
        history["Te_est"].append(ekf.torque)

    # convert to numpy for quick plotting
    t = np.array(history["t"])
    plt.figure(figsize=(10, 8))
    plt.subplot(3,1,1)
    plt.plot(t, history["i_d_true"], "r--", label="Id true")
    plt.plot(t, history["i_d_est"], "b-", label="Id est")
    plt.plot(t, history["i_q_true"], "m--", label="Iq true")
    plt.plot(t, history["i_q_est"], "c-", label="Iq est")
    plt.ylabel("Current [A]")
    plt.legend(); plt.grid(True)

    plt.subplot(3,1,2)
    plt.plot(t, history["phi_d_true"], "r--", label="phi_d true")
    plt.plot(t, history["phi_d_est"], "b-", label="phi_d est")
    plt.plot(t, history["phi_q_true"], "m--", label="phi_q true")
    plt.plot(t, history["phi_q_est"], "c-", label="phi_q est")
    plt.ylabel("Flux [Wb]")
    plt.legend(); plt.grid(True)

    plt.subplot(3,1,3)
    plt.plot(t, history["Te_true"], "r--", label="Torque true")
    plt.plot(t, history["Te_est"], "b-", label="Torque est")
    plt.xlabel("Time [s]")
    plt.ylabel("Torque [Nm]")
    plt.legend(); plt.grid(True)

    plt.tight_layout()
    plt.show()

simulate()
