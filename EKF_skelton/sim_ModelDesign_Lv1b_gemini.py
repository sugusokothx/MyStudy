
import numpy as np
import matplotlib.pyplot as plt

class PMSM_EKF_Simulator_Salient:
    def __init__(self, params):
        # --- PMSM and Simulation Parameters ---
        self.Rs = params['Rs']
        self.psi_f_true = params['psi_f_true']
        self.Ld_true = params['Ld_true'] # True Ld for simulation
        self.Lq_true = params['Lq_true'] # True Lq for simulation
        self.p = params['p']
        self.Ts = params['Ts']
        self.T_total = params['T_total']
        self.vd = params['vd']
        self.vq = params['vq']
        
        # Convert RPM to electrical rad/s
        self.omega = params['p'] * params['Omega_rpm'] * 2 * np.pi / 60

        # --- EKF Parameters ---
        # State vector: [id, iq, psi_f, Ld, Lq]
        self.n_states = 5
        # Initial state estimate (psi_f, Ld, Lq are intentionally incorrect)
        self.x_est = np.array([0.0, 0.0, params['psi_f_initial_guess'], 
                               params['Ld_initial_guess'], params['Lq_initial_guess']])
        # Initial covariance matrix
        self.P_est = np.eye(self.n_states) * 1.0 # Increased initial uncertainty
        # Process noise covariance Q (5x5 matrix for [id, iq, psi_f, Ld, Lq])
        self.Q = np.diag(params['Q_diag'])
        # Measurement noise covariance R (2x2 for [id, iq])
        self.R = np.diag(params['R_diag'])
        # Measurement matrix H
        self.H = np.array([, 
                          ])

    def _f_discrete(self, x, u):
        """ Discrete-time nonlinear state transition function f(x, u) for salient PMSM """
        id_k, iq_k, psi_f_k, Ld_k, Lq_k = x
        vd_k, vq_k, omega_k = u
        
        # PMSM d-q axis equations with Ld and Lq
        # id_dot = (1/Ld) * (-Rs * id + omega * Lq * iq + vd)
        # iq_dot = (1/Lq) * (-Rs * iq - omega * Ld * id - omega * psi_f + vq)

        # Ensure Ld_k and Lq_k are not zero to avoid division by zero
        Ld_k_safe = max(Ld_k, 1e-9) 
        Lq_k_safe = max(Lq_k, 1e-9)

        id_kp1 = id_k + self.Ts * (1/Ld_k_safe) * (-self.Rs * id_k + omega_k * Lq_k_safe * iq_k + vd_k)
        iq_kp1 = iq_k + self.Ts * (1/Lq_k_safe) * (-self.Rs * iq_k - omega_k * Ld_k_safe * id_k - omega_k * psi_f_k + vq_k)
        psi_f_kp1 = psi_f_k # Random walk model for flux
        Ld_kp1 = Ld_k       # Random walk model for Ld
        Lq_kp1 = Lq_k       # Random walk model for Lq
        
        return np.array([id_kp1, iq_kp1, psi_f_kp1, Ld_kp1, Lq_kp1])

    def _calculate_F(self, x, u):
        """ Calculate the state Jacobian matrix F for salient PMSM """
        id_k, iq_k, psi_f_k, Ld_k, Lq_k = x
        vd_k, vq_k, omega_k = u

        # Ensure Ld_k and Lq_k are not zero for Jacobian calculation
        Ld_k_safe = max(Ld_k, 1e-9)
        Lq_k_safe = max(Lq_k, 1e-9)
          
        # Partial derivatives of f_discrete w.r.t. id, iq, psi_f, Ld, Lq
        # f1 = id_k + Ts * (1/Ld_k) * (-Rs * id_k + omega_k * Lq_k * iq_k + vd_k)
        # f2 = iq_k + Ts * (1/Lq_k) * (-Rs * iq_k - omega_k * Ld_k * id_k - omega_k * psi_f_k + vq_k)
        # f3 = psi_f_k
        # f4 = Ld_k
        # f5 = Lq_k

        F = np.zeros((self.n_states, self.n_states))

        # Row 1 (id_kp1 derivatives)
        F = 1 + self.Ts * (-self.Rs / Ld_k_safe) # d(f1)/d(id_k)
        F = self.Ts * (omega_k * Lq_k_safe / Ld_k_safe) # d(f1)/d(iq_k)
        F = 0.0 # d(f1)/d(psi_f_k)
        F = self.Ts * (-1 / (Ld_k_safe**2)) * (-self.Rs * id_k + omega_k * Lq_k_safe * iq_k + vd_k) # d(f1)/d(Ld_k)
        F = self.Ts * (omega_k * id_k / Ld_k_safe) # d(f1)/d(Lq_k)

        # Row 2 (iq_kp1 derivatives)
        F = self.Ts * (-omega_k * Ld_k_safe / Lq_k_safe) # d(f2)/d(id_k)
        F = 1 + self.Ts * (-self.Rs / Lq_k_safe) # d(f2)/d(iq_k)
        F = self.Ts * (-omega_k / Lq_k_safe) # d(f2)/d(psi_f_k)
        F = self.Ts * (-omega_k * id_k / Lq_k_safe) # d(f2)/d(Ld_k)
        F = self.Ts * (-1 / (Lq_k_safe**2)) * (-self.Rs * iq_k - omega_k * Ld_k_safe * id_k - omega_k * psi_f_k + vq_k) # d(f2)/d(Lq_k)

        # Row 3 (psi_f_kp1 derivatives)
        F = 1.0 # d(f3)/d(psi_f_k)

        # Row 4 (Ld_kp1 derivatives)
        F = 1.0 # d(f4)/d(Ld_k)

        # Row 5 (Lq_kp1 derivatives)
        F = 1.0 # d(f5)/d(Lq_k)
          
        return F

    def run_simulation(self):
        """ Run the full simulation """
        num_steps = int(self.T_total / self.Ts)
          
        # --- Storage for results ---
        x_true_history = np.zeros((num_steps, self.n_states))
        x_est_history = np.zeros((num_steps, self.n_states))