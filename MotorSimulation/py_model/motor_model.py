# motor_model.py

# ==============================================================================
# 1. モーターモデルの定義
# ==============================================================================
# pyright: reportMissingImports=false
from dataclasses import dataclass
from typing import Callable
import numpy as np

@dataclass
class MotorInputs:
    """モーターモデルへの入力値を格納するクラスです。"""
    u_dq: complex = 0j
    w_m: float = 0.0

@dataclass
class MotorOutputs:
    """モーターモデルからの出力値を格納するクラスです。"""
    i_dq: complex = 0j
    torque: float = 0.0

@dataclass
class MotorParameters:
    """モーターの物理的パラメータを定義するクラスです。"""
    R_s: float
    L_d_map: Callable[[float], float] | float
    L_q_map: Callable[[float], float] | float
    psi_dq_map: Callable[[complex], complex]
    temp_coeff: float = 0.0
    n_p: int = 1

    def get_L_d(self, psi_s_dq: complex) -> float:
        return self.L_d_map(psi_s_dq.real) if callable(self.L_d_map) else self.L_d_map

    def get_L_q(self, psi_s_dq: complex) -> float:
        return self.L_q_map(psi_s_dq.imag) if callable(self.L_q_map) else self.L_q_map

    def i_s_dq(self, psi_s_dq: complex) -> complex:
        psi_pm = self.psi_dq_map(psi_s_dq)
        L_d = self.get_L_d(psi_s_dq)
        L_q = self.get_L_q(psi_s_dq)
        term_d = (psi_s_dq.real - psi_pm.real) / L_d
        term_q = 1j * (psi_s_dq.imag - psi_pm.imag) / L_q
        return term_d + term_q

class MotorModel:
    """d-q軸の磁束に基づいたシンプルなモーターモデルクラスです。"""
    def __init__(self, par: MotorParameters) -> None:
        self.par = par
        self.inp = MotorInputs()
        self.out = MotorOutputs()
        self.psi_s_dq: complex = 0j

    def compute_outputs(self) -> None:
        i_s_dq = self.par.i_s_dq(self.psi_s_dq)
        tau = 1.5 * self.par.n_p * np.imag(self.psi_s_dq * np.conj(i_s_dq))
        self.out.i_dq = i_s_dq
        self.out.torque = tau

    def rhs(self) -> complex:
        u_s_dq = self.inp.u_dq
        w_e = self.par.n_p * self.inp.w_m
        i_s_dq = self.par.i_s_dq(self.psi_s_dq)
        return u_s_dq - self.par.R_s * i_s_dq - 1j * w_e * self.psi_s_dq

    def step(self, dt: float) -> None:
        self.psi_s_dq += dt * self.rhs()
        self.compute_outputs()