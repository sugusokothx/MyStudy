# run_simulation.py

import numpy as np
import matplotlib.pyplot as plt
import japanize_matplotlib

# motor_model.py ファイルから必要なクラスをインポート
from motor_model import MotorModel, MotorParameters

def inverse_park_transform(i_d, i_q, theta_e):
    """逆dq変換（逆パーク変換）によりd-q軸電流を三相電流に変換する"""
    i_a = i_d * np.cos(theta_e) - i_q * np.sin(theta_e)
    i_b = i_d * np.cos(theta_e - 2 * np.pi / 3) - i_q * np.sin(theta_e - 2 * np.pi / 3)
    i_c = i_d * np.cos(theta_e + 2 * np.pi / 3) - i_q * np.sin(theta_e + 2 * np.pi / 3)
    return i_a, i_b, i_c

# --- メインの実行ブロック ---
if __name__ == '__main__':
    # --- シミュレーションパラメータの設定 ---
    motor_params = MotorParameters(
        R_s=0.5,
        L_d_map=0.001,
        L_q_map=0.002,
        psi_dq_map=lambda psi_s_dq: 0.1 + 0j,
        n_p=4
    )

    # シミュレーション条件
    Ts = 1e-6
    T_sim = 0.05
    time = np.arange(0, T_sim, Ts)

    # 入力値
    u_dq_input = 50.0 + 20.0j
    w_m_input = 150.0

    # --- シミュレーションの準備 ---
    motor = MotorModel(par=motor_params) # インポートしたクラスのインスタンスを作成
    motor.inp.u_dq = u_dq_input
    motor.inp.w_m = w_m_input
    
    # 結果保存用のリスト
    history_id, history_iq, history_torque = [], [], []
    history_ia, history_ib, history_ic = [], [], []
    
    theta_e = 0.0
    w_e = motor.par.n_p * motor.inp.w_m

    # --- シミュレーションループ ---
    print("シミュレーションを開始します...")
    for t in time:
        motor.step(Ts)
        i_dq_out = motor.out.i_dq
        torque_out = motor.out.torque
        
        theta_e = (theta_e + w_e * Ts) % (2 * np.pi)
        ia, ib, ic = inverse_park_transform(i_dq_out.real, i_dq_out.imag, theta_e)

        history_id.append(i_dq_out.real)
        history_iq.append(i_dq_out.imag)
        history_torque.append(torque_out)
        history_ia.append(ia)
        history_ib.append(ib)
        history_ic.append(ic)
    print("シミュレーションが完了しました。")

    # --- 結果のプロット ---
    print("結果をプロットします...")
    fig, axes = plt.subplots(3, 1, figsize=(12, 18), constrained_layout=True)
    fig.suptitle(f'モーターシミュレーション結果 (Ud={u_dq_input.real}V, Uq={u_dq_input.imag}V, ωm={w_m_input}rad/s)', fontsize=16)
    
    axes[0].plot(time, history_id, label='d軸電流 (Id)')
    axes[0].plot(time, history_iq, label='q軸電流 (Iq)')
    axes[0].set_title('d-q軸電流の時間応答'); axes[0].set_xlabel('時間 [s]'); axes[0].set_ylabel('電流 [A]'); axes[0].grid(True); axes[0].legend()

    axes[1].plot(time, history_torque, label='トルク', color='g')
    axes[1].set_title('発生トルクの時間応答'); axes[1].set_xlabel('時間 [s]'); axes[1].set_ylabel('トルク [Nm]'); axes[1].grid(True); axes[1].legend()

    axes[2].plot(time, history_ia, label='a相電流 (ia)')
    axes[2].plot(time, history_ib, label='b相電流 (ib)')
    axes[2].plot(time, history_ic, label='c相電流 (ic)')
    axes[2].set_title('三相固定子電流'); axes[2].set_xlabel('時間 [s]'); axes[2].set_ylabel('電流 [A]'); axes[2].grid(True); axes[2].legend()
    
    plt.show()
    print("プロットを終了しました。")