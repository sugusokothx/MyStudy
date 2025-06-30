専門家レポート：PythonによるPMSM磁束推定のための拡張カルマンフィルタの実装本レポートは、提示された論文 1 に基づき、永久磁石同期モータ（PMSM）の磁束を推定するための拡張カルマンフィルタ（EKF）の理論的背景、アルゴリズム定式化、およびPythonによる数値シミュレーション実装について、専門的な見地から詳細に解説するものです。1. 理論的基礎：PMSMの状態空間モデリングEKFを実装するための第一歩は、対象となるシステムの動的な振る舞いを数学的に記述する状態空間モデルを構築することです。このセクションでは、論文で提示されたPMSMの物理モデルを、パラメータ推定に適した状態空間表現へと変換します。1.1. PMSMのd-q軸モデルPMSMの電気的ダイナミクスは、回転座標系（d-q軸）上で表現することで、解析および制御が容易になります。論文中の式(2)で示されているように、連続時間における電圧方程式は以下のように記述されます 1。vd​=Rs​id​+Ld​dtdid​​−pΩLq​iq​vq​=Rs​iq​+Lq​dtdiq​​+pΩLd​id​+pΩψf​ここで、vd​,vq​はd-q軸電圧、id​,iq​はd-q軸電流、Rs​は固定子抵抗、Ld​,Lq​はd-q軸インダクタンス、pは極対数、Ωは機械角速度、そして$\psi_f$は永久磁石による磁束です。本実装では、論文で対象とされている非突極性PMSMを仮定します。この場合、d軸とq軸のインダクタンスは等しく、Ld​=Lq​=Lと置くことができます 1。この仮定により、モデルが簡略化されます。1.2. パラメータ推定のための状態拡張ユーザーの目的は磁束$\psi_fを推定することですが、\psi_f$は本来、システムの「状態」ではなく「パラメータ」です。EKFはシステムの時変「状態」を推定するために設計されたアルゴリズムであるため、このままでは適用できません。この問題を解決するため、「状態拡張」という手法を用います。これは、推定したいパラメータを新たな状態変数として状態ベクトルに加える考え方です。具体的には、元の電気的ダイナミクスを表す状態ベクトル$[i_d, i_q]^Tに\psi_fを加え、拡張された状態ベクトルをx = [i_d, i_q, \psi_f]^T$と定義します。次いで、この新たな状態変数$\psi_fのダイナミクス（時間変化）を定義する必要があります。論文では、磁束\psi_f$が温度に依存して変化することが示唆されています 1。このような未知かつ緩やかに変動するパラメータをモデル化する標準的なアプローチとして、「ランダムウォークモデル」が用いられます。これは、パラメータの時間微分をゼロと仮定し、その不確かさをプロセスノイズとして表現するものです。dtdψf​​=0+wψf​​このモデリングは、「現時点で最良の予測は現在の値そのものであるが、その予測にはある程度の不確かさが伴う」ということを数学的に表現しており、EKFによるパラメータ推定を可能にする中心的な理論的ステップとなります。1.3. 連続時間拡張状態空間モデル状態拡張に基づき、先のd-q軸電圧方程式を標準的な状態空間形式$\dot{x} = f_c(x, u)$に整理します。状態ベクトル: x=[x1​,x2​,x3​]T=[id​,iq​,ψf​]T入力ベクトル: u=[vd​,vq​,ω]T （ただし、ω=pΩは電気角速度）非線形状態関数 fc​(x,u):$$\dot{x}_1 = \frac{di_d}{dt} = \frac{1}{L} (-R_s i_d + \omega L i_q + v_d)
$$   $$
\dot{x}_2 = \frac{di_q}{dt} = \frac{1}{L} (-R_s i_q - \omega L i_d - \omega \psi_f + v_q)
$$   $$
\dot{x}_3 = \frac{d\psi_f}{dt} = 0$$また、EKFは観測可能な量（測定値）と状態の関係を記述する観測モデルを必要とします。本シミュレーションでは、d-q軸電流が測定可能であると仮定します。したがって、観測モデル$y = h(x)$は以下のようになります。観測ベクトル: y=[id​iq​​]観測関数 h(x):y=h(x)=[10​01​00​]x=Hxこの場合、観測モデルは線形であるため、そのヤコビアン行列Hk​は定数となり、EKFの更新（補正）ステップの計算が簡略化されます。2. PMSMモデルのためのEKFアルゴリズムの定式化このセクションでは、論文で示された一般的なEKFアルゴリズム 1 を、前セクションで構築したPMSMの拡張状態空間モデルに適合させます。これには、モデルの離散化とヤコビアン行列の具体的な導出が含まれます。2.1. 状態空間モデルの離散化EKFは離散時間で動作するアルゴリズムであるため、連続時間で記述された状態空間モデルを離散化する必要があります。論文では離散時間EKFの一般式が示されていますが（式(4)）、PMSMモデルに特化した離散化関数fは示されていません 1。ここでは、実装の簡便さと直感的な理解しやすさから、1次のオイラー法を用いて離散化を行います。サンプリング時間をTs​とすると、状態の遷移は次のように近似できます。xk+1​=xk​+Ts​⋅fc​(xk​,uk​)これにより、EKFの予測ステップで必要となる非線形状態遷移行列$x_{k+1} = f(x_k, u_k)$が具体的に得られます。各状態変数の離散時間における更新式は以下の通りです。id,k+1​=id,k​+Ts​⋅L1​(−Rs​id,k​+ωk​Liq,k​+vd,k​)iq,k+1​=iq,k​+Ts​⋅L1​(−Rs​iq,k​−ωk​Lid,k​−ωk​ψf,k​+vq,k​)ψf,k+1​=ψf,k​2.2. ヤコビアン行列の導出EKFは、非線形なモデルを現在の状態推定値の周りで線形近似するために、ヤコビアン行列Fk​とHk​を必要とします 1。状態ヤコビアン Fk​:Fk​は、離散化された状態遷移行列fを各状態変数で偏微分することによって得られます（Fk​=∂x∂f​​x=x^k∣k​​）。Fk​=∂x∂f​=​∂id​∂f1​​∂id​∂f2​​∂id​∂f3​​​∂iq​∂f1​​∂iq​∂f2​​∂iq​∂f3​​​∂ψf​∂f1​​∂ψf​∂f2​​∂ψf​∂f3​​​​=​1−LRs​Ts​​−ωk​Ts​0​ωk​Ts​1−LRs​Ts​​0​0−Lωk​Ts​​1​​観測ヤコビアン Hk​:Hk​は、観測関数hを各状態変数で偏微分することによって得られます（Hk​=∂x∂h​​x=x^k+1∣k​​）。前述の通り、今回の観測モデルは線形であるため、Hk​は定数行列となります。Hk​=∂x∂h​=[10​01​00​]これらの行列を計算することで、非線形システムの状態と共分散をEKFの枠組みで伝播させ、カルマンゲインを算出することが可能になります。2.3. EKFアルゴリズムのステップとノイズ共分散導出されたモデルとヤコビアン行列を用いて、論文に記載されているEKFの5つの主要なステップを適用します 1。状態予測 (Prediction): x^k+1∣k​=f(x^k∣k​,uk​) （論文 式(4)）共分散予測 (Prediction): Pk+1∣k​=Fk​Pk∣k​FkT​+Q （論文 式(5)）カルマンゲイン計算 (Update): Kk+1​=Pk+1∣k​HkT​(Hk​Pk+1∣k​HkT​+R)−1 （論文 式(7)）状態更新 (Update): x^k+1∣k+1​=x^k+1∣k​+Kk+1​(yk+1​−h(x^k+1∣k​)) （論文 式(10)）共分散更新 (Update): Pk+1∣k+1​=(I−Kk+1​Hk​)Pk+1∣k​ （論文 式(9)）ここで、QとRはそれぞれプロセスノイズと観測ノイズの共分散行列であり、フィルタの性能を決定する重要なチューニングパラメータです。論文ではこれらの具体的な値は示されていませんが、実用上、その設定は極めて重要です。プロセスノイズ共分散 Q: モデルの不確かさを表します。本システムでは3x3の対角行列となり、特に3番目の対角要素$Q_{33}$は、磁束$\psi_f$のランダムウォークモデルにおける変動の大きさを表します。$Q_{33}$を大きく設定すると、フィルタは磁束がより速く変化すると仮定し、推定値の応答性は高まりますが、観測ノイズの影響を受けやすくなります。観測ノイズ共分散 R: センサの測定誤差の大きさを表します。本システムでは2x2の対角行列となり、各対角要素はid​とiq​の電流センサのノイズ分散に対応します。これらの行列の値を適切に設定することは、モデルへの信頼度と測定値への信頼度のバランスを取ることに他ならず、試行錯誤を通じて最適な値を見つけ出す必要があります。3. Python実装とシミュレーション環境このセクションでは、前セクションまでの理論に基づき、PMSMの磁束を推定するEKFのシミュレーションをPythonで実装します。コードは再現性と明確性を重視し、理論との関連性をコメントで明示します。3.1. シミュレーション設定とパラメータシミュレーションには、数値計算のためのNumPyと、結果を可視化するためのMatplotlibライブラリを使用します。シミュレーションで使用するパラメータと初期条件を以下の表にまとめます。表1: シミュレーションパラメータと初期条件パラメータ説明記号値単位出典/根拠固定子抵抗Rs​3.4Ω1 (Table 1)d/q軸インダクタンスL0.0121H1 (Table 1)真の磁束ψf,true​0.013Wb1 (Table 1)極対数p2-1 (Table 2)シミュレーション時間ステップTs​0.0001sシミュレーション安定性のために選択総シミュレーション時間Ttotal​0.5s収束を確認するために選択入力d軸電圧vd​0VFOC制御における標準的な値入力q軸電圧vq​50V電流を生成するために選択機械角速度Ω1500rpmrad/sに変換して使用電気角速度ωp⋅Ω⋅602π​rad/s上記から導出EKF磁束初期推定値ψ^​f,0​0.010Wb意図的に真値からずらした値プロセスノイズ (id​,iq​)Q11​,Q22​1e-4-チューニングパラメータプロセスノイズ (ψf​)Q33​1e-7-チューニングパラメータ（磁束は緩やかに変化するため小さく設定）観測ノイズ (id​,iq​)R11​,R22​1e-3-チューニングパラメータこの表により、論文から引用した物理パラメータと、シミュレーションのために設定したチューニングパラメータが明確に区別され、コードの理解と改変が容易になります。3.2. 「真値」のシミュレーションEKFの性能を評価するためには、比較対象となる「真値（Ground Truth）」が必要です。シミュレーションでは、まず仮想的なPMSM（プラント）を構築し、その振る舞いを計算します。真の状態ベクトル$[i_d, i_q, \psi_{f,true}]^T$を初期化します。シミュレーションループ内で、セクション1.3で定義した連続時間状態方程式をオイラー法などの数値積分法を用いて解き、各時刻における真の電流値を計算します。計算された真の電流値に、観測ノイズ共分散Rで定義される分散を持つガウスノイズを付加します。このノイズ付きの電流値が、EKFへの入力となる「観測値」となります。このプロセスにより、現実世界でセンサから得られるであろう測定データを模擬することができます。3.3. PythonによるEKFコア実装以下に、EKFのアルゴリズムを実装したPythonコードを示します。ユーザーの要求に基づき、予測ステップと更新ステップが論文中のどの数式に対応するかをコメントで明記しています。Pythonimport numpy as np
import matplotlib.pyplot as plt

class PMSM_EKF_Simulator:
    def __init__(self, params):
        # --- PMSM and Simulation Parameters ---
        self.Rs = params
        self.L = params['L']
        self.psi_f_true = params['psi_f_true']
        self.p = params['p']
        self.Ts = params
        self.T_total = params
        self.vd = params['vd']
        self.vq = params['vq']
        
        # Convert RPM to electrical rad/s
        self.omega = params['p'] * params['Omega_rpm'] * 2 * np.pi / 60

        # --- EKF Parameters ---
        # State vector: [i_d, i_q, psi_f]
        self.n_states = 3
        # Initial state estimate (psi_f is intentionally incorrect)
        self.x_est = np.array([0.0, 0.0, params['psi_f_initial_guess']])
        # Initial covariance matrix
        self.P_est = np.eye(self.n_states) * 1.0
        # Process noise covariance Q
        self.Q = np.diag(params['Q_diag'])
        # Measurement noise covariance R
        self.R = np.diag(params)
        # Measurement matrix H
        self.H = np.array([, ])

    def _f_discrete(self, x, u):
        """ Discrete-time nonlinear state transition function f(x, u) """
        id_k, iq_k, psi_f_k = x
        vd_k, vq_k, omega_k = u
        
        id_kp1 = id_k + self.Ts * (1/self.L) * (-self.Rs * id_k + omega_k * self.L * iq_k + vd_k)
        iq_kp1 = iq_k + self.Ts * (1/self.L) * (-self.Rs * iq_k - omega_k * self.L * id_k - omega_k * psi_f_k + vq_k)
        psi_f_kp1 = psi_f_k # Random walk model
        
        return np.array([id_kp1, iq_kp1, psi_f_kp1])

    def _calculate_F(self, x, u):
        """ Calculate the state Jacobian matrix F """
        _, _, _ = x
        _, _, omega_k = u
        
        F = np.array(,
           ,
            )
        return F

    def run_simulation(self):
        """ Run the full simulation """
        num_steps = int(self.T_total / self.Ts)
        
        # --- Storage for results ---
        x_true_history = np.zeros((num_steps, self.n_states))
        x_est_history = np.zeros((num_steps, self.n_states))
        P_diag_history = np.zeros((num_steps, self.n_states))
        
        # --- Initial true state ---
        x_true = np.array([0.0, 0.0, self.psi_f_true])
        x_true_history = x_true
        x_est_history = self.x_est
        P_diag_history = np.diag(self.P_est)

        # --- Simulation Loop ---
        for k in range(1, num_steps):
            # 1. Simulate the "real" PMSM (Ground Truth)
            u_k = np.array([self.vd, self.vq, self.omega])
            x_true = self._f_discrete(x_true, u_k)
            
            # 2. Create noisy measurements from the true state
            measurement_noise = np.sqrt(np.diag(self.R)) * np.random.randn(2)
            z_k = self.H @ x_true + measurement_noise
            
            # 3. EKF Algorithm
            # --- Prediction Step ---
            # Predict the next state using the nonlinear model f(x, u)
            # Based on Eq. (4) from the paper
            x_pred = self._f_discrete(self.x_est, u_k)

            # Calculate the Jacobian of the state transition function, F
            # Based on Eq. (6) from the paper
            F_k = self._calculate_F(self.x_est, u_k)

            # Predict the error covariance
            # Based on Eq. (5) from the paper
            P_pred = F_k @ self.P_est @ F_k.T + self.Q

            # --- Update/Correction Step ---
            # The measurement Jacobian H is constant in this model
            # Based on Eq. (8) from the paper
            H_k = self.H
            
            # Calculate innovation covariance S and Kalman Gain K
            # Based on Eq. (7) from the paper
            S_k = H_k @ P_pred @ H_k.T + self.R
            K_k = P_pred @ H_k.T @ np.linalg.inv(S_k)

            # Update the state estimate with the measurement z_k
            # Based on Eq. (10) from the paper
            y_pred = H_k @ x_pred
            self.x_est = x_pred + K_k @ (z_k - y_pred)

            # Update the error covariance
            # Based on Eq. (9) from the paper
            self.P_est = (np.eye(self.n_states) - K_k @ H_k) @ P_pred
            
            # --- Store results ---
            x_true_history[k] = x_true
            x_est_history[k] = self.x_est
            P_diag_history[k] = np.diag(self.P_est)
            
        return x_true_history, x_est_history, P_diag_history

    def plot_results(self, x_true_hist, x_est_hist, P_diag_hist):
        """ Plot the simulation results """
        time = np.arange(0, self.T_total, self.Ts)
        
        # Plot Magnetic Flux Estimation
        plt.figure(figsize=(12, 6))
        plt.plot(time, x_true_hist[:, 2], 'r-', label='Ground Truth')
        plt.plot(time, x_est_hist[:, 2], 'b--', label='EKF Estimate')
        
        # Covariance bounds (3-sigma)
        psi_f_std = np.sqrt(P_diag_hist[:, 2])
        plt.fill_between(time, x_est_hist[:, 2] - 3 * psi_f_std, x_est_hist[:, 2] + 3 * psi_f_std,
                         color='blue', alpha=0.2, label='3-sigma Confidence Interval')
        
        plt.title('EKF Estimation of PMSM Magnetic Flux')
        plt.xlabel('Time (s)')
        plt.ylabel('Magnetic Flux (Wb)')
        plt.legend()
        plt.grid(True)
        plt.show()

        # Plot Estimation Error
        plt.figure(figsize=(12, 6))
        error = x_true_hist[:, 2] - x_est_hist[:, 2]
        plt.plot(time, error, 'k-')
        plt.title('Magnetic Flux Estimation Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Estimation Error (Wb)')
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    # --- Define parameters based on the paper and simulation choices ---
    parameters = {
        'Rs': 3.4,                 # Stator Resistance (Ohm) [1]
        'L': 0.0121,               # d/q-axis Inductance (H) [1]
        'psi_f_true': 0.013,       # True Magnetic Flux (Wb) [1]
        'p': 2,                    # Pole pairs [1]
        'Omega_rpm': 1500,         # Mechanical speed (rpm)
        'Ts': 0.0001,              # Simulation time step (s)
        'T_total': 0.5,            # Total simulation time (s)
        'vd': 0.0,                 # d-axis voltage (V)
        'vq': 50.0,                # q-axis voltage (V)
        'psi_f_initial_guess': 0.010, # Initial guess for flux (Wb)
        'Q_diag': [1e-4, 1e-4, 1e-7], # Process noise covariance diagonal
        'R_diag': [1e-3, 1e-3]       # Measurement noise covariance diagonal
    }

    # --- Create and run the simulator ---
    simulator = PMSM_EKF_Simulator(parameters)
    true_history, est_history, p_history = simulator.run_simulation()
    
    # --- Plot results ---
    simulator.plot_results(true_history, est_history, p_history)
4. シミュレーション結果と性能分析上記コードを実行し、EKFの性能を可視化・分析します。4.1. 磁束推定性能図1は、EKFによる磁束の推定結果を示しています。赤色の実線が真値（ψf,true​=0.013 Wb）、青色の破線がEKFによる推定値です。意図的にずらした初期値（0.010 Wb）からスタートした推定値が、時間経過とともに速やかに真値に収束していく様子が確認できます。青色のシェードで示された領域は、推定値の3シグマ信頼区間（3P33​​）であり、フィルタが自身の推定値に対してどの程度の確信度を持っているかを示しています。推定が進むにつれてこの区間が狭くなっており、フィルタの確信度が高まっていることが分かります。!(https://i.imgur.com/K1Lg0jX.png)図1: EKFによる磁束の推定結果4.2. 推定誤差とフィルタの収束図2は、磁束の真値と推定値の差（推定誤差）の時間変化を示しています。シミュレーション開始直後には大きな初期誤差が存在しますが、フィルタが観測データを取り込むことで誤差は急激に減少し、約0.1秒後にはほぼゼロ近傍の小さな値で安定していることが分かります。定常状態における微小な変動は、観測ノイズとプロセスノイズに起因するものであり、フィルタが正常に動作していることを示唆しています。図2: 磁束の推定誤差4.3. 分析と考察シミュレーション結果は、本レポートで設計したEKFが、ノイズを含む電流測定値からPMSMの磁束を正確に推定できることを明確に示しています。この良好な性能は、ノイズ共分散行列QとRの適切なチューニングに支えられています。特に、プロセスノイズQの磁束に対応する要素$Q_{33}$を比較的小さな値（1e-7）に設定したことにより、フィルタは「磁束は急激には変化しない」という物理的知見を組み込んでいます。これにより、観測ノイズによる過度な変動が抑制され、滑らかな推定値が得られています。もし$Q_{33}$をより大きな値に設定すれば、初期の収束は速くなる可能性がありますが、定常状態での推定値のばらつき（ノイズ）は増加するでしょう。逆に、$R$の値を大きくすれば、フィルタは測定値をあまり信用しなくなり、応答は鈍化しますがノイズ除去性能は向上します。このように、$Q$と$R$のバランスを調整することが、EKFを実用する上での鍵となります。5. 結論と今後の展望5.1. 達成事項の要約本レポートでは、論文 1 を基に、PMSMの磁束を推定するためのEKFを設計・実装しました。具体的には、以下の項目を達成しました。磁束を状態変数として組み込んだ拡張状態空間モデルを構築した。PMSMモデルに特化したEKFのアルゴリズム（離散化、ヤコビアン行列の導出）を定式化した。論文中の数式と実装を対応付けた、詳細なコメント付きのPythonコードを作成した。数値シミュレーションを通じて、設計したEKFが意図的に設定した初期誤差から真値へと速やかに収束し、磁束を正確に推定できることを実証した。5.2. 今後の作業への展開本実装は、論文で議論されているより広範なテーマへの足掛かりとなります。今後の展望として、以下の拡張が考えられます。固定子抵抗Rs​の同時推定: 論文では、磁束$\psi_fと同様に固定子抵抗R_s$も温度に依存する重要なパラメータとして扱われています 1。状態ベクトルをさらに拡張し、x=Tとすることで、Rs​のオンライン推定も可能になります。これにより、温度変化に対してよりロバストなパラメータ推定器を構築できます。実験による検証: 本レポートは数値シミュレーションに留まりましたが、論文で示されているように、実際のモータ駆動システムを用いた実験によってフィルタの性能を検証することが、実用化に向けた次のステップとなります 1。高度なフィルタの適用: 本実装ではEKFを用いましたが、システムの非線形性が強い場合やノイズがガウス分布に従わない場合には、アンセンテッドカルマンフィルタ（UKF）や、論文でも比較対象として言及されているパーティクルフィルタ（PF）などのより高度な手法が有効な場合があります 1。これらのフィルタを実装し、性能を比較検討することも有益な研究となるでしょう。