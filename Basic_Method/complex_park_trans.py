import numpy as np

# IQDemodulator クラスは既にあるものとする

# ... (IQDemodulator のインスタンス化とループ処理) ...

# どこかの時点で、推定または実際の回転子電気角 theta_e が利用可能であるとする
# 例: theta_e = current_rotor_electrical_angle

# IQDemodulator から Ihα と Ihβ を取得
# (ここでは簡単のため、Ih が complex モードの戻り値であると仮定)
# Ih = demod.update(Iα, Iβ, t)
Ih_alpha_beta_complex = Ih # この Ih は Ihα + 1j * Ihβ の形式

# 仮の回転子電気角 (例として)
theta_e = 0.5 # 仮の値。実際にはエンコーダなどから取得

# パーク変換を実行
# 回転子座標系への変換は e^(-jθe) を乗算
Ih_gamma_delta_complex = Ih_alpha_beta_complex * np.exp(-1j * theta_e)

# Iγ と Iδ を抽出
Igamma = Ih_gamma_delta_complex.real
Idelta = Ih_gamma_delta_complex.imag

print(f"Ih_alpha_beta_complex: {Ih_alpha_beta_complex}")
print(f"Igamma: {Igamma}, Idelta: {Idelta}")