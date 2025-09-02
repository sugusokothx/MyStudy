了解。**manim（Community版）**で、添付イラスト風に「UVW 三相のステータ＋磁石ロータ＋合成ベクトル」が同期して回るアニメを作るサンプルです。
※ 三相電流の大きさは正弦波（120°位相差）でコイルの**明るさ**に反映、合成ベクトルは等角速度で回転、ロータ（N/Sバー）はそれに同期して回転します。

---

# 1) セットアップ

```bash
pip install manim  # (Community v0.18 以降を想定)
# レンダリング（高速プレビュー）
manim -pqh rotating_field.py RotatingField
# 高品質
manim -p -qk rotating_field.py RotatingField
```

---

# 2) サンプルコード（`rotating_field.py`）

```python
from math import sin, cos, pi
from manim import *

class RotatingField(Scene):
    def construct(self):
        # ── パラメータ ──────────────────────────────────────────────
        R_STATOR  = 3.0            # ステータ半径（見た目用）
        R_COIL_IN = 2.55           # コイル内側半径
        COIL_W    = 0.6            # コイル幅（円周方向）
        COIL_H    = 0.35           # コイル厚（半径方向）
        N_SLOTS   = 6              # スロット数（U,V,W,U,V,W）
        OMEGA     = 1.0 * 2*pi     # 電気角速度 [rad/s]（アニメ速度）
        RUN_TIME  = 6              # 秒数
        # 三相色
        C_U = RED
        C_V = YELLOW
        C_W = BLUE

        # ── 背景（ステータ円） ────────────────────────────────────
        stator = Circle(radius=R_STATOR, stroke_width=6, color=GRAY_C)
        self.add(stator)

        # ── スロット配置（6つ、色は U,V,W,U,V,W の順） ─────────────
        phase_colors = [C_U, C_V, C_W, C_U, C_V, C_W]
        # U/V/W の電流位相 [rad]
        phase_shift = {
            C_U: 0.0,
            C_V: -2*pi/3,   # -120°
            C_W: +2*pi/3,   # +120°
        }

        coils = VGroup()
        slot_angles = []
        for k in range(N_SLOTS):
            theta = k * 2*pi / N_SLOTS  # 0, 60°, 120°...
            slot_angles.append(theta)
            color = phase_colors[k]

            # コイル長方形（円周に沿って配置→回転）
            rect = RoundedRectangle(
                corner_radius=0.06,
                width=COIL_W, height=COIL_H,
                stroke_width=2, stroke_color=GRAY_A,
                fill_color=color, fill_opacity=0.15,  # 初期は薄く
            )
            # 原点からの配置：極座標→直交
            cx, cy = (R_COIL_IN * cos(theta), R_COIL_IN * sin(theta))
            rect.move_to([cx, cy, 0])
            # 長辺が円周方向を向くように回転
            rect.rotate(theta + pi/2)
            coils.add(rect)

        self.add(coils)

        # ── 合成ベクトル（回転磁界） ─────────────────────────────
        # 矢印の基準長
        R_VEC = 1.8
        vec_tip = Dot(radius=0.05, color=WHITE)
        res_vec = always_redraw(
            lambda: Arrow(
                start=ORIGIN,
                end=[R_VEC*cos(t.get_value()), R_VEC*sin(t.get_value()), 0],
                stroke_width=8, buff=0, color=WHITE
            )
        )
        vec_tip.add_updater(
            lambda m: m.move_to([R_VEC*cos(t.get_value()), R_VEC*sin(t.get_value()), 0])
        )
        self.add(res_vec, vec_tip)

        # ── ロータ（N/S バー磁石：合成ベクトルに同期回転） ──────────
        rotor_len = 2.0
        rotor_th  = 0.35
        # 半分ずつ色分けされたカプセル状バーをグループで作る
        barN = RoundedRectangle(corner_radius=0.15, width=rotor_len/2, height=rotor_th,
                                fill_color=RED, fill_opacity=1.0, stroke_width=0)
        barS = RoundedRectangle(corner_radius=0.15, width=rotor_len/2, height=rotor_th,
                                fill_color=BLUE, fill_opacity=1.0, stroke_width=0)
        barN.next_to(ORIGIN, RIGHT, buff=0)  # 右側（N）
        barS.next_to(ORIGIN, LEFT,  buff=0)  # 左側（S）
        rotor = VGroup(barS, barN)
        # 端に文字
        labelN = Text("N", font_size=28, color=WHITE).move_to(barN.get_center())
        labelS = Text("S", font_size=28, color=WHITE).move_to(barS.get_center())
        rotor_group = VGroup(rotor, labelN, labelS)
        self.add(rotor_group)

        # ロータを常に合成ベクトル角度に合わせる
        def align_rotor(mob):
            ang = t.get_value()
            mob.rotate(ang - mob.get_angle(), about_point=ORIGIN)
        rotor_group.add_updater(align_rotor)

        # ── 三相の正弦に応じて各スロットの輝度を変える ───────────────
        def coil_opacity(color, theta_slot):
            # 三相の時間位相：ωt + φphase
            phi = phase_shift[color]
            val = sin(OMEGA_time.get_value() + phi)  # [-1,1]
            # できれば空間的な寄与も掛けたければ cos(theta_slot - electrical_angle) 等を入れる
            # ここでは見た目を分かりやすく：正で明るく、負で暗く
            return 0.15 + 0.45 * max(0.0, val)  # 0.15~0.60

        def update_coils(group):
            for idx, rect in enumerate(group):
                col = phase_colors[idx]
                th  = slot_angles[idx]
                rect.set_fill(color=col, opacity=coil_opacity(col, th))

        coils.add_updater(update_coils)

        # ── 時間トラッカ（t: 合成ベクトルの角度, OMEGA_time: 三相信号用） ─
        t = ValueTracker(0.0)              # 合成ベクトルの角度 [rad]
        OMEGA_time = ValueTracker(0.0)     # 三相の時間位相 ωt
        # 合成ベクトルは一定角速度 → d/dt t = ω_elec * k
        # 見た目は OMEGA と同じ速度で回す
        self.add_foreground_mobject()  # no-op（整列用のダミー）

        # ── 軽い注釈 ───────────────────────────────────────────────
        caption = VGroup(
            Text("三相（U, V, W）", font_size=28, color=GRAY_B),
            Text("→ 合成ベクトル：回転磁界", font_size=28, color=WHITE)
        ).arrange(DOWN, aligned_edge=LEFT).to_corner(UL).shift(DOWN*0.3 + RIGHT*0.2)
        self.add(caption)

        # ── アニメーション本体 ─────────────────────────────────────
        # t と OMEGA_time を RUN_TIME の間、等速で進める
        self.play(
            t.animate.set_value(OMEGA * RUN_TIME),
            OMEGA_time.animate.set_value(OMEGA * RUN_TIME),
            run_time=RUN_TIME, rate_func=linear
        )
        self.wait(0.5)
```

---

## 3) 何をしているか

* \*\*スロット 6 枚（60°ピッチ）**を U/V/W/U/V/W で配置。
  `coils.add_updater(...)` でフレーム毎に**明るさ（opacity）\*\*を
  `sin(ωt + 位相)` に合わせて更新（正で明るく、負で暗く）。
* **合成ベクトル**は `Arrow(ORIGIN → R[cosθ, sinθ])` を `t` で回転。
* **ロータ（N/Sバー）**は `add_updater` で**常に合成ベクトル角へ整列**。
* 6 秒間、**一定角速度**で連続回転。

> 実写物理を厳密にやるなら、空間高調波を抑えた MMF 合成や、q軸成分のサチの表現等も入れられますが、教育用の「直感可視化」ならこの表現が軽量で分かりやすいです。

---

## 4) 仕上げのTips

* **速度調整**：`OMEGA` と `RUN_TIME` を変更
* **スロット数**：`N_SLOTS` を 12/18 等にすると「分布巻きっぽい」見た目に
* **色や太さ**：`fill_opacity`/`stroke_width`/`R_VEC` を調整
* **GIF化**：出力 mp4 を GIF に（macOS 例）

  ```bash
  ffmpeg -i media/videos/rotating_field/1080p60/RotatingField.mp4 -vf "fps=20,scale=1280:-1:flags=lanczos" rotating_field.gif
  ```

---

必要なら、**画面下に三相の波形を小窓で表示**してタイムマーカーと同期させる版、**インバータの6スイッチON/OFF**を矩形波で見せる第3章アニメ版も用意します。どこまで可視化したいか、教えてください。

了解です！
第3章の「インバータの6スイッチ ON/OFF → 矩形波 → 正弦波合成」のアニメーションを **manim** で作るサンプルコードを用意します。

---

## アニメの構成（教育用の可視化）

1. **6スイッチ（U, V, W 各上下アーム）**

   * ON時：緑に点灯
   * OFF時：灰色
   * 時間軸に従って PWM で切り替え

2. **PWM矩形波**（各相のスイッチ信号）

   * 下に 3 本の矩形波（U相, V相, W相）を描画
   * デューティ比が変化する様子を動的に

3. **合成正弦波電流**

   * 矩形波をフィルタ通過した結果として「なめらかな正弦波」に近づくことを波形で描画
   * 実際のモータ電流を模した正弦波を重畳して見せる

---

## サンプルコード（`inverter_pwm.py`）

```python
from manim import *
import numpy as np

class PWMInverter(Scene):
    def construct(self):
        # パラメータ
        T = 6         # 全体の表示時間
        fc = 8        # キャリア周波数（三角波）
        fm = 1        # 指令正弦波周波数

        # ── 三角波キャリア ──
        def carrier(t):
            # -1～1の三角波
            return 2*(abs(2*((t*fc)%1)-1))-1

        # ── 正弦波指令 ──
        def sine_cmd(t, phase=0):
            return np.sin(2*np.pi*fm*t + phase)

        # ── PWM比較 → 矩形波 ──
        def pwm_wave(t, phase=0):
            return 1 if sine_cmd(t, phase) >= carrier(t) else -1

        # ── 描画領域 ──
        axes = Axes(
            x_range=[0, T, 1],
            y_range=[-1.2, 1.2, 1],
            tips=False, axis_config={"include_numbers": False}
        ).scale(0.9).to_edge(DOWN)

        labels = axes.get_axis_labels(x_label="time", y_label="amplitude")
        self.add(axes, labels)

        # ── 波形プロット（U,V,W 相） ──
        colors = [RED, GREEN, BLUE]
        phases = [0, -2*np.pi/3, +2*np.pi/3]

        sine_lines = VGroup()
        pwm_lines = VGroup()

        for k, (c, ph) in enumerate(zip(colors, phases)):
            sine_line = axes.plot(lambda t: sine_cmd(t, ph),
                                  x_range=[0, T], color=c)
            pwm_line = axes.plot(lambda t: pwm_wave(t, ph),
                                 x_range=[0, T], color=c, stroke_width=2)
            sine_lines.add(sine_line.shift(UP*(k*0.6)))
            pwm_lines.add(pwm_line.shift(DOWN*(k*0.6)))

        # ── 注釈 ──
        caption = VGroup(
            Text("上: 指令正弦波 (U,V,W)", font_size=28, color=WHITE),
            Text("下: PWM矩形波 (スイッチON/OFF)", font_size=28, color=GRAY_B)
        ).arrange(DOWN).to_edge(UP)

        self.play(Write(caption))
        self.wait(1)

        # 描画
        self.play(Create(sine_lines))
        self.wait(1)
        self.play(Create(pwm_lines))
        self.wait(2)

        # 強調: PWM → 正弦に近づく
        note = Text("PWMパルス列をフィルタすると正弦波電流に！",
                    font_size=30, color=YELLOW).to_edge(DOWN)
        self.play(Write(note))
        self.wait(3)
```

---

## 出力

```bash
manim -pqh inverter_pwm.py PWMInverter
```

* 上段：指令三相正弦波
* 下段：PWM矩形波（各相 ON/OFF）
* 最後に「フィルタすると正弦波電流に！」と注釈

---

## 拡張アイデア

* 6個のスイッチ（IGBT/MOSFET）を絵として描き、矩形波に応じて **点灯/消灯アニメ**。
* 波形とスイッチを「同期」させることで、**回路動作 → 波形 → 回転磁界**の流れを一気に説明可能。
* 下に「合成磁界ベクトル」を追加し、**第2章アニメ**と連続させられる。

---

👉 ご希望は「**矩形波と三相正弦波の比較アニメだけ**」ですか？
それとも「**6スイッチ（インバータ回路）も一緒に描いて動かす**」完全版が欲しいですか？

了解。**「6スイッチのインバータ回路（U/V/W各レグの上下アーム）を描画し、PWMの比較結果に応じてON/OFF色を切り替え、同時に波形（指令正弦・ゲート矩形・相端子電圧・フィルタ後の電流）を表示する“完全版”** の manim サンプルを用意しました。
プレゼンに入れられる高解像度 mp4/gif を出せます。

---

# 使い方

```bash
pip install manim   # Community版 (v0.18+ 推奨)

# 高速プレビュー（水平 1080p 相当）
manim -pqh inverter_full.py PWMInverterFull

# 高品質
manim -p -qk inverter_full.py PWMInverterFull
```

---

# `inverter_full.py`

```python
from manim import *
import numpy as np

# ───────────────────────────────────────────────────────────────
# 可視化ポリシー
#  - 上アームON: 緑, 下アームON: 緑（相補）、OFF: グレー
#  - 三相指令 (U,V,W): 赤, 黄, 青
#  - 相端子電圧 v_u,v_v,v_w: 濃い色の段差波
#  - フィルタ後の相電流 i_u,i_v,i_w: 破線の正弦波（同相）
#  - 時間窓は「右→左」のオシロ表示（スクロールではなく“現在時刻を左端”にせず、常に0~Windowの範囲に t を足す）
# ───────────────────────────────────────────────────────────────

class PWMInverterFull(Scene):
    def construct(self):
        # ===== パラメータ =====
        VDC      = 1.0                 # 正規化直流電圧（±1/2を相端子電圧のレベルに使う）
        fc       = 8.0                 # キャリア（三角波）周波数 [Hz]
        fm       = 1.0                 # 指令正弦波周波数 [Hz]
        m_index  = 0.9                 # 変調指数（0~1目安）
        window_s = 1.6                 # 波形の表示ウィンドウ [s]
        run_t    = 6.0                 # 再生時間 [s]

        # 三相の位相
        phases = {
            "U": 0.0,
            "V": -2*np.pi/3,
            "W": +2*np.pi/3,
        }
        phase_color = {
            "U": RED,
            "V": YELLOW,
            "W": BLUE,
        }

        # ===== 基本関数 =====
        def triangle_carrier(t, f=fc):
            # -1～+1 の三角波
            # saw: 0..1 -> tri: -1..+1
            s = (t * f) % 1.0
            tri = 2.0 * np.abs(2.0 * (s - 0.5)) - 1.0
            return tri

        def sine_cmd(t, phase, m=m_index):
            return m * np.sin(2*np.pi*fm*t + phase)

        def pwm_gate(t, phase):  # 上アームのゲート（1/0）
            return 1.0 if sine_cmd(t, phase) >= triangle_carrier(t) else 0.0

        def leg_pole_voltage(t, phase):  # 相端子電圧（上:+Vdc/2, 下:-Vdc/2）
            return (pwm_gate(t, phase) * VDC/2.0) + ((1.0 - pwm_gate(t, phase)) * (-VDC/2.0))

        # フィルタ後の相電流は指令と同相の正弦（教育的単純化）
        def filtered_current(t, phase):
            return np.sin(2*np.pi*fm*t + phase)

        # ===== DCバス & インバータ回路のレイアウト =====
        # 配置基準
        origin_left = LEFT*5 + UP*1.2

        # DCレール
        bus_length = 3.0
        bus_gap    = 1.2
        line_plus  = Line(origin_left, origin_left + RIGHT*bus_length, color=RED)
        line_minus = Line(origin_left + DOWN*bus_gap, origin_left + DOWN*bus_gap + RIGHT*bus_length, color=BLUE)
        plus_label = Text("+Vdc", font_size=24).next_to(line_plus, LEFT, buff=0.2)
        minus_label= Text("−Vdc", font_size=24).next_to(line_minus, LEFT, buff=0.2)

        self.add(line_plus, line_minus, plus_label, minus_label)

        # レグのX位置
        leg_xs = [ -2.8, -1.2, +0.4 ]  # U, V, W
        leg_names = ["U","V","W"]

        leg_groups = []

        for leg_i, leg_name in enumerate(leg_names):
            x = leg_xs[leg_i]
            # 上下スイッチの矩形
            sw_w, sw_h = 0.6, 0.42
            up_rect   = RoundedRectangle(corner_radius=0.06, width=sw_w, height=sw_h,
                                         stroke_width=2, stroke_color=GRAY_B, fill_color=GRAY_D, fill_opacity=1.0)
            dn_rect   = RoundedRectangle(corner_radius=0.06, width=sw_w, height=sw_h,
                                         stroke_width=2, stroke_color=GRAY_B, fill_color=GRAY_D, fill_opacity=1.0)

            # 位置
            up_rect.move_to([x, line_plus.get_y() - 0.35, 0])
            dn_rect.move_to([x, line_minus.get_y() + 0.35, 0])

            # 接続ライン
            up_to_mid = Line(up_rect.get_bottom(), [x, (line_plus.get_y()+line_minus.get_y())/2, 0], color=GRAY_B)
            dn_to_mid = Line(dn_rect.get_top(),    [x, (line_plus.get_y()+line_minus.get_y())/2, 0], color=GRAY_B)
            mid_node  = Dot([x, (line_plus.get_y()+line_minus.get_y())/2, 0], radius=0.06, color=WHITE)

            # DCバスとの接続
            up_bus   = Line([x, line_plus.get_y(), 0], up_rect.get_top(), color=GRAY_B)
            dn_bus   = Line([x, line_minus.get_y(),0], dn_rect.get_bottom(), color=GRAY_B)

            # ラベル
            leg_lbl  = Text(leg_name, font_size=28, color=phase_color[leg_name]).next_to(mid_node, DOWN, buff=0.15)

            grp = VGroup(up_bus, dn_bus, up_rect, dn_rect, up_to_mid, dn_to_mid, mid_node, leg_lbl)
            self.add(grp)
            leg_groups.append((leg_name, grp, up_rect, dn_rect, mid_node))

        # ===== 波形エリア（右側） =====
        # 上段: 指令正弦 (3相)
        # 中段: 上アームゲート（矩形波, 0/1）
        # 下段: 相端子電圧（±Vdc/2）とフィルタ後の相電流（破線正弦）
        axes_scale = 0.9
        gap_y = 2.0
        right_anchor = RIGHT*4.4 + UP*1.8

        ax_cmd = Axes(
            x_range=[0, window_s, 0.2],
            y_range=[-1.2, 1.2, 1],
            x_length=6.2, y_length=1.8,
            tips=False, axis_config={"include_numbers": False}
        ).scale(axes_scale).move_to(right_anchor + UP*gap_y)
        ax_pwm = Axes(
            x_range=[0, window_s, 0.2],
            y_range=[-0.2, 1.2, 1],
            x_length=6.2, y_length=1.8,
            tips=False, axis_config={"include_numbers": False}
        ).scale(axes_scale).move_to(right_anchor + ORIGIN)
        ax_vol = Axes(
            x_range=[0, window_s, 0.2],
            y_range=[-0.7, 0.7, 0.5],
            x_length=6.2, y_length=1.8,
            tips=False, axis_config={"include_numbers": False}
        ).scale(axes_scale).move_to(right_anchor + DOWN*gap_y)

        self.add(ax_cmd, ax_pwm, ax_vol)

        # 軸ラベル
        self.add(Text("指令正弦波  (U,V,W)", font_size=22).next_to(ax_cmd, UP, buff=0.1))
        self.add(Text("上アーム・ゲート (0/1)", font_size=22).next_to(ax_pwm, UP, buff=0.1))
        self.add(Text("相端子電圧  v_u,v_v,v_w   /  フィルタ後の電流  i_u,i_v,i_w",
                      font_size=22).next_to(ax_vol, UP, buff=0.1))

        # 時間トラッカー
        t = ValueTracker(0.0)

        # 常時再描画：関数 f(x + t)
        def shifted_plot(ax, func, color, stroke_width=3, dashed=False):
            def _plot():
                if dashed:
                    # manim の DashedVMobject は plot に直接適用できないので Line 近似用に点列生成
                    xs = np.linspace(0, window_s, 600)
                    pts = np.column_stack([xs, [func(x + t.get_value()) for x in xs]])
                    poly = ax.plot_line_graph(xs, pts[:,1], add_vertex_dots=False, stroke_width=stroke_width, vertex_dot_style={})
                    return DashedVMobject(poly, num_dashes=60, color=color, dash_length=0.06).set_color(color)
                else:
                    return ax.plot(lambda x: func(x + t.get_value()), x_range=[0, window_s], color=color, stroke_width=stroke_width)
            return always_redraw(_plot)

        # 指令正弦（U,V,W）
        cmd_plots = VGroup()
        for leg in leg_names:
            cmd_plots.add(shifted_plot(ax_cmd,
                                       lambda tau, p=phases[leg]: sine_cmd(tau, p),
                                       phase_color[leg], stroke_width=3))
        self.add(cmd_plots)

        # 上アーム ゲート（矩形 0/1）
        def gate_func(leg):
            return lambda tau, p=phases[leg]: pwm_gate(tau, p)

        gate_plots = VGroup()
        for leg in leg_names:
            gate_plots.add(shifted_plot(ax_pwm, gate_func(leg), phase_color[leg], stroke_width=3))
        self.add(gate_plots)

        # 相端子電圧（±Vdc/2）の段差波
        def v_func(leg):
            return lambda tau, p=phases[leg]: leg_pole_voltage(tau, p)

        v_plots = VGroup()
        for leg in leg_names:
            v_plots.add(shifted_plot(ax_vol, v_func(leg), phase_color[leg], stroke_width=3))
        self.add(v_plots)

        # フィルタ後の電流（破線の正弦）
        i_plots = VGroup()
        for leg in leg_names:
            i_plots.add(shifted_plot(ax_vol,
                                     lambda tau, p=phases[leg]: 0.55*filtered_current(tau, p),
                                     phase_color[leg], stroke_width=3, dashed=True))
        self.add(i_plots)

        # ===== スイッチ矩形の色更新（ON=緑 / OFF=灰） =====
        def make_switch_updater(leg, up_rect, dn_rect):
            # 毎フレーム pwm_gate を評価し、色を切り替える
            phase = phases[leg]
            def _upd(mob):
                g = pwm_gate(t.get_value(), phase)
                if g >= 0.5:  # 上ON
                    up_rect.set_fill(GREEN_E, opacity=1.0)
                    dn_rect.set_fill(GRAY_D,  opacity=1.0)
                else:         # 下ON
                    up_rect.set_fill(GRAY_D,  opacity=1.0)
                    dn_rect.set_fill(GREEN_E, opacity=1.0)
            return _upd

        for leg, grp, up_rect, dn_rect, node in leg_groups:
            up_rect.add_updater(make_switch_updater(leg, up_rect, dn_rect))
            dn_rect.add_updater(lambda m: None)  # 上で同時制御

            # ノード色も相端子電圧に応じて少し変える（赤系/青系）
            def node_updater(mob, leg_name=leg):
                v = leg_pole_voltage(t.get_value(), phases[leg_name])
                c = interpolate_color(BLUE_E, RED_E, (v + 0.5) / 1.0)  # -0.5→BLUE, +0.5→RED
                mob.set_fill(c).set_stroke(c)
            node.add_updater(node_updater)

        # ===== 進める =====
        self.play(t.animate.set_value(run_t), run_time=run_t, rate_func=linear)
        self.wait(0.4)
```

---

## 画面の説明

* **左**：三相フルブリッジの6スイッチ。
  PWM比較の結果に合わせて、\*\*上/下の矩形が緑（ON）/灰（OFF）\*\*に切替。中点ノードの色も瞬時の相端子電圧で赤（＋）⇄青（−）に推移。
* **右上**：三相の**指令正弦波**（位相差120°）。
* **右中**：三相の**上アーム・ゲート信号（矩形）**。
* **右下**：三相の\*\*相端子電圧（段差波）\*\*と、\*\*フィルタ後の相電流（破線の正弦）\*\*を重ねて表示。

---

## パラメータ調整のヒント

* **変調指数**：`m_index`（0.9 → 0.5 など）。
* **キャリア周波数**：`fc`、**指令周波数**：`fm`。
* **表示窓**：`window_s`（長くすると曲線が密に）。
* **実機っぽさ**を少し足す：

  * デッドタイムを導入 → 比較結果に僅かな空白時間を入れる
  * 相電流位相を少し遅らせる（`filtered_current` に位相遅れ `-phi_lag` を足す）
  * 中性点電位や線間電圧 `v_uv = v_u - v_v` も追加プロット

---

必要なら、**回転磁界ベクトル（第2章の合成ベクトル）**をこのシーンの右端に小窓で出して「PWM→電圧→電流→回転磁界」まで**一本で繋がる**拡張も作れます。入れたい要素（デッドタイム、SVPWM、SPWM、スイッチ名表示、結線図の詳細など）を教えてください。

