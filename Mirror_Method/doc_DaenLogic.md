はい、可能です！
楕円運動を「同じ角速度で互いに逆方向に回転する2本の円ベクトルの和」として表す考え方は、Cut-the-Knot の解説にある幾何学的事実で、物理（偏波楕円、振動解析など）でも右回り・左回り成分（円偏波成分）に分解する標準的な表現と対応します。([cut-the-knot.org][1], [ウィキペディア][2], [opg.optica.org][3])

---

## 数学的背景（最小限）

2次元ベクトルを複素数 $z(t)=x(t)+iy(t)$ とみなします。半径 $r_1,r_2$、共通角速度 $\omega$、初期位相 $\phi_1,\phi_2$ の **正方向**・**逆方向**回転円ベクトルを

$$
z_1(t)=r_1 e^{i(\omega t+\phi_1)},\qquad
z_2(t)=r_2 e^{-i(\omega t+\phi_2)}
$$

とすると、合成

$$
z(t)=z_1(t)+z_2(t)
$$

の軌跡は楕円になります（位相の平行移動や座標回転で標準形に持ち込める）。特に位相差を揃えて座標を回転しておけば

$$
x(t)=(r_1+r_2)\cos\omega t,\qquad
y(t)=(r_1-r_2)\sin\omega t,
$$

従って

$$
\left(\frac{x}{r_1+r_2}\right)^2+\left(\frac{y}{r_1-r_2}\right)^2=1,
$$

確かに楕円（特殊例：$r_2=0$ で円、$r_1=r_2$ で線分）。([cut-the-knot.org][1], [ウィキペディア][2], [opg.optica.org][3])

この「右回り成分」「左回り成分」への分解は、電磁波の偏波を右円偏波＋左円偏波に分ける表現（Jones ベクトルの R/L 基底）と同型です。([ウィキペディア][2], [opg.optica.org][3])

---

## MATLAB デモコード

下記スクリプトは：

1. パラメータ（$r_1,r_2,\omega,\phi_1,\phi_2$）を設定。
2. 2本の回転ベクトルを計算。
3. 合成軌跡（楕円）を描画。
4. アニメーションでベクトルの回転と合成点を可視化。
5. 自動的に標準形（共通位相除去）楕円長短半径と回転角も計算して表示。

> **使い方**
> そのまま `.m` ファイルに保存して実行。キーボード入力でパラメータを変えたい場合はコード末尾のコメントを参照。

```matlab
function dyno_ellipse_demo()
% DYNO_ELLIPSE_DEMO  Decompose an ellipse into two counter-rotating circles.
% Inspired by Cut-the-Knot "Dynamic construction of ellipse" and polarization ellipse ideas.
%
% You can edit the parameters below and rerun.

% ---- User parameters ----
r1    = 2.0;          % radius of +omega (CCW) circle
r2    = 1.0;          % radius of -omega (CW) circle
omega = 2*pi*0.2;     % rad/s  (0.2 Hz)
phi1  = deg2rad(20);  % initial phase of +omega vector
phi2  = deg2rad(-40); % initial phase of -omega vector
T     = 5;            % total sim time [s]
dt    = 0.01;         % time step [s]
do_anim = true;       % animate?

% ---- Time base ----
t = 0:dt:T;

% ---- Two rotating vectors (complex form) ----
z1 = r1 * exp( 1i*(omega*t + phi1) );   % CCW
z2 = r2 * exp(-1i*(omega*t + phi2) );   % CW  (note minus sign in exponent)

% ---- Sum ----
z  = z1 + z2;
x  = real(z);
y  = imag(z);

% ---- Compute ellipse parameters from general (phase-shifted) form ----
% Remove global rotation: rotate by mean phase
phi_avg = (phi1 - phi2)/2;  % note sign because second term has -omega t
% Actually, to align with canonical derivation we also need time shift:
% We'll numerically fit major/minor axes instead (robust & simple).
[xc,yc,a,b,theta] = fitEllipseFromPoints(x,y);

% ---- Static plot of full trajectory ----
figure('Name','Dyno Ellipse Demo'); clf;
plot(x,y,'k-','LineWidth',1.5); hold on; axis equal;
xlabel('x'); ylabel('y'); grid on;
title('Ellipse as sum of two counter-rotating circles');

% Draw circles for reference at t=0 (centred at origin)
th = linspace(0,2*pi,200);
plot(r1*cos(th), r1*sin(th), 'r:');
plot(r2*cos(th), r2*sin(th), 'b:');

% Annotation
txt = sprintf('r_1=%.3g, r_2=%.3g,  \\omega=%.3g rad/s\\nEstimated ellipse: a=%.3g, b=%.3g, \\theta=%.1f^\\circ', ...
    r1,r2,omega,a,b,rad2deg(theta));
text(0.02,0.98,txt,'Units','normalized','VerticalAlignment','top');

if do_anim
    % Handles for animation
    h1 = plot(nan,nan,'r-','LineWidth',2); % vector 1
    h2 = plot(nan,nan,'b-','LineWidth',2); % vector 2
    hsum = plot(nan,nan,'mo','MarkerFaceColor','m'); % sum tip
    for k = 1:numel(t)
        % endpoints
        p1 = [real(z1(k)), imag(z1(k))];
        p2 = [real(z2(k)), imag(z2(k))];
        ps = [x(k), y(k)];
        set(h1,'XData',[0 p1(1)],'YData',[0 p1(2)]);
        set(h2,'XData',[0 p2(1)],'YData',[0 p2(2)]);
        set(hsum,'XData',ps(1),'YData',ps(2));
        drawnow;
    end
end

end % main function

% -------------------------------------------------------------------------
function [xc,yc,a,b,theta] = fitEllipseFromPoints(x,y)
% Quick, numerically robust ellipse fit (least-squares conic -> geom params).
% Returns center (xc,yc), semi-axes a>=b, and rotation theta (rad).
% Uses Fitzgibbon-style direct fit modified for numeric stability.

% Build design matrix
D = [x(:).^2, x(:).*y(:), y(:).^2, x(:), y(:), ones(size(x(:)))];
% Scatter matrix
S = D.'*D;
% Constraint matrix (enforce ellipse 4ac - b^2 > 0)
C = zeros(6); C(1,3) = 2; C(2,2) = -1; C(3,1) = 2;
% Solve generalized eig (S*a = lambda*C*a)
[ev,~] = eig(S,C);
% Find an eigenvector that yields an ellipse
cond = 4*ev(1,:).*ev(3,:) - ev(2,:).^2;
i = find(cond > 0,1,'first');
if isempty(i), error('No ellipse found'); end
a_vec = ev(:,i);

% Conic coefficients
A=a_vec(1); B=a_vec(2); Cc=a_vec(3); Dd=a_vec(4); Ee=a_vec(5); F=a_vec(6);

% Center
den = B^2 - 4*A*Cc;
xc = (2*Cc*Dd - B*Ee)/den;
yc = (2*A*Ee - B*Dd)/den;

% Translate to center
F_c = F + A*xc^2 + B*xc*yc + Cc*yc^2 + Dd*xc + Ee*yc;
A_c = A; B_c = B; C_c = Cc;

% Rotation angle
theta = 0.5*atan2(B_c, A_c - C_c);

% Semi-axes
cos_t = cos(theta); sin_t = sin(theta);
Ap = A_c*cos_t^2 + B_c*cos_t*sin_t + C_c*sin_t^2;
Cp = A_c*sin_t^2 - B_c*cos_t*sin_t + C_c*cos_t^2;
% Ax^2 + Cy^2 = -F_c
a = sqrt(-F_c/Ap);
b = sqrt(-F_c/Cp);
if a < b
    [a,b] = deal(b,a);
    theta = theta + pi/2;
end
end
```

---

## 「標準形」とパラメータの関係を確認する

上のコードでは一般位相を持つ 2円ベクトルを与えても楕円フィットで幾何学パラメータ（中心、長半径 $a$、短半径 $b$、回転角 $\theta$）を抽出します。Cut-the-Knot の議論にならって位相を揃えた場合、長半径 $a = r_1 + r_2$、短半径 $b = |r_1 - r_2|$ となることを、コード冒頭のパラメータを変更しながら確認してみてください。([cut-the-knot.org][1], [ウィキペディア][2])

---

## 実験して理解を深めるためのヒント

| 実験      | 推奨設定                       | 観察ポイント               |
| ------- | -------------------------- | -------------------- |
| 楕円→円    | `r2 = 0`                   | 軌跡が半径 `r1` の円になる。    |
| 楕円→線分   | `r1 = r2`                  | x 軸上の往復運動（線偏波）。      |
| 長軸/短軸変化 | `r1 > r2` vs `r1 < r2`     | どちらが長半径になるか確認。       |
| 回転角の効果  | 片方の位相を変える (`phi1`, `phi2`) | 楕円の傾きが変化。            |
| 角速度変更   | `omega`                    | アニメーション速度のみ変化。形状は不変。 |

（偏波との類比：円偏波 = 一方のみ、線偏波 = 同振幅逆回転、一般楕円偏波 = 不等振幅。) ([ウィキペディア][2], [opg.optica.org][3])

---

## 次にやれること

* **信号処理視点**：2チャンネル信号（例：$x(t),y(t)$）から複素包絡 $z(t)=x+iy$ を作り、単一周波数成分を抽出（FFTや狭帯域フィルタ）後に複素回転基底 $(R,L)$ へ投影すると $r_1,r_2$ 推定が可能。([ウィキペディア][2], [opg.optica.org][3])
* **極端ケースで検証**：シミュレーションでノイズや振幅不一致を入れて推定ロバスト性を確認。
* **モータ応用**（ご関心領域）：ステータ電流の 2相成分を高速軸で複素化し、特定周波数（HF注入やリップル）を2円成分に分解して非対称を評価する、などの解析手段として利用可能。

---

必要なら Python 版や、パラメータを GUI スライダーで操作できる MATLAB App Designer 版も作成できます。ご希望があれば教えてください。

[1]: https://www.cut-the-knot.org/Curriculum/Geometry/DynoEllipse.shtml "Dynamic construction of ellipse and other curves"
[2]: https://en.wikipedia.org/wiki/Polarization_%28waves%29 "Polarization (waves) - Wikipedia"
[3]: https://opg.optica.org/viewmedia.cfm?seq=0&uri=josaa-29-1-89 "Optica Publishing Group"
