data = readtable('flux_raw.csv');    % 列: id, iq, phid, phiq
id  = data.id;  iq  = data.iq;
phid = data.phid;  phiq = data.phiq;

% --- 偶奇対称性 (必要ならペア処理を実装) -----------------------------
% ここでは割愛。Python 例と同じロジックで実装可
% ---------------------------------------------------------------

% --- 補間器作成 ---------------------------------------------------
Fd = scatteredInterpolant(id, iq, phid, 'natural', 'none');
Fq = scatteredInterpolant(id, iq, phiq, 'natural', 'none');

id_vec = linspace(min(id), max(id), 121);
iq_vec = linspace(min(iq), max(iq), 121);
[ID, IQ] = meshgrid(id_vec, iq_vec);

phi_d_map = Fd(ID, IQ);
phi_q_map = Fq(ID, IQ);

save('flux_map.mat', 'id_vec', 'iq_vec', 'phi_d_map', 'phi_q_map');
disp('→ flux_map.mat を保存しました');

%% load flux map
load('flux_map.mat', 'id_vec', 'iq_vec', 'phi_d_map', 'phi_q_map');

% optional: 数値微分で Ld, Lq を求める
[ID, IQ] = meshgrid(id_vec, iq_vec);
d_id = id_vec(2) - id_vec(1);
d_iq = iq_vec(2) - iq_vec(1);

Ld_map = gradient(phi_d_map, d_id, d_iq);   % Ld ≈ ∂φd/∂id
[~, Lq_map] = gradient(phi_q_map, d_id, d_iq); % Lq ≈ ∂φq/∂iq

figure('Name','Flux surface','Color','w');

subplot(1,2,1)
surf(ID, IQ, phi_d_map, 'EdgeColor','none');
xlabel('i_d [A]'); ylabel('i_q [A]'); zlabel('\phi_d [Wb]');
title('\phi_d(id, iq)'); colormap turbo; colorbar; view(40,30);
grid on; axis tight;

subplot(1,2,2)
surf(ID, IQ, phi_q_map, 'EdgeColor','none');
xlabel('i_d [A]'); ylabel('i_q [A]'); zlabel('\phi_q [Wb]');
title('\phi_q(id, iq)'); colormap turbo; colorbar; view(40,30);
grid on; axis tight;

figure('Name','Flux contour','Color','w');

subplot(1,2,1)
contourf(ID, IQ, phi_d_map, 40, 'LineColor','none');
xlabel('i_d [A]'); ylabel('i_q [A]');
title('\phi_d – contour'); axis equal tight; colorbar; colormap turbo;

subplot(1,2,2)
contourf(ID, IQ, phi_q_map, 40, 'LineColor','none');
xlabel('i_d [A]'); ylabel('i_q [A]');
title('\phi_q – contour'); axis equal tight; colorbar; colormap turbo;

figure('Name','Inductance maps','Color','w');

subplot(1,2,1)
surf(ID, IQ, Ld_map*1e3, 'EdgeColor','none');
xlabel('i_d [A]'); ylabel('i_q [A]'); zlabel('L_d [mH]');
title('L_d(id, iq)'); colorbar; view(45,25);

subplot(1,2,2)
surf(ID, IQ, Lq_map*1e3, 'EdgeColor','none');
xlabel('i_d [A]'); ylabel('i_q [A]'); zlabel('L_q [mH]');
title('L_q(id, iq)'); colorbar; view(45,25);

%% 元の散在測定点を重ねて誤差を可視化
scatter3(data.id, data.iq, data.phid, 30, 'k', 'filled'); hold on;
surf(ID, IQ, phi_d_map, 'EdgeAlpha',0.2, 'FaceAlpha',0.7);
legend('Measured','Interpolated'); view(40,30);

