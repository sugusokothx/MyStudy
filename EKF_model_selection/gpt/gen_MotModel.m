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
