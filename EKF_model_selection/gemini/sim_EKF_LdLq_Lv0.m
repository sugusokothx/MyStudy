function [psi_f_est, Ld_est, Lq_est, P_diag, Id_est, Iq_est]=ekf_ipm_estimator(id, iq, vd, vq, omega)

persistent x_est psi_f_est

Rs=30/1000;
Ts=1/5000;

Q=diag([1e-4, 1e-4, 1e-7, 1e-8, 1e-8]);
R=diag([1e-3, 1e-3]);

H=[1 0 0 0 0;
   0 1 0 0 0];

if isempty(x_est)
    %[id;iq;psi_f;Ld;Lq]
    x_est=[0;0;0.010;0.012;0.012];
    P_est=eye(5)*0.01;
end

z=[id;iq];

u=[vd; vq; omega];

% 予測ステップ
x_pred = f_discrete_ipm(x_est, u, Rs, Ts);
F = calculate_F_ipm(x_est, u, Rs, Ts);
P_pred = F*P_est*F'+Q;

% 更新ステップ
y_pred = H*x_pred;
P_est=(eye(5)-K*H)*P_pred;
K=P_pred*H'/S';

x_est=x_pred+K*(z-y_pred);
P_est=(eye(5)-K*H)*P_pred;

% バウンディング処理
x_est(4) = max(1e-4, x_est(4));
x_est(5) = max(1e-4, x_est(5));

psi_f_est = x_est(3);
Ld_est = x_est(4);
Lq_est = x_est(5);
P_diag = diag(P_est);
Id_est = x_est(1);
Iq_est = x_est(2);

end

function x_next = f_discrete_ipm(x, u, Rs, Ts)

id = x(1);
iq = x(2);
psi_f = x(3);
Ld = x(4);
Lq = x(5);

vd = u(1);
vq = u(2);
omega = u(3);

did_dt = (1/Ld)*(-Rs*id + omega*Lq*iq + vd);
diq_dt = (1/Lq)*(-Rs*iq - omega*Ld*id - omega*psi_f + vq);

id_next = id + Ts*did_dt;
iq_next = iq + Ts*diq_dt;
psi_f_next = psi_f;
Ld_next = Ld;
Lq_next = Lq;

x_next = [id_next;iq_next;psi_f_next;Ld_next;Lq_next];

end

function F = calculate_F_ipm(x, u, Rs, Ts)

id = x(1);
iq = x(2);
psi_f = x(3);
Ld = x(4);
Lq = x(5);

vd = u(1);
vq = u(2);
omega = u(3);


F=eye(5);

F(1,1)=1-Ts*Rs/Ld;
F(1,2)=Ts*omega*Lq/Ld;
F(1,4)=Ts*(Rs*id-omega*Lq*iq-vd)/(Ld^2);
F(1,5)=Ts*omega*iq/Ld;
F(2,1)=-Ts*omega*Ld/Lq;
F(2,2)=1-Ts*Rs/Lq;
F(2,3)=-Ts*omega/Lq;
F(2,4)=-Ts*omega*id/Lq;
F(2,5)=Ts*(Rs*iq+omega*Ld*id+omega*psi_f-vq)/(Lq^2);

end