% generate_flux_hessian.m
% Author: ChatGPT (o3) – 2025‑07‑02
%
% Compute 2‑nd order derivative (Hessian) maps of d‑ and q‑axis flux from
% base flux maps sampled on a regular id/iq grid.
%
% INPUTS
%   id_vec      : 1×Ni array of d‑axis current grid [A] (monotonically increasing)
%   iq_vec      : 1×Nq array of q‑axis current grid [A] (monotonically increasing)
%   phi_d_map   : Ni×Nq matrix – φd(id,iq) [Wb]
%   phi_q_map   : Ni×Nq matrix – φq(id,iq) [Wb]
%
% OUTPUTS (all Ni×Nq)
%   Hd.dd , Hd.dq , Hd.qq : second partials of φd
%   Hq.dd , Hq.dq , Hq.qq : second partials of φq
%
% Usage example:
%   [Hd,Hq] = generate_flux_hessian(id_grid, iq_grid, phiD, phiQ);
% ----------------------------------------------------------------------
function [Hd, Hq] = generate_flux_hessian(id_vec, iq_vec, phi_d_map, phi_q_map)

%% argument check
if ~isvector(id_vec) || ~isvector(iq_vec)
    error('id_vec and iq_vec must be 1‑D arrays.');
end
[id_step_ok, iq_step_ok] = internal__check_uniform_grid(id_vec, iq_vec);
if ~id_step_ok || ~iq_step_ok
    warning('Grid is not perfectly uniform; local step sizes will be used.');
end

%% helper: central difference wrapper ------------------------------------------------
central = @(f_forward, f_backward, h) (f_forward - f_backward) ./ (2*h);

Ni = numel(id_vec);
Nq = numel(iq_vec);

% pre‑allocate Hessians
Hd.dd = zeros(Ni, Nq); Hd.dq = zeros(Ni, Nq); Hd.qq = zeros(Ni, Nq);
Hq.dd = zeros(Ni, Nq); Hq.dq = zeros(Ni, Nq); Hq.qq = zeros(Ni, Nq);

%% finite differences along id (rows) and iq (cols)
for i = 2:Ni-1
    for j = 2:Nq-1
        % local steps
        hid = id_vec(i+1) - id_vec(i-1);
        hiq = iq_vec(j+1) - iq_vec(j-1);

        % --- φd derivatives ---
        dphi_d_did_fwd = (phi_d_map(i+1,j) - phi_d_map(i,j)) / (id_vec(i+1) - id_vec(i));
        dphi_d_did_bwd = (phi_d_map(i,j)   - phi_d_map(i-1,j)) / (id_vec(i) - id_vec(i-1));
        dphi_d_diq_fwd = (phi_d_map(i,j+1) - phi_d_map(i,j)) / (iq_vec(j+1) - iq_vec(j));
        dphi_d_diq_bwd = (phi_d_map(i,j)   - phi_d_map(i,j-1)) / (iq_vec(j) - iq_vec(j-1));

        % second derivatives
        Hd.dd(i,j) = central(dphi_d_did_fwd, dphi_d_did_bwd, hid/2);
        Hd.qq(i,j) = central(dphi_d_diq_fwd, dphi_d_diq_bwd, hiq/2);
        Hd.dq(i,j) = ( ... % mixed ∂²φd / ∂id∂iq (symmetric)
            (phi_d_map(i+1,j+1) - phi_d_map(i+1,j-1) - phi_d_map(i-1,j+1) + phi_d_map(i-1,j-1)) ) / (4*(id_vec(i+1)-id_vec(i))*(iq_vec(j+1)-iq_vec(j)));

        % --- φq derivatives (same pattern) ---
        dphi_q_did_fwd = (phi_q_map(i+1,j) - phi_q_map(i,j)) / (id_vec(i+1) - id_vec(i));
        dphi_q_did_bwd = (phi_q_map(i,j)   - phi_q_map(i-1,j)) / (id_vec(i) - id_vec(i-1));
        dphi_q_diq_fwd = (phi_q_map(i,j+1) - phi_q_map(i,j)) / (iq_vec(j+1) - iq_vec(j));
        dphi_q_diq_bwd = (phi_q_map(i,j)   - phi_q_map(i,j-1)) / (iq_vec(j) - iq_vec(j-1));

        Hq.dd(i,j) = central(dphi_q_did_fwd, dphi_q_did_bwd, hid/2);
        Hq.qq(i,j) = central(dphi_q_diq_fwd, dphi_q_diq_bwd, hiq/2);
        Hq.dq(i,j) = ( ...
            (phi_q_map(i+1,j+1) - phi_q_map(i+1,j-1) - phi_q_map(i-1,j+1) + phi_q_map(i-1,j-1)) ) / (4*(id_vec(i+1)-id_vec(i))*(iq_vec(j+1)-iq_vec(j)));
    end
end

% edges: replicate nearest interior value (simple choice)
Hd = internal__fill_edges(Hd);
Hq = internal__fill_edges(Hq);
end

%% -----------------------------------------------------------------------------
function [id_ok, iq_ok] = internal__check_uniform_grid(id_vec, iq_vec)
 id_ok = max(abs(diff(diff(id_vec)))) < 1e-9;
 iq_ok = max(abs(diff(diff(iq_vec)))) < 1e-9;
end

function H = internal__fill_edges(H)
 fields = fieldnames(H);
 for k = 1:numel(fields)
     F = H.(fields{k});
     F(1,:)   = F(2,:);
     F(end,:) = F(end-1,:);
     F(:,1)   = F(:,2);
     F(:,end) = F(:,end-1);
     H.(fields{k}) = F;
 end
end
