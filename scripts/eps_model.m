function model = eps_model(spec, design)
% EPS_MODEL  Compute unified EPS surfaces over the full D1-D2 grid.
%
%   MODEL = EPS_MODEL(SPEC, DESIGN) evaluate the paper's three unified
%   surfaces across every feasible (D1, D2) operating point.
%
%   SURFACES  (paper notation, buck mode k >= 1)
%   --------
%     p*   = 4*D2*(1-D2) + 2*D1*(1-D1-2*D2)          unified power
%     G'   = 2*[k*(1-D1) + (2*D1+2*D2-1)]            current stress
%     Mbf' = [k*(1-D1)+(2*D2-1)]^2 / (2*(k+1))       backflow power
%
%   Actual power:  P = PN * p*  where  PN = n*V1*V2 / (8*fsw*L)
%

% ── Unpack -----------------------------------------------------------
V1  = spec.V1;
V2  = spec.V2;
n   = spec.n;
fsw = spec.fsw;
N   = spec.N;
k   = design.k;
L   = design.L;             

% Normalising power 
PN = (n * V1 * V2) / (8 * fsw * L);
fprintf('  Using design L = %.6e H  =>  PN = %.4f W\n', L, PN);
fprintf('  Physical power limit (PN): %.4f W\n', PN);

% ── D1-D2 grid -------------------------------------------------------
d1_vec = linspace(0, 1, N);
d2_vec = linspace(0, 1, N);
[D1_mat, D2_mat] = meshgrid(d1_vec, d2_vec);

% ── Feasibility mask (EPS constraint: D1 + D2 <= 1) ------------------
valid_mask = (D1_mat + D2_mat <= 1);

% ── Unified transmission power p* ------------------------------------
%   p* = 4*D2*(1-D2) + 2*D1*(1-D1-2*D2)
p_star = 4 .* D2_mat .* (1 - D2_mat) + ...
         2 .* D1_mat .* (1 - D1_mat - 2 .* D2_mat);

% Actual power surface
P_mat  = PN .* p_star;

% ── Unified current stress G' ----------------------------------------
%   G' = 2*[k*(1-D1) + (2*D1+2*D2-1)]
G_mat  = 2 .* (k .* (1 - D1_mat) + (2 .* D1_mat + 2 .* D2_mat - 1));

% ── Unified backflow power Mbf' --------------------------------------
%   Mbf' = [k*(1-D1)+(2*D2-1)]^2 / (2*(k+1))

core_term = k .* (1 - D1_mat) + (2 .* D2_mat - 1);
core_term = max(0, core_term);

Mbf_mat = ((core_term)).^2 ./ (2 .* (k + 1));

% ── Apply masks (NaN outside feasible region or negative power) ------
p_star(~valid_mask)  = NaN;
P_mat(~valid_mask)   = NaN;
G_mat(~valid_mask)   = NaN;
Mbf_mat(~valid_mask) = NaN;

p_star(p_star < -1e-12) = NaN;
P_mat(P_mat   < -1e-9)  = NaN;

% ── Verification: D1=0 reduces to TPS --------------------------------
[~, idx_D1_zero] = min(abs(d1_vec - 0));
p_tps_equiv  = 4 .* d2_vec .* (1 - d2_vec);
p_eps_slice  = p_star(:, idx_D1_zero).';
diag_err = max(abs(p_eps_slice - p_tps_equiv), [], 'omitnan');
fprintf('  TPS self-check (D1=0): max |EPS slice - TPS| = %.3e\n', diag_err);

% ── 2-D regulation boundary curves (paper Fig 8b) --------------------
p_min_curve = 2 .* d2_vec .* (1 - d2_vec);
p_max_curve = zeros(size(d2_vec));
idx_lo = d2_vec <  0.5;
idx_hi = d2_vec >= 0.5;
p_max_curve(idx_lo) = 1 - (1 - 2 .* d2_vec(idx_lo)).^2 ./ 2;  % Eq.(21)
p_max_curve(idx_hi) = 4 .* d2_vec(idx_hi) .* (1 - d2_vec(idx_hi)); % Eq.(22)

% ── Pack output struct ------------------------------------------------
model.d1_vec      = d1_vec;
model.d2_vec      = d2_vec;
model.D1_mat      = D1_mat;
model.D2_mat      = D2_mat;
model.valid_mask  = valid_mask;
model.p_star      = p_star;
model.P_mat       = P_mat;
model.G_mat       = G_mat;
model.Mbf_mat     = Mbf_mat;
model.PN          = PN;
model.p_tps_equiv = p_tps_equiv;
model.p_eps_slice = p_eps_slice;
model.p_min_curve = p_min_curve;
model.p_max_curve = p_max_curve;
end
