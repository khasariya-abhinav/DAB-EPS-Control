function lut = eps_lut(spec, design, model)
% EPS_LUT  Build an optimal look-up table: demanded power → (D1, D2).
%
%   LUT = EPS_LUT(SPEC, DESIGN, MODEL) sweeps demanded power from a
%   minimum operating point up to 98 % of the physical power limit
%   PN, and for each level selects the (D1, D2) that delivers the
%   requested power while minimising the unified current stress G'.
%
%   INPUT
%     spec   – system spec struct (V1, V2, n, fsw …)
%     design – output of eps_design  (carries design.L, design.PN, …)
%     model  – output of eps_model   (carries grid surfaces)
%
%   OUTPUT
%     lut – struct with vectors indexed by demanded power:
%           P_array, D1_table, D2_table, P_achieved_table,
%           p_star_achieved_table, G_achieved_table, Mbf_achieved_table

% ── Unpack -----------------------------------------------------------
N      = spec.N;
PN     = model.PN;
p_star = model.p_star;
P_mat  = model.P_mat;
G_mat  = model.G_mat;
Mbf_mat= model.Mbf_mat;
D1_mat = model.D1_mat;
D2_mat = model.D2_mat;

% ── Power sweep -------------------------------------------------------
P_max_sweep = 0.90 * PN;          % stay below physical limit
n_pts       = 100;
P_array     = linspace(10, P_max_sweep, n_pts);

% Pre-allocate
D1_table              = zeros(1, n_pts);
D2_table              = zeros(1, n_pts);
P_achieved_table      = zeros(1, n_pts);
p_star_achieved_table = zeros(1, n_pts);
G_achieved_table      = zeros(1, n_pts);
Mbf_achieved_table    = zeros(1, n_pts);

% ── Main LUT loop ----------------------------------------------------
for i = 1:n_pts
    Preq  = P_array(i);
    p_req = Preq / PN;              % target normalized power

    % Distance from every grid point to the target p*
    diffP   = abs(p_star - p_req);
    minDiff = min(diffP(:), [], 'omitnan');

    % Candidate set: points within numerical tolerance of the target

    cand = diffP <= (minDiff + 0.5*(1/(N-1)));

    gCand     = G_mat;
    gCand(~cand) = Inf;             % mask non-candidates

    % Among candidates, pick minimum current stress
    [~, opt_idx] = min(gCand(:));

    D1_table(i)              = D1_mat(opt_idx);
    D2_table(i)              = D2_mat(opt_idx);
    P_achieved_table(i)      = P_mat(opt_idx);
    p_star_achieved_table(i) = p_star(opt_idx);
    G_achieved_table(i)      = G_mat(opt_idx);
    Mbf_achieved_table(i)    = Mbf_mat(opt_idx);
end

% ── Diagnostics -------------------------------------------------------
max_err = max(abs(P_achieved_table - P_array));
fprintf('  LUT: %d entries, P range [%.1f, %.1f] W\n', ...
        n_pts, P_array(1), P_array(end));
fprintf('  Max power tracking error in LUT: %.4f W\n', max_err);

% ── Pack output struct ------------------------------------------------
lut.P_array              = P_array;
lut.D1_table             = D1_table;
lut.D2_table             = D2_table;
lut.P_achieved_table     = P_achieved_table;
lut.p_star_achieved_table= p_star_achieved_table;
lut.G_achieved_table     = G_achieved_table;
lut.Mbf_achieved_table   = Mbf_achieved_table;
end
