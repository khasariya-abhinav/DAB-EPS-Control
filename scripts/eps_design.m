function design = eps_design(spec)
% EPS_DESIGN  Open-loop design of a DAB converter using Extended Phase Shift.
%
%   DESIGN = EPS_DESIGN(SPEC) finds the (D1, D2) operating point that
%   minimises peak inductor current stress (J = G/F) for the demanded
%   power SPEC.Pdes
%
%   INPUT
%     spec  – struct with fields: V1, V2, n, fsw, Pdes, Ilim
%
%   OUTPUT
%     design – struct (see field list at the end of this function)
%
%%    EQUATIONS  (forward EPS, buck mode k >= 1)
%     F(D1,D2)   = D2(1-D2) + 0.5*D1*(1-D1-2*D2)
%     P          = n*V1*V2 / (2*fsw*L) * F
%     G(D1,D2)   = k*(1-D1) + (2*D1+2*D2-1)
%     Ipk        = n*V2/(4*fsw*L) * G
%     L_design   = n*V1*V2*F / (2*fsw*Pdes)
%     Ipk_design = Pdes/(2*V1) * (G/F)
%     X_bf       = k*(1-D1) + (2*D2-1)
%     Pbf        = Pdes * X_bf^2 / (8*(k+1)*F)
%     Deq        = (1 - sqrt(1-4*F)) / 2          [TPS equivalent]

% ── Unpack -----------------------------------------------------------
V1   = spec.V1;
V2   = spec.V2;
n    = spec.n;
fsw  = spec.fsw;
Pdes = spec.Pdes;
Ilim = spec.Ilim;
N    = spec.N;      % grid points per axis (increase for finer resolution)

% ── Buck-mode guard ---------------------------------------------------
k = V1 / (n * V2);
if k <= 1
    error('eps_design: buck mode requires k = V1/(n*V2) > 1. Got k = %.4f.', k);
end

fprintf('  k = V1/(n*V2) = %.4f  [buck mode confirmed]\n', k);
fprintf('  Ts = %.4e s,  Ths = %.4e s\n', 1/fsw, 1/(2*fsw));

% ── Grid search over (D1, D2) ----------------------------------------
D2_grid = linspace(0, 1, N);

best.J  = inf;
best.D1 = NaN;
best.D2 = NaN;
best.F  = NaN;
best.G  = NaN;

for ii = 1:N
    D2      = D2_grid(ii);
    D1_grid = linspace(0, 1 - D2, N);

    F = D2 .* (1 - D2) + 0.5 .* D1_grid .* (1 - D1_grid - 2*D2);
    G = k  .* (1 - D1_grid) + (2 .* D1_grid + 2*D2 - 1);
    J = G ./ F;             % cost: proportional to Ipk for fixed Pdes

    valid = (F > 1e-12) & isfinite(J);
    if ~any(valid), continue; end

    [Jmin, idx] = min(J(valid));
    D1v = D1_grid(valid);
    Fv  = F(valid);
    Gv  = G(valid);

    if Jmin < best.J
        best.J  = Jmin;
        best.D1 = D1v(idx);
        best.D2 = D2;
        best.F  = Fv(idx);
        best.G  = Gv(idx);
    end
end

if isnan(best.D1)
    error('eps_design: no feasible EPS point found. Check specifications.');
end

D1 = best.D1;
D2 = best.D2;
F  = best.F;
G  = best.G;

% ── Derived quantities ------------------------------------------------
L   = n * V1 * V2 * F / (2 * fsw * Pdes);     % required inductance [H]
Ipk = (Pdes / (2 * V1)) * (G / F);            % peak inductor current [A]

X_bf = k * (1 - D1) + (2*D2 - 1);
Pbf  = Pdes * (X_bf^2) / (8 * (k + 1) * F);   % backflow power [W]

% TPS equivalent phase-shift (forward-power branch, D in [0, 0.5])
Deq = (1 - sqrt(max(0, 1 - 4*F))) / 2;

% Normalisation constants
PN = n * V1 * V2 / (8 * fsw * L);              % [W] normalising power
pN = Pdes / PN;                                % dimensionless
IN = PN / V1;                                  % normalising current [A]

% Half-period timing instants
Ths = 1 / (2 * fsw);
timing.t0 = 0;
timing.t1 = D1 * Ths;
timing.t2 = D2 * Ths;
timing.t3 = Ths;
timing.t4 = Ths + D1 * Ths;
timing.t5 = Ths + D2 * Ths;
timing.t6 = 2 * Ths;

% ── Print results -----------------------------------------------------
fprintf('\n  --- EPS design result (minimum-peak-current point) ---\n');
fprintf('  D1 = %.6f\n', D1);
fprintf('  D2 = %.6f\n', D2);
fprintf('  D1 + D2 = %.6f\n', D1 + D2);
fprintf('  Equivalent TPS phase shift Deq = %.6f\n', Deq);

fprintf('\n  --- Derived converter values ---\n');
fprintf('  Required inductance L = %.6e H\n', L);
fprintf('  Peak inductor current Ipk = %.4f A\n', Ipk);
fprintf('  Backflow power Pbf = %.4f W\n', Pbf);

fprintf('\n  --- Normalised quantities ---\n');
fprintf('  PN = %.4f W\n', PN);
fprintf('  p = Pdes/PN = %.4f\n', pN);
fprintf('  IN = PN/V1 = %.4f A\n', IN);

if ~isnan(Ilim)
    fprintf('\n  --- Peak-current limit check ---\n');
    if Ipk <= Ilim
        fprintf('  OK : Ipk (%.4f A) <= Ilim (%.4f A)\n', Ipk, Ilim);
    else
        fprintf('  WARNING : Ipk (%.4f A) > Ilim (%.4f A)\n', Ipk, Ilim);
    end
end

% ── Pack output struct ------------------------------------------------
design.k      = k;
design.D1     = D1;
design.D2     = D2;
design.F      = F;
design.G      = G;
design.J      = best.J;
design.L      = L;        
design.Ipk    = Ipk;
design.Pbf    = Pbf;
design.Deq    = Deq;
design.PN     = PN;
design.pN     = pN;
design.IN     = IN;
design.Pdes   = Pdes;
design.X_bf   = X_bf;
design.timing = timing;
end
