function ctrl = eps_control(spec, design)
% EPS_CONTROL  Small-signal plant and outer PI loop design for the DAB.
%
%   CTRL = EPS_CONTROL(SPEC, DESIGN) builds the linearised plant
%   G_vp(s) around the output voltage and synthesises a PI controller.
%   Stability margins are computed and printed.
%
%   Small-signal model (first-order approximation):
%     G_vp(s) = [R / (2*V2)] / [(R*C/2)*s + 1]
%
%   PI controller (pre-tuned gains, identical to original script):
%     C_pi(s) = Kp + Ki/s
%
%   INPUT
%     spec   – system spec struct  (V2, R, C)
%     design – output of eps_design (used for reporting context)
%
%   OUTPUT
%     ctrl – struct with fields:
%              G_vp, C_pi, Lo, T, Kp, Ki, Gm, Pm, Wcg, Wcp

% ── Unpack -----------------------------------------------------------
V2 = spec.V2;
R  = spec.R;
C  = spec.C;

% ── Small-signal plant -----------------------------------------------
s    = tf('s');
G_vp = (R / (2 * V2)) / (((R * C) / 2) * s + 1);

fprintf('  --- Plant Transfer Function ---\n');
disp(G_vp);

% ── PI controller ----------------------------------------------------

Kp   = 10;
Ki   = 54441;

C_pi = Kp + (Ki / s);

fprintf('  PI gains:  Kp = %.4f,  Ki = %.4f\n', Kp, Ki);

% ── Open- and closed-loop TFs ----------------------------------------
Lo = C_pi * G_vp;
T  = feedback(Lo, 1);

% ── Stability margins -------------------------------------------------
[Gm, Pm, Wcg, Wcp] = margin(Lo);

fprintf('\n  --- Stability Margins ---\n');
if isinf(Gm)
    fprintf('  Gain Margin  = Inf dB\n');
else
    fprintf('  Gain Margin  = %.2f dB\n', 20*log10(Gm));
end
fprintf('  Phase Margin = %.2f degrees\n', Pm);
fprintf('  Gain crossover frequency   = %.2f Hz\n', Wcp / (2*pi));
fprintf('  Phase crossover frequency  = %.2f Hz\n', Wcg / (2*pi));

% ── Pack output struct ------------------------------------------------
ctrl.G_vp = G_vp;
ctrl.C_pi = C_pi;
ctrl.Lo   = Lo;
ctrl.T    = T;
ctrl.Kp   = Kp;
ctrl.Ki   = Ki;
ctrl.Gm   = Gm;
ctrl.Pm   = Pm;
ctrl.Wcg  = Wcg;
ctrl.Wcp  = Wcp;
end
