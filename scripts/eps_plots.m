function eps_plots(spec, design, model, lut, ctrl)
% EPS_PLOTS  Generate all plots for the DAB/EPS design.
%
%   EPS_PLOTS(SPEC, DESIGN, MODEL, LUT, CTRL) produces:
%     Fig 1  – Optimal EPS control trajectory (D1, D2 vs P)
%     Fig 2  – Control loop: Bode + step response
%     Fig 3  – Actual transmitted power surface  (3-D)
%     Fig 4  – Unified p* surface (3-D)
%     Fig 5  – TPS verification at D1 = 0
%     Fig 6  – 2-D power-regulation region
%     Fig 7  – Unified current stress surface   (3-D)
%     Fig 8  – Unified backflow power surface   (3-D)
%     Fig 9  – Requested vs achieved power from LUT
%
%   MARKERS (consistent across all relevant figures)
%     ★  Open-loop design point  (D1_OL, D2_OL)
%     ◆  Equivalent TPS point    (0, Deq)
%     ─  LUT optimal trajectory

% ── Shared style constants -------------------------------------------
OL_CLR  = [0.85 0.33 0.10];    % burnt orange – open-loop design point
TPS_CLR = [0.47 0.67 0.19];    % green        – TPS equivalent point
LUT_CLR = [0.00 0.00 0.00];    % black        – LUT trajectory
SURF_ALPHA = 0.88;
MS = 12;                        % marker size
LW = 2.5;                       % line width

% ── Unpack for convenience -------------------------------------------
D1_OL  = design.D1;
D2_OL  = design.D2;
Deq    = design.Deq;
Pdes   = design.Pdes;
PN     = model.PN;

D1_mat = model.D1_mat;
D2_mat = model.D2_mat;
p_star = model.p_star;
P_mat  = model.P_mat;
G_mat  = model.G_mat;
Mbf_mat= model.Mbf_mat;
d1_vec = model.d1_vec;
d2_vec = model.d2_vec;

P_array              = lut.P_array;
D1_table             = lut.D1_table;
D2_table             = lut.D2_table;
P_achieved_table     = lut.P_achieved_table;
p_star_achieved_table= lut.p_star_achieved_table;
G_achieved_table     = lut.G_achieved_table;
Mbf_achieved_table   = lut.Mbf_achieved_table;

Lo = ctrl.Lo;
T  = ctrl.T;

% ── Helper: open-loop marker values on surfaces ----------------------
% Find closest LUT index to Pdes (for 3-D surface markers)
[~, ol_lut_idx] = min(abs(P_array - Pdes));
P_OL_surf    = P_achieved_table(ol_lut_idx);
pstar_OL     = design.pN;     % normalized design power
G_OL         = design.G * 2;  % paper G' = 2*G_paper_raw (consistent with G_mat definition)
Mbf_OL       = design.X_bf^2 / (2 * (design.k + 1));

% TPS point: D1=0, D2=Deq  (p* = 4*Deq*(1-Deq))
pstar_TPS = 4 * Deq * (1 - Deq);

% ═══════════════════════════════════════════════════════════════════════
%  FIG 1 – Optimal EPS Control Trajectory
% ═══════════════════════════════════════════════════════════════════════
figure('Name','Fig 1: Optimal EPS Control Trajectory','Position',[60 60 920 500]);

yyaxis left
plot(P_array, D1_table, 'b-', 'LineWidth', LW, 'DisplayName', 'D_1  (LUT)');
hold on;
plot(Pdes, D1_OL, 'p', 'MarkerSize', MS+2, ...
     'Color', OL_CLR, 'MarkerFaceColor', OL_CLR, ...
     'DisplayName', sprintf('D_1^{OL} = %.3f (design pt)', D1_OL));
ylabel('D_1');

yyaxis right
plot(P_array, D2_table, 'r-', 'LineWidth', LW, 'DisplayName', 'D_2  (LUT)');
plot(Pdes, D2_OL, 's', 'MarkerSize', MS, ...
     'Color', OL_CLR, 'MarkerFaceColor', OL_CLR, ...
     'DisplayName', sprintf('D_2^{OL} = %.3f (design pt)', D2_OL));
plot(Pdes, Deq, 'd', 'MarkerSize', MS, ...
     'Color', TPS_CLR, 'MarkerFaceColor', TPS_CLR, ...
     'DisplayName', sprintf('D_{eq} = %.3f (TPS equiv)', Deq));
ylabel('D_2');

grid on;
xlabel('Demanded Power P_{des} (W)');
title(sprintf('Optimal EPS Control Trajectory  (L = %.4g H)', design.L));
legend('Location', 'best');
xline(Pdes, '--k', 'P_{des}', 'LabelVerticalAlignment','bottom');

% ═══════════════════════════════════════════════════════════════════════
%  FIG 2 – Control Loop Analysis
% ═══════════════════════════════════════════════════════════════════════
figure('Name','Fig 2: Control Loop Analysis','Position',[60 60 1050 480]);

subplot(1,2,1);
margin(Lo);
grid on;
title('Open-Loop Bode Plot');

subplot(1,2,2);
step(T, 0.005);
grid on;
title('Closed-Loop Step Response');
ylabel('Normalised Output');
xlabel('Time (s)');

% ═══════════════════════════════════════════════════════════════════════
%  FIG 3 – Actual Transmitted Power Surface (3-D)
% ═══════════════════════════════════════════════════════════════════════
figure('Name','Fig 3: Actual Transmitted Power Surface','Position',[80 80 920 640]);

surf(D2_mat, D1_mat, P_mat, 'EdgeColor','none','FaceAlpha',SURF_ALPHA);
colormap turbo; colorbar;
hold on;

% LUT trajectory
plot3(D2_table, D1_table, P_achieved_table, '-', ...
      'Color', LUT_CLR, 'LineWidth', LW, 'DisplayName', 'LUT optimal trajectory');
plot3(D2_table(1), D1_table(1), P_achieved_table(1), ...
      'go', 'MarkerSize', MS, 'LineWidth', 2, 'DisplayName', 'LUT P_{min}');
plot3(D2_table(end), D1_table(end), P_achieved_table(end), ...
      'ro', 'MarkerSize', MS, 'LineWidth', 2, 'DisplayName', 'LUT P_{max}');

% Open-loop design point
plot3(D2_OL, D1_OL, P_OL_surf, 'p', ...
      'MarkerSize', MS+4, 'Color', OL_CLR, 'MarkerFaceColor', OL_CLR, ...
      'DisplayName', sprintf('OL design pt (D1=%.3f, D2=%.3f)', D1_OL, D2_OL));

% TPS equivalent point (D1=0, D2=Deq)
P_TPS = pstar_TPS * PN;
plot3(Deq, 0, P_TPS, 'd', ...
      'MarkerSize', MS, 'Color', TPS_CLR, 'MarkerFaceColor', TPS_CLR, ...
      'DisplayName', sprintf('TPS equiv (D1=0, D2=Deq=%.3f)', Deq));

grid on; box on;
set(gca, 'XDir', 'reverse', 'YDir', 'reverse');   
xlabel('D_2'); ylabel('D_1'); zlabel('Transferred Power (W)');
title('Actual Transmitted Power Surface');
legend('Location','best');
view(135,30);

% ═══════════════════════════════════════════════════════════════════════
%  FIG 4 – Paper Fig 8(a): Unified p* Surface (3-D)
% ═══════════════════════════════════════════════════════════════════════
figure('Name','Fig 4: Unified p* Surface','Position',[80 80 920 640]);

surf(D2_mat, D1_mat, p_star, 'EdgeColor','none','FaceAlpha',SURF_ALPHA);
colormap turbo; colorbar;
hold on;

% LUT trajectory
plot3(D2_table, D1_table, p_star_achieved_table, '-', ...
      'Color', LUT_CLR, 'LineWidth', LW, 'DisplayName', 'LUT optimal trajectory');
plot3(D2_table(1), D1_table(1), p_star_achieved_table(1), ...
      'go', 'MarkerSize', MS, 'LineWidth', 2, 'DisplayName', 'LUT P_{min}');
plot3(D2_table(end), D1_table(end), p_star_achieved_table(end), ...
      'ro', 'MarkerSize', MS, 'LineWidth', 2, 'DisplayName', 'LUT P_{max}');

% Open-loop design point
plot3(D2_OL, D1_OL, pstar_OL, 'p', ...
      'MarkerSize', MS+4, 'Color', OL_CLR, 'MarkerFaceColor', OL_CLR, ...
      'DisplayName', sprintf('OL design pt (D1=%.3f, D2=%.3f)', D1_OL, D2_OL));

% TPS equivalent point
plot3(Deq, 0, pstar_TPS, 'd', ...
      'MarkerSize', MS, 'Color', TPS_CLR, 'MarkerFaceColor', TPS_CLR, ...
      'DisplayName', sprintf('TPS equiv (D2=Deq=%.3f)', Deq));

grid on; box on;
set(gca, 'XDir', 'reverse', 'YDir', 'reverse');   
xlabel('D_2'); ylabel('D_1'); zlabel('Unified Transmission Power p^*');
title('Paper Fig. 8(a): Unified Transmission Power Surface');
legend('Location','best');
view(135,30);

% ═══════════════════════════════════════════════════════════════════════
%  FIG 5 – TPS Verification Slice (D1 = 0)
% ═══════════════════════════════════════════════════════════════════════
figure('Name','Fig 5: TPS Verification (D1 = 0)','Position',[100 100 850 480]);

plot(d2_vec, model.p_eps_slice, 'b-', 'LineWidth', LW, ...
     'DisplayName', 'EPS slice at D_1 = 0');
hold on;
plot(d2_vec, model.p_tps_equiv, 'r--', 'LineWidth', LW, ...
     'DisplayName', 'TPS curve: 4D_2(1-D_2)');

% TPS equivalent marker
plot(Deq, pstar_TPS, 'd', 'MarkerSize', MS, ...
     'Color', TPS_CLR, 'MarkerFaceColor', TPS_CLR, ...
     'DisplayName', sprintf('TPS equiv D_{eq}=%.3f', Deq));

% Open-loop design point (projected onto D1=0 slice for reference)
plot(D2_OL, 4*D2_OL*(1-D2_OL), 'p', 'MarkerSize', MS, ...
     'Color', OL_CLR, 'MarkerFaceColor', OL_CLR, ...
     'DisplayName', sprintf('D2_{OL}=%.3f (TPS projection)', D2_OL));

grid on;
xlabel('D_2');
ylabel('Unified Power p^*');
title('TPS Verification: D_1 = 0 Reduces to TPS Formula');
legend('Location','best');

% ═══════════════════════════════════════════════════════════════════════
%  FIG 6 – Paper Fig 8(b): 2-D Power-Regulation Region
% ═══════════════════════════════════════════════════════════════════════
figure('Name','Fig 6: 2-D Power Regulation Region','Position',[100 100 960 620]);

% Shaded feasible EPS region
fill([d2_vec, fliplr(d2_vec)], ...
     [model.p_min_curve, fliplr(model.p_max_curve)], ...
     [0.85 0.92 1], 'EdgeColor','none', 'FaceAlpha',0.8, ...
     'DisplayName', 'EPS feasible region');
hold on;

plot(d2_vec, model.p_min_curve, 'b-',  'LineWidth', LW, 'DisplayName', 'p^*_{min}');
plot(d2_vec, model.p_max_curve, 'r-',  'LineWidth', LW, 'DisplayName', 'p^*_{max}');
plot(d2_vec, model.p_tps_equiv, 'k--', 'LineWidth', LW, 'DisplayName', 'TPS curve');

% LUT optimal trajectory (in normalised power)
plot(D2_table, p_star_achieved_table, '-', ...
     'Color', LUT_CLR, 'LineWidth', LW, 'DisplayName', 'LUT optimal path');

% Open-loop design point
plot(D2_OL, pstar_OL, 'p', 'MarkerSize', MS+2, ...
     'Color', OL_CLR, 'MarkerFaceColor', OL_CLR, ...
     'DisplayName', sprintf('OL design pt (D2=%.3f, p^*=%.3f)', D2_OL, pstar_OL));

% TPS equivalent point
plot(Deq, pstar_TPS, 'd', 'MarkerSize', MS, ...
     'Color', TPS_CLR, 'MarkerFaceColor', TPS_CLR, ...
     'DisplayName', sprintf('TPS equiv (D_{eq}=%.3f)', Deq));

grid on; box on;
xlabel('D_2');
ylabel('Unified Transmission Power p^*');
title('Paper Fig. 8(b): 2-D Transmission-Power Regulation Region');
legend('Location','best');
ylim([-1.05 1.05]);

% ═══════════════════════════════════════════════════════════════════════
%  FIG 7 – Unified Current Stress Surface (3-D)
% ═══════════════════════════════════════════════════════════════════════
figure('Name','Fig 7: Unified Current Stress Surface','Position',[120 120 920 640]);

surf(D2_mat, D1_mat, G_mat, 'EdgeColor','none','FaceAlpha',SURF_ALPHA);
colormap turbo; colorbar;
hold on;

% LUT trajectory
plot3(D2_table, D1_table, G_achieved_table, '-', ...
      'Color', LUT_CLR, 'LineWidth', LW, 'DisplayName', 'LUT optimal trajectory');

% Open-loop design point (G' = 2*[k*(1-D1)+(2*D1+2*D2-1)])
k = design.k;
G_OL_surf = 2 * (k*(1-D1_OL) + (2*D1_OL + 2*D2_OL - 1));
plot3(D2_OL, D1_OL, G_OL_surf, 'p', ...
      'MarkerSize', MS+4, 'Color', OL_CLR, 'MarkerFaceColor', OL_CLR, ...
      'DisplayName', sprintf('OL design pt G''=%.3f', G_OL_surf));

% TPS equivalent (D1=0, D2=Deq)
G_TPS_surf = 2 * (k*1 + (2*Deq - 1));
plot3(Deq, 0, G_TPS_surf, 'd', ...
      'MarkerSize', MS, 'Color', TPS_CLR, 'MarkerFaceColor', TPS_CLR, ...
      'DisplayName', sprintf('TPS equiv G''=%.3f', G_TPS_surf));

grid on; box on;
set(gca, 'XDir', 'reverse', 'YDir', 'reverse');   
xlabel('D_2'); ylabel('D_1'); zlabel('Unified Current Stress G''');
title('Unified Current Stress Surface and Optimal Trajectory');
legend('Location','best');
view(135,30);

% ═══════════════════════════════════════════════════════════════════════
%  FIG 8 – Unified Backflow Power Surface (3-D)
% ═══════════════════════════════════════════════════════════════════════
figure('Name','Fig 8: Unified Backflow Power Surface','Position',[140 140 920 640]);

surf(D2_mat, D1_mat, Mbf_mat, 'EdgeColor','none','FaceAlpha',SURF_ALPHA);
colormap turbo; colorbar;
hold on;

plot3(D2_table, D1_table, Mbf_achieved_table, '-', ...
      'Color', LUT_CLR, 'LineWidth', LW, 'DisplayName', 'LUT optimal trajectory');

% Open-loop design point
Mbf_OL_surf = (k*(1-D1_OL) + (2*D2_OL-1))^2 / (2*(k+1));
plot3(D2_OL, D1_OL, Mbf_OL_surf, 'p', ...
      'MarkerSize', MS+4, 'Color', OL_CLR, 'MarkerFaceColor', OL_CLR, ...
      'DisplayName', sprintf('OL design pt M_{bf}''=%.3f', Mbf_OL_surf));

% TPS equivalent
Mbf_TPS_surf = (k*(1-0) + (2*Deq-1))^2 / (2*(k+1));
plot3(Deq, 0, Mbf_TPS_surf, 'd', ...
      'MarkerSize', MS, 'Color', TPS_CLR, 'MarkerFaceColor', TPS_CLR, ...
      'DisplayName', sprintf('TPS equiv M_{bf}''=%.3f', Mbf_TPS_surf));

grid on; box on;
set(gca, 'XDir', 'reverse', 'YDir', 'reverse');   
xlabel('D_2'); ylabel('D_1'); zlabel('Unified Backflow Power M_{bf}''');
title('Unified Backflow Power Surface and Optimal Trajectory');
legend('Location','best');
view(135,30);

% ═══════════════════════════════════════════════════════════════════════
%  FIG 9 – Requested vs Achieved Power from LUT
% ═══════════════════════════════════════════════════════════════════════
figure('Name','Fig 9: Requested vs Achieved Power','Position',[140 140 900 440]);

plot(P_array, P_array, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Requested');
hold on;
plot(P_array, P_achieved_table, 'r-', 'LineWidth', LW, 'DisplayName', 'Achieved');
xline(Pdes, '--', 'Color', OL_CLR, 'LineWidth', 1.5, ...
      'Label', sprintf('P_{des}=%.0f W', Pdes), 'LabelVerticalAlignment','bottom', ...
      'DisplayName', 'Design P_{des}');

grid on;
xlabel('Demanded Power (W)');
ylabel('Power (W)');
title('Requested vs Achieved Power from LUT');
legend('Location','best');

fprintf('\n  All figures generated.\n');
end
