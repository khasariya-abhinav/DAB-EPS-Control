% =========================================================================
%  DAB / EPS UNIFIED DESIGN PIPELINE  –  README
% =========================================================================
%
%  FILE STRUCTURE
%  ──────────────
%  DAB_EPS_Main.m   ← Entry point: run this file
%  eps_design.m     ← Module 1: open-loop design of the converter and
%                               deriving the values of components and 
%                               other parameters.
%  eps_model.m      ← Module 2: full D1-D2 grid model for power, normalized
%                               power, backpower flow and current stress.
%  eps_lut.m        ← Module 3: LUT(Look-Up Table) generation mapped most
%                               suitable D1 and D2 o the power curve for 
%                               desired power transfer.
%  eps_control.m    ← Module 4: PI control + stability analysis
%  eps_plots.m      ← Module 5: all plots and visualisations to support
%                               study analysis and evaluation.
%
%   p*    = Normalized Power
%   G'    = Normalized current stress
%   Mbf'  = Normalized back power
%  
% =========================================================================
%  DATA FLOW
% =========================================================================
%
%   spec (user-defined)
%    │
%    ├──► eps_design(spec)
%    │         • grid-searches D1, D2 to minimise J = G/F
%    │         • back-calculates  L = n*V1*V2*F / (2*fsw*Pdes)
%    │         • outputs: design struct (D1, D2, L, Ipk, Pbf, Deq (SPS), PN, …)
%    │
%    ├──► eps_model(spec, design)
%    │         • receives design.L 
%    │         • recomputes PN = n*V1*V2 / (8*fsw*L)  
%    │         • builds N×N (D1, D2) grid
%    │         • evaluates p*, G', Mbf' over feasible region
%    │         • outputs: model struct (grids, surfaces, boundary curves)
%    │
%    ├──► eps_lut(spec, design, model)
%    │         • uses model.PN and model.p_star / G_mat / Mbf_mat
%    │         • sweeps P from 10 W to 0.98*PN
%    │         • for each P: finds (D1,D2) minimising G' subject to P=P_req
%    │         • outputs: lut struct (P_array, D1_table, D2_table, …)
%    │
%    ├──► eps_control(spec, design)
%    │         • builds small-signal plant G_vp(s) = [R/(2*V2)] / [(RC/2)s+1]
%    │         • synthesises PI: C_pi(s) = Kp + Ki/s
%    │         • computes gain margin, phase margin, crossover frequencies
%    │         • outputs: ctrl struct (G_vp, C_pi, Lo, T, margins)
%    │
%    └──► eps_plots(spec, design, model, lut, ctrl)
%              • renders 9 figures (see list below)
%              • applies consistent markers across all surfaces:
%                  ★  pentagon  (orange)  = open-loop design point
%                  ◆  diamond   (green)   = TPS equivalent point
%                  ─  black line          = LUT optimal trajectory
%
% =========================================================================
%  FIGURES GENERATED
% =========================================================================
%
%  Fig 1  – Optimal EPS control trajectory D1, D2 vs P_des
%  Fig 2  – Control loop: Bode diagram + closed-loop step response
%  Fig 3  – Actual transmitted power surface P(D1,D2) [W]  (3-D)
%  Fig 4  – Paper Fig 8(a): Unified p*(D1,D2) surface      (3-D)
%  Fig 5  – TPS verification slice at D1 = 0
%  Fig 6  – Paper Fig 8(b): 2-D power-regulation region
%  Fig 7  – Unified current stress surface G'(D1,D2)        (3-D)
%  Fig 8  – Unified backflow power surface Mbf'(D1,D2)      (3-D)
%  Fig 9  – Requested vs achieved power from LUT
%
% =========================================================================
%  SPEC STRUCT FIELDS
% =========================================================================
%
%  spec.V1    Primary DC bus voltage          [V]
%  spec.V2    Secondary DC bus voltage        [V]
%  spec.n     Transformer turns ratio (Np/Ns)
%  spec.fsw   Switching frequency             [Hz]
%  spec.Pdes  Open-loop design power target   [W]
%  spec.R     Load resistance                 [Ω]
%  spec.C     Output filter capacitor         [F]
%  spec.Ilim  Peak-current hard limit (NaN = ignore)
%
% =========================================================================
% TO DO A COMPARATIVE ANALYSIS OF EPS TO SPS 
% =========================================================================
%
% Simply run the compare_script.m once after the pipline is completed.
% It will simulate the respective models of SPS and EPS with the given
% parameters in spec.Pdes in DAB_EPS_Main.m.
%
% =========================================================================
%  HOW TO ADD A NEW OPERATING POINT
% =========================================================================
%
%  Simply change spec.Pdes in DAB_EPS_Main.m and re-run.
%  All modules automatically re-derive L and propagate consistently.
%
% =========================================================================
