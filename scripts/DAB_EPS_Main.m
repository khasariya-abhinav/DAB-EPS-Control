clc; clear; close all;

%% ========================================================================
% Project : DAB EPS Control without ZVS
%
% File : DAB_CL_Pure_R.m
%
% Description:
% MATLAB script for parameter calculation of the SPS controlled
% Dual Active Bridge converter.
%
% Author : Abhinav Khasariya
% =========================================================================
% =========================================================================
%  DAB / EPS Unified Design Pipeline  –  Main Entry Point
%  Buck mode only: k = V1/(n*V2) > 1
%
%  PIPELINE
%  --------
%  1. eps_design   → optimal (D1,D2,L) from J=G/F minimisation
%  2. eps_model    → p*, G', Mbf' over the full D1-D2 grid
%  3. eps_lut      → power → (D1,D2) look-up table (minimising current stress)
%  4. eps_control  → small-signal plant + PI design + stability margins
%  5. eps_plots    → all power and stress surfaces and relevent 2-D curves
% =========================================================================

%% ─── Converter / Plant Specifications ────────────────────────────────────
spec.V1   = 220;        % Primary DC bus voltage          [V]
spec.V2   = 80;         % Secondary DC bus voltage        [V]

N1 = 220;           % Define primary winding voltage
N2 = 110;           % Define secondary winding voltage      (For turn ratio n)

spec.n    = N1/N2;      % Transformer turns ratio  (Np/Ns)   
spec.fsw  = 50000;      % Switching frequency             [Hz]
spec.Pdes = 1000;       % Open-loop design power target   [W]
spec.R    = 6;          % Load resistance                 [Ω]
spec.C    = 2.2e-4;     % Output filter capacitor         [F]
spec.Coss = 100e-12;    % Mosfets parasitic capacitances  [F]
spec.Ilim = NaN;        % Peak-current hard limit (NaN = ignore)
spec.N = 500;           % Grid size 500*500 (increase for higher resolution)

%% ─── 1. Open-Loop Design ─────────────────────────────────────────────────
fprintf('=== STAGE 1 : Open-Loop EPS Design ===\n');
design = eps_design(spec);

%% ─── 2. Full D1-D2 Model ─────────────────────────────────────────────────
fprintf('\n=== STAGE 2 : D1-D2 Grid Model ===\n');
model = eps_model(spec, design);

%% ─── 3. LUT Generation ───────────────────────────────────────────────────
fprintf('\n=== STAGE 3 : LUT Generation ===\n');
lut = eps_lut(spec, design, model);

%% ─── 4. PI Control Design ────────────────────────────────────────────────
fprintf('\n=== STAGE 4 : PI Control Design ===\n');
ctrl = eps_control(spec, design);

%% ─── 5. Visualisation ────────────────────────────────────────────────────
fprintf('\n=== STAGE 5 : Visualisation ===\n');
eps_plots(spec, design, model, lut, ctrl);

fprintf('\n=== Pipeline complete. ===\n');
