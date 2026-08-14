%% 1. Run the EPS Model
fprintf('Simulating EPS Model...\n');
load_system('DAB_EPS_CL_model.slx'); % Put your actual EPS model filename here
out_eps = sim('DAB_EPS_CL_model.slx');

%% 2. Run the SPS Model
fprintf('Simulating SPS Model...\n');
load_system('DAB_SPS_CL_model.slx'); % Put your actual SPS model filename here
out_sps = sim('DAB_SPS_CL_model.slx');

%% ========================================================================
% EPS DATA EXTRACTION
% ========================================================================
% We pull from 'out_eps' now
t_eps   = out_eps.eps_simout.time;
x_eps   = out_eps.eps_simout.data;

Vh1_eps  = x_eps(:,1);
Vh2_eps  = x_eps(:,2);
I_lk_eps = x_eps(:,3);
Pin_eps  = x_eps(:,4);

%% ========================================================================
% SPS DATA EXTRACTION
% ========================================================================
% We pull from 'out_sps' now
t_sps   = out_sps.sps_simout.time; 
x_sps   = out_sps.sps_simout.data; 

Vh1_sps  = x_sps(:,1);
Vh2_sps  = x_sps(:,2);
I_lk_sps = x_sps(:,3);
Pin_sps  = x_sps(:,4);

%% ========================================================================
% REMOVE STARTUP TRANSIENT
% ========================================================================

t_start = 0.07;   % adjust as needed

idx_eps = t_eps > t_start;
idx_sps = t_sps > t_start;

t_eps = t_eps(idx_eps);
Vh1_eps = Vh1_eps(idx_eps);
Vh2_eps = Vh2_eps(idx_eps);
I_lk_eps = I_lk_eps(idx_eps);
Pin_eps = Pin_eps(idx_eps);

t_sps = t_sps(idx_sps);
Vh1_sps = Vh1_sps(idx_sps);
Vh2_sps = Vh2_sps(idx_sps);
I_lk_sps = I_lk_sps(idx_sps);
Pin_sps = Pin_sps(idx_sps);

%% ========================================================================
% CURRENT STRESS ANALYSIS
% ========================================================================

Ipk_eps = max(abs(I_lk_eps));
Ipk_sps = max(abs(I_lk_sps));

Irms_eps = rms(I_lk_eps);
Irms_sps = rms(I_lk_sps);

Ripple_eps = max(I_lk_eps)-min(I_lk_eps);
Ripple_sps = max(I_lk_sps)-min(I_lk_sps);

%% ========================================================================
% POWER ANALYSIS
% ========================================================================

Pavg_eps = mean(Pin_eps);
Pavg_sps = mean(Pin_sps);

%% ========================================================================
% BACKFLOW POWER ANALYSIS
% ========================================================================

% Negative power intervals

Pbf_inst_eps = Pin_eps;
Pbf_inst_eps(Pbf_inst_eps > 0) = 0;

Pbf_inst_sps = Pin_sps;
Pbf_inst_sps(Pbf_inst_sps > 0) = 0;

Pbf_avg_eps = trapz(t_eps,abs(Pbf_inst_eps)) ...
              /(t_eps(end)-t_eps(1));

Pbf_avg_sps = trapz(t_sps,abs(Pbf_inst_sps)) ...
              /(t_sps(end)-t_sps(1));

Pbf_peak_eps = max(abs(Pbf_inst_eps));
Pbf_peak_sps = max(abs(Pbf_inst_sps));

%% ========================================================================
% BACKFLOW ENERGY PER SIMULATION
% ========================================================================

Ebf_eps = trapz(t_eps, abs(Pbf_inst_eps));
Ebf_sps = trapz(t_sps, abs(Pbf_inst_sps));

%% ========================================================================
% IMPROVEMENTS
% ========================================================================

PeakCurrentReduction = ...
    (Ipk_sps - Ipk_eps)/Ipk_sps*100;

RMSCurrentReduction = ...
    (Irms_sps - Irms_eps)/Irms_sps*100;

BackflowReduction = ...
    (Pbf_avg_sps - Pbf_avg_eps)/Pbf_avg_sps*100;

%% ========================================================================
% PRINT RESULTS
% ========================================================================

fprintf('\n================================================\n');
fprintf('                 SPS RESULTS\n');
fprintf('================================================\n');

fprintf('Peak Current      = %.4f A\n',Ipk_sps);
fprintf('RMS Current       = %.4f A\n',Irms_sps);
fprintf('Current Ripple    = %.4f A\n',Ripple_sps);
fprintf('Average Power     = %.4f W\n',Pavg_sps);
fprintf('Avg Backflow Pow. = %.4f W\n',Pbf_avg_sps);
fprintf('Peak Backflow Pow.= %.4f W\n',Pbf_peak_sps);
fprintf('Backflow Energy   = %.6f J\n',Ebf_sps);

fprintf('\n================================================\n');
fprintf('                 EPS RESULTS\n');
fprintf('================================================\n');

fprintf('Peak Current      = %.4f A\n',Ipk_eps);
fprintf('RMS Current       = %.4f A\n',Irms_eps);
fprintf('Current Ripple    = %.4f A\n',Ripple_eps);
fprintf('Average Power     = %.4f W\n',Pavg_eps);
fprintf('Avg Backflow Pow. = %.4f W\n',Pbf_avg_eps);
fprintf('Peak Backflow Pow.= %.4f W\n',Pbf_peak_eps);
fprintf('Backflow Energy   = %.6f J\n',Ebf_eps);

fprintf('\n================================================\n');
fprintf('             EPS IMPROVEMENT\n');
fprintf('================================================\n');

fprintf('Peak Current Reduction = %.2f %%\n',PeakCurrentReduction);
fprintf('RMS Current Reduction  = %.2f %%\n',RMSCurrentReduction);
fprintf('Backflow Reduction     = %.2f %%\n',BackflowReduction);

%% ========================================================================
% PLOT 1 : LEAKAGE CURRENT
% ========================================================================

figure('Name','Leakage Current Comparison');

ax(1)=subplot(2,1,1);
plot(t_sps,I_lk_sps,'LineWidth',1.2)
grid on
title('SPS Leakage Inductor Current')

ax(2)=subplot(2,1,2);
plot(t_eps,I_lk_eps,'LineWidth',1.2)
grid on
title('EPS Leakage Inductor Current')

linkaxes(ax,'x')

%% ========================================================================
% PLOT 2 : INSTANTANEOUS POWER
% ========================================================================

figure('Name','Instantaneous Power');

ax(1)=subplot(2,1,1);
plot(t_sps,Pin_sps,'LineWidth',1.2)
grid on
title('SPS Instantaneous Power')
ylabel('Power (W)')

ax(2)=subplot(2,1,2);
plot(t_eps,Pin_eps,'LineWidth',1.2)
grid on
title('EPS Instantaneous Power')
ylabel('Power (W)')
xlabel('Time (s)')

linkaxes(ax,'x')

%% ========================================================================
% PLOT 3 : BACKFLOW POWER
% ========================================================================

figure('Name','Backflow Power');

ax(1)=subplot(2,1,1);
plot(t_sps,Pbf_inst_sps,'LineWidth',1.2)
grid on
title('SPS Backflow Power')
ylabel('Power (W)')

ax(2)=subplot(2,1,2);
plot(t_eps,Pbf_inst_eps,'LineWidth',1.2)
grid on
title('EPS Backflow Power')
ylabel('Power (W)')
xlabel('Time (s)')

linkaxes(ax,'x')

%% ========================================================================
% PLOT 4 : PRIMARY BRIDGE VOLTAGE
% ========================================================================

figure('Name','Primary Bridge Voltage');

ax(1)=subplot(2,1,1);
plot(t_sps,Vh1_sps,'LineWidth',1)
grid on
title('SPS Primary Bridge Voltage')
ylabel('V_{h1}')

ax(2)=subplot(2,1,2);
plot(t_eps,Vh1_eps,'LineWidth',1)
grid on
title('EPS Primary Bridge Voltage')
ylabel('V_{h1}')
xlabel('Time (s)')

linkaxes(ax,'x')