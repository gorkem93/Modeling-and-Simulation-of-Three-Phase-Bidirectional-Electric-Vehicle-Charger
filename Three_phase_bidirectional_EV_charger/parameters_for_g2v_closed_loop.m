% ****************************************************
%% PARAMETERS FOR G2V MODE OF OPERATION %%
% ****************************************************
indicator = 1;
%% AC-DC converter ratings
Vm = 325;    % Peak value of phase voltages
Vdc = 800;  % DC bus voltage
fline = 50;   % Line frequency
fsw_ac = 10e3;  % Switching freq
Ls = 1e-3;  % Line inductance
Rs = 0.5;    % Internal resistance of line inductor
Co = 2e-3;  % DC bus capacitance
Lg = 1e-3;  % Grid side filter inductance
Li = 1.6e-3;  % Converter side filter inductance
Cf = 300e-3;  % Filter capacitance
Rd = 0.5;   % Filter damping resistance

Lt = Ls+Lg+Li;  % Total inductance at grid side
Ts = Lt/Rs;  % Time constant of line inductor

Tsw_ac = 1/fsw_ac;  % Switching period
Tsamp_ac = Tsw_ac/2; % Sampling period
T1 = Tsamp_ac/10;     % Time constant of voltage sensor 
                      %(voltage sensor delay)
T2 = Tsamp_ac/10; % Time constant of current sensor 
                  % (current sensor delay)

G = 1;  % Converter gain
K1 = 1  % Gain of voltage sensor
K2 = 1; % Gain of current sensor
Tsig = T2 + Tsamp_ac % Effective delay for current control loop
Tdel = T1 + 2*Tsig   % Effective delay for voltage control loop

Ron_ac = 10e-3; % On-state resistance of transistors in AC-DC 
                % PWM converter
Von_ac = 0.7;   % Forward voltage drop of internal diode for 
                % transistors in AC-DC PWM converter

% PLL parameters %
alfa_pll=sqrt(10); % Constant ratio of Kp and Ki values of PI 
                   % controller for PLL loop

Kpll=(1/Vm)*(1/(alfa_pll*Tsamp_ac)) % Kp value of PLL
Tpll=alfa_pll^2*Tsamp_ac            % PLL time constant
Kipll=Kpll/Tpll              % Ki value of PLL

%Compensated open PLL
s= tf('s')
Gol_pll = (Vm*Kipll*(Tpll*s+1))/(s^2*(1+s*Tsamp_ac))
fc_pll = (1/(alfa_pll*Tsamp_ac))/(2*pi)    % Cross-over freq. of PLL


% Current controller parameters %
Kc = (Rs*Ts)/(2*G*K2*Tsig)  
Kpc = Kc;             % Kp value of current controller

Tc = Ts               % Current controller time constant
Kic = Kpc/Tc          % Ki value of current controller

wn = 1/(sqrt(2)*Tsig)  % Current loop bandwidth


% Voltage controller parameters
K = (3/2)*(Vm/Vdc) % Ratio between DC bus current and q component 
                   % of line current (K = I_dc/i_sqe)

alfa_volt = sqrt(10);  % Constant ratio of Kp and Ki values of PI 
                       % controller for voltage loop
Tv_ac = alfa_volt^2*Tdel      % Voltage controller time constant
Kv = (Co*K2)/(K1*K*alfa_volt*Tdel) 

Kpv = Kv                  % Kp value of voltage controller
Kiv = Kpv/Tv_ac              % Ki value of voltage controller

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Idc_max = 10
Iqe_max = -Idc_max/K
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Closed current loop
Gc = (Kc*(1 + s*Tc))/(s*Tc)
Gdelay = G/(1+s*Tsamp_ac)
Gind = (1/Rs)/(1+s*Ts)
Gc_sense = K2/(1+s*T2)
Gcl_c = (Gc*Gdelay*Gind)/(1 + (Gc*Gdelay*Gind*Gc_sense))
fn = (1/(sqrt(2)*Tsig))/(2*pi)      % Current loop bandwidth
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Open voltage loop
Ke = (Kv*K*K1)/K2
Gol_v = (Ke*(1+s*Tv_ac))/(s^2*Tv_ac*Co*(1+s*Tdel)) 
fc_v = (1/(alfa_volt*Tdel))/(2*pi)% Cross-over freq. of voltage 
                                  % loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Closed voltage loop
Gv_sense = K1/(1+s*T1)
Gcl_v = (Gol_v/Gv_sense)/(1+Gol_v) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1)
bode(Gcl_c)
legend('Inner closed current loop gain')

figure(2)
bode(Gol_v)
legend('Outer open voltage loop gain')

figure(3)
bode(Gcl_v)
legend('Closed voltage loop gain')

%% DC-DC converter ratings (buck mode)
Vg=800;             % Input voltage
V=500;              % Output voltage

P=5000;            % Output power
P_light = 500

Iload=P/V;          % Load current
Iload_light = P_light/V;

R=V/Iload;          % Load resistance
R_light = V/Iload_light;

fsw_dc=100e3;       % Switching frequency
Tsw_dc=1/fsw_dc;    % Switching period

%Inductor
L=250e-6;            % Inductance
RL=10e-3;            % Internal resistance

%Capacitor
C=33e-6;             % Capacitance
Resr=6e-3;           % ESR value

%Mosfet
Rdson=8e-3;          % On-state resistance
Vf=0.78;             % Anti-paralel diode forward voltage

VM = 4;             % Peak value of PWM sawtooth wave

Rf = 0.25;          % Current sensing resistance
H = 0.0075;         % Voltage sensing gain

Vref = H*V;

Tsamp_dc = Tsw_dc/2;    % Sampling delay

%% DC analysis

% Duty cycle
D=(V/Vg)*(1+((RL+Rdson)/R))     % Duty cycle value

% Inductor current
I = V/R

% Efficiency
nd = 1/(1+((RL+Rdson)/R))

%Inductor ripple current
I_ripple=(((Rdson+RL)*I+V)*(1-D)*Tsw_dc)/(2*L)

%Output voltage ripple (effect of "Resr" is neglected)
V_ripple=(I_ripple*Tsw_dc)/(8*C)

%% Small signal AC analysis
s=tf('s')

Re=RL+Rdson

%Gvd - duty cycle-to-output voltage TF
Gvd=((Vg*R)/(Re+R))*((1+s*C*Resr)/(((L*C*(R+Resr))/(Re+R))*s^2+((R*Re*C+Resr*Re*C+Resr*R*C+L)/(Re+R))*s+1))

Gvd_light = ((Vg*R_light)/(Re+R_light))*((1+s*C*Resr)/(((L*C*(R_light+Resr))/(Re+R_light))*s^2+((R_light*Re*C+Resr*Re*C+Resr*R_light*C+L)/(Re+R_light))*s+1))

%Gvi - inductor current-to-output TF
Gvi=(R*(1+s*C*Resr))/(1+s*C*(R+Resr))

Gvi_light = (R_light*(1+s*C*Resr))/(1+s*C*(R_light+Resr))

%Gid - duty cycle-to-inductor current TF
Gid=Gvd/Gvi

Gid_light = Gvd_light/Gvi_light

%Gvg - line-to-output voltage TF
Gvg=((D*R)/(Re+R))*((1+s*C*Resr)/(((L*C*(R+Resr))/(Re+R))*s^2+((R*Re*C+Resr*Re*C+Resr*R*C+L)/(Re+R))*s+1))

%Gig - line-to-inductor current TF
Gig=Gvg/Gvi

%Zout - output impedance TF
Zout=(-(R*Re)/(Re+R))*(1+s*(L/Re))*((1+s*C*Resr)/(((L*C*(R+Resr))/(Re+R))*s^2+((R*Re*C+Resr*Re*C+Resr*R*C+L)/(Re+R))*s+1))

%Giz - load-to-inductor current TF
Giz=(R/(Re+R))*((1+s*C*Resr)/(((L*C*(R+Resr))/(Re+R))*s^2+((R*Re*C+Resr*Re*C+Resr*R*C+L)/(Re+R))*s+1))


%% Compensator design

% Sampling delay TF
Hsamp = 1/(1+s*Tsamp_dc)

% Inner current loop design

Tiu = Rf*(1/VM)*Hsamp*Gid;  % Uncompensated current loop gain
Tiu_light = Rf*(1/VM)*Hsamp*Gid_light

Tiu_fw = Tiu/Rf;            % Forward uncomp. current loop gain
Kpc_dc = 0.3;               % Kp of current controller
Kic_dc = 6749.8;            % Ki of current controller
Gci = Kpc_dc+Kic_dc/s;  % Current controller

Ti = Tiu*Gci;               % Compensated current loop gain
Ti_light = Tiu_light*Gci;   % Compensated current loop gain


Ti_cl = (1/Rf)*(Ti/(1+Ti));    % Inner closed loop gain
Gig_cl = Gig/(1+Ti);            % Closed-loop line-to-current gain
Giz_cl = Giz/(1+Ti);            % Closed-loop load-to-current gain

figure(4)
bode(Ti)
hold on
bode(Ti_light)
legend('5kW','500W')

figure(5)
bode(Ti_cl)
legend('Inner closed loop gain')


figure(6)
bode(Giz_cl)
legend('Closed-loop load-to-current gain')



% Outer voltage loop design

Gvc = (1/Rf)*(Gvd/Gid)*(Ti/(1+Ti))  % control-to-output 
                                    % voltage transfer 
                                    % function
Gvc_light = (1/Rf)*(Gvd_light/Gid_light)*(Ti_light/(1+Ti_light))

Tvu = H*Gvc;       % Uncompensated voltage loop gain
Tvu_light = H*Gvc_light;

Kpv_dc = 6.9;               % Kp of voltage controller
Kiv_dc = 15687;             % Ki of voltage controller
Gcv = Kpv_dc+Kiv_dc/s;  % Voltage controller

Tv = Tvu*Gcv;               % Compensated voltage loop gain
Tv_light = Tvu_light*Gcv;

Tv_cl = (1/H)*(Tv/(1+Tv));    % Inner closed loop gain
Gvg_cl = (Gvg-(Gvd/Gid)*Gig*(Ti/(1+Ti)))/(1+Tv); % Closed-loop 
                                         % line-to-output gain
Zout_cl = (Zout-(Gvd/Gid)*Giz*(Ti/(1+Ti)))/(1+Tv); % Closed
                                        %-loop output impedance


figure(7)
bode(Tv)
hold on
bode(Tv_light)
legend('5kW','500W')

figure(8)
bode(Tv_cl)
legend('Outer closed loop gain')

figure(9)
bode(Zout_cl)
legend('Closed-loop output impedance bode plot')




Gvg_vcl_tot = Gvg_cl*Gcl_v  % Closed-loop line-to-output gain 
                            % for complete system
Gig_vcl_tot = Gig_cl*Gcl_v  % Closed-loop line-to-current gain 
                            % for complete system

figure(10)
bode(Gvg_vcl_tot)
legend('Closed-loop line-to-output gain')

figure(11)
bode(Gig_vcl_tot)
legend('Closed-loop line-to-current gain')

figure(12)
bode(Gol_pll)
legend('Compensated open loop gain for PLL system')


%% Battery parameters (for various voltage values)
CellCap = 3;
Vcell_nom = 3.7;
Nser = 116;
Npar = 10;
Vbat_nom = Nser*Vcell_nom;
BatCap = CellCap * Npar;
T_bat = 1e-4;

% Vbat = 300V %
SOC_1 = 0.03;
% Vbat = 400V %
SOC_2 = 2.4;
% Vbat = 500V %
SOC_3 = 98.5;
