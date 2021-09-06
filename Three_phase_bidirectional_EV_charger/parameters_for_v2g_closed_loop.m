% ****************************************************
%% PARAMETERS FOR V2G MODE OF OPERATION %%
% ****************************************************
indicator = 0;
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
                      % (voltage sensor delay)
T2 = Tsamp_ac/10; % Time constant of current sensor 
                  % (current sensor delay)

G = 1;  % Converter gain
K1 = 1  % Gain of voltage sensor
K2 = 1; % Gain of current sensor
Tsig = T2 + Tsamp_ac   % Effective delay for current 
                       % control loop
Tdel = T1 + 2*Tsig     % Effective delay for voltage 
                       % control loop

Ron_ac = 10e-3;      % On-state resistance of transistors 
                     % in AC-DC PWM converter
Von_ac = 0.7;        % Forward voltage drop of 
                     % transistors in AC-DC PWM converter

% PLL parameters %
alfa_pll=sqrt(10);    % Constant ratio of Kp and Ki values 
                      % of PI controller for PLL loop

Kpll=(1/Vm)*(1/(alfa_pll*Tsamp_ac)) % Kp value of PLL
Tpll=alfa_pll^2*Tsamp_ac            % PLL time constant
Kipll=Kpll/Tpll              % Ki value of PLL

%Compensated open PLL
s= tf('s')
Gol_pll = (Vm*Kipll*(Tpll*s+1))/(s^2*(1+s*Tsamp_ac))
fc_pll = (1/(alfa_pll*Tsamp_ac))/(2*pi)   
% Cross-over freq. of PLL


% Current controller parameters %
Kc = (Rs*Ts)/(2*G*K2*Tsig)  
Kpc = Kc;    % Kp value of current controller

Tc = Ts      % Current controller time constant
Kic = Kpc/Tc % Ki value of current controller

wn = 1/(sqrt(2)*Tsig)  % Current loop bandwidth


% Voltage controller parameters
K = (3/2)*(Vm/Vdc)  % Ratio between DC bus current 
                    % and q component of line current 
                    % (K = I_dc/i_sqe)

alfa_volt = sqrt(10);  % Constant ratio of Kp and Ki 
                       % values of PI controller for 
                       % voltage loop
Tv_ac = alfa_volt^2*Tdel  % Voltage controller time 
                          % constant
Kv = (Co*K2)/(K1*K*alfa_volt*Tdel) 

Kpv = Kv         % Kp value of voltage controller
Kiv = Kpv/Tv_ac  % Ki value of voltage controller

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
fn = (1/(sqrt(2)*Tsig))/(2*pi) % Current loop bandwidth
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Open voltage loop
Ke = (Kv*K*K1)/K2
Gol_v = (Ke*(1+s*Tv_ac))/(s^2*Tv_ac*Co*(1+s*Tdel)) 
fc_v = (1/(alfa_volt*Tdel))/(2*pi) % Cross-over freq. 
                                   % of voltage loop
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

%% DC-DC converter ratings (boost mode)
Vg=500;             % Input voltage
V=800;              % Output voltage

R=1e6;    % Load is assumed to be nearly as no load

fsw_dc=100e3;           % Switching frequency
Tsw_dc=1/fsw_dc;        % Switching period

%Inductor
L=250e-6;            % Inductance
RL=10e-3;            % Internal resistance

%Capacitor
C=Co;                % Capacitance
Resr=0;              % ESR value

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
D=1-((Vg+sqrt(Vg^2-4*V^2*((RL+Rdson)/R)))/(2*V))   

% Inductor current
I = V/(R*(1-D))

% Efficiency
nd = 1/(1+(RL+Rdson)/(R*(1-D)^2))

%Inductor ripple current
I_ripple=((Vg-(Rdson+RL)*I)*D*Tsw_dc)/(2*L)
%% Small signal AC analysis
s=tf('s')

Re=RL+Rdson

%Gvd - duty cycle-to-output voltage TF
Gvd=((1-D)*(1+s*C*Resr)*(V-I*((s*L+Re)/(1-D))))/(((L*C*(R+Resr))/R)*s^2+((R*Re*C+Resr*Re*C+Resr*R*C*(1-D)^2+L)/R)*s+((Re/R)+(1-D)^2))

%Gid - duty cycle-to-inductor current TF
Gid=(2*V*(1+s*C*((R/2)+Resr)))/((L*C*(R+Resr))*s^2+(R*Re*C+Resr*Re*C+Resr*R*C*(1-D)^2+L)*s+(Re+R*(1-D)^2))

%Gvg - line-to-output voltage TF
Gvg=((1-D)*(1+s*C*Resr))/(((L*C*(R+Resr))/R)*s^2+((R*Re*C+Resr*Re*C+Resr*R*C*(1-D)^2+L)/R)*s+((Re/R)+(1-D)^2))

%Gig - line-to-inductor current TF
Gig=(1+s*C*(R+Resr))/((L*C*(R+Resr))*s^2+(R*Re*C+Resr*Re*C+Resr*R*C*(1-D)^2+L)*s+(Re+R*(1-D)^2))

%Zout - output impedance TF
Zout=(-(1+s*C*Resr)*(s*L+Re))/(((L*C*(R+Resr))/R)*s^2+((R*Re*C+Resr*Re*C+Resr*R*C*(1-D)^2+L)/R)*s+((Re/R)+(1-D)^2))

%Giz - load-to-inductor current TF
Giz=((1-D)*(1+s*C*Resr))/(((L*C*(R+Resr))/R)*s^2+((R*Re*C+Resr*Re*C+Resr*R*C*(1-D)^2+L)/R)*s+((Re/R)+(1-D)^2))


%% Compensator design

% Sampling delay TF
Hsamp = 1/(1+s*Tsamp_dc)

% Inner current loop design

Tiu = Rf*(1/VM)*Hsamp*Gid;  % Uncompensated current loop gain
Tiu_fw = Tiu/Rf;       % Forward uncomp. current loop gain
Kpc_dc = 0.29;              % Kp of current controller 
Kic_dc = 9270.6;            % Ki of current controller
Gci = Kpc_dc+Kic_dc/s;      % Current controller
Ti = Tiu*Gci;         % Compensated current loop gain

Ti_cl = (1/Rf)*(Ti/(1+Ti));    % Inner closed loop gain
Gig_cl = Gig/(1+Ti);   % Closed-loop line-to-current gain
Giz_cl = Giz/(1+Ti);   % Closed-loop load-to-current gain

figure(4)
bode(Ti)
legend('Compensated current loop gain')

figure(5)
bode(Gig_cl)
legend('Closed-loop line-to-current gain')


% Outer voltage loop design

Gvc = (1/Rf)*(Gvd/Gid)*(Ti/(1+Ti))  
% Control-to-output voltage transfer function
Tvu = H*Gvc;% Uncompensated voltage loop gain
Kpv_dc = 625.3;     % Kp of voltage controller
Kiv_dc = 1.26e6;    % Ki of voltage controller
Gcv = Kpv_dc+Kiv_dc/s;  % Voltage controller
Tv = Tvu*Gcv;  % Compensated voltage loop gain

Tv_cl =  (Tv/(1+Tv)); % Inner closed loop gain
Gvg_cl = (Gvg-(Gvd/Gid)*Gig*(Ti/(1+Ti)))/(1+Tv);     
% Closed-loop line-to-output gain

Zout_cl = (Zout-(Gvd/Gid)*Giz*(Ti/(1+Ti)))/(1+Tv);   
% Closed-loop output impedance


figure(6)
bode(Tv)
legend('Compensated voltage loop gain')

figure(7)
bode(Gvg_cl)
legend('Closed-loop line-to-output gain')



Tv_cl_tot = Tv_cl + Gcl_v % Closed-loop gain for DC bus 
                          % voltage deviation

figure(8)
bode(Tv_cl_tot)
legend('Closed-loop gain for DC bus voltage deviation')


figure(9)
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


% Vbat = 373V %
SOC_1 = 2.3;

% Vbat = 500V %
SOC_3 = 99.04995;


