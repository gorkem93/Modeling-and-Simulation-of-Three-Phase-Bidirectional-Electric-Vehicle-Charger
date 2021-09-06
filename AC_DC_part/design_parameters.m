% System parameters %
Vm = 325;    % Peak value of phase voltages
Vdc = 800;  % DC bus voltage
fline = 50;   % Line frequency
fsw = 10e3;  % Switching freq
Ls = 10e-3;  % Line inductance
Rs = 0.5;    % Internal resistance of line inductor
Ts = Ls/Rs;  % Time constant of line inductor
Co = 10e-3;  % DC bus capacitance
Rload = 200; % Load resistance

Tsw = 1/fsw;  % Switching period
Tsamp = Tsw/2; % Sampling period
T1 = Tsamp/10;     % Time constant of voltage sensor (voltage sensor delay)
T2 = Tsamp/10; % Time constant of current sensor (current sensor delay)

G = 1;  % Converter gain
K1 = 1  % Gain of voltage sensor
K2 = 1; % Gain of current sensor
Tsig = T2 + Tsamp       % Effective delay for current control loop
Tdel = T1 + 2*Tsig      % Effective delay for voltage control loop

% PLL parameters %
alfa_pll=sqrt(10);    % Constant ratio of Kp and Ki values of PI controller for PLL loop

Kpll=(1/Vm)*(1/(alfa_pll*Tsamp)) % Kp value of PLL
Tpll=alfa_pll^2*Tsamp            % PLL time constant
Kipll=Kpll/Tpll              % Ki value of PLL



% Current controller parameters %
Kc = (Rs*Ts)/(2*G*K2*Tsig)  
Kpc = Kc;                   % Kp value of current controller

Tc = Ts                     % current controller time constant
Kic = Kpc/Tc                % Ki value of current controller

wn = 1/(sqrt(2)*Tsig)  % Current loop bandwidth


% Voltage controller parameters
K = (3/2)*(Vm/Vdc)         % Ratio between DC bus current and q component of line current (K = I_dc/i_sqe)

alfa_volt = sqrt(10);      % Constant ratio of Kp and Ki values of PI controller for voltage loop
Tv = alfa_volt^2*Tdel      % voltage controller time constant
Kv = (Co*K2)/(K1*K*alfa_volt*Tdel) 

Kpv = Kv                  % Kp value of voltage controller
Kiv = Kpv/Tv              % Ki value of voltage controller



%Closed current loop
s= tf('s')
Gc = (Kc*(1 + s*Tc))/(s*Tc)
Gdelay = G/(1+s*Tsamp)
Gind = (1/Rs)/(1+s*Ts)
Gc_sense = K2/(1+s*T2)
Gcl_c = (Gc*Gdelay*Gind)/(1 + (Gc*Gdelay*Gind*Gc_sense))
fn = (1/(sqrt(2)*Tsig))/(2*pi)      % Current loop bandwidth
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Open voltage loop
Ke = (Kv*K*K1)/K2
Gol_v = (Ke*(1+s*Tv))/(s^2*Tv*Co*(1+s*Tdel)) 
fc = (1/(alfa_volt*Tdel))/(2*pi)    % Cross-over freq. of voltage loop
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

