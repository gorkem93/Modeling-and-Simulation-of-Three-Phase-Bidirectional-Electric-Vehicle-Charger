% System parameters %
Vm = 325;    % Peak value of phase voltages
Vdc = 1000;  % DC bus voltage
fline = 50;   % Line frequency
fsw = 10e3;  % Switching freq
Ls = 10e-3;  % Line inductance
Rs = 0.5;    % Internal resistance of line inductor
Ts = Ls/Rs;  % Time constant of line inductor

Tsw = 1/fsw;  % Switching period
Tsamp = Tsw/2; % Sampling period
T1 = Tsamp/10; % Time constant of voltage sensor (voltage sensor delay)
T2 = Tsamp/10; % Time constant of current sensor (current sensor delay)

G = 1;  % Converter gain
K2 = 1; % Gain of current sensor
Tsig = T2 + Tsamp % Effective delay for current control loop


% PLL parameters %

alfa=sqrt(10);    %Constant ratio of Kp and Ki values of PI controller

Kpll=(1/Vm)*(1/(alfa*Tsamp)) % Kp value of PLL
Tpll=alfa^2*Tsamp            % PLL time constant
Kipll=Kpll/Tpll              % Ki value of PLL



% Current controller parameters %

Kc = (Rs*Ts)/(2*G*K2*Tsig)  
Kpc = Kc;                   % Kp value of current controller

Tc = Ts                     % current controller time constant
Kic = Kpc/Tc                % Ki value of current controller

wn = 1/(sqrt(2)*Tsig)  % Current loop bandwidth
