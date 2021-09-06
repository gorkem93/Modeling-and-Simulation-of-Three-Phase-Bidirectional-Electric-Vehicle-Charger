%% Converter ratings
Vg=170;             % Input voltage
V=400;              % Output voltage

P=2000;             % Output power
Iload=P/V;              % Load current
R=V/Iload;              % Load resistance

fsw=100e3;           % Switching frequency
Tsw=1/fsw;           % Switching period

%Inductor
L=250e-6;             % Inductance
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

Tsamp_dc = Tsw/2;    % Sampling delay

%% DC analysis

% Duty cycle
D=1-((Vg+sqrt(Vg^2-4*V^2*((RL+Rdson)/R)))/(2*V))      % Duty cycle value

% Inductor current
I = V/(R*(1-D))

% Efficiency
nd = 1/(1+(RL+Rdson)/(R*(1-D)^2)) 
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
Tiu_fw = Tiu/Rf;            % Forward uncomp. current loop gain
Kpc_boost = 0.618;          % Kp of current controller
Kic_boost = 13590;          % Ki of current controller
Gci = Kpc_boost+Kic_boost/s;    % Current controller
Ti = Tiu*Gci;                   % Compensated current loop gain

Ti_cl = (1/Rf)*(Ti/(1+Ti));    % Inner closed loop gain
Gig_cl = Gig/(1+Ti);            % Closed-loop line-to-current gain
Giz_cl = Giz/(1+Ti);            % Closed-loop load-to-current gain

figure(1)
bode(Tiu)
hold on
bode(Ti)
legend('Uncompensated current loop gain','Compensated current loop gain')

figure(2)
bode(Ti_cl)
legend('Inner closed loop gain')

figure(3)
bode(Gig_cl)
legend('Closed-loop line-to-current gain')


figure(4)
bode(Giz_cl)
legend('Closed-loop load-to-current gain')




% Outer voltage loop design

Gvc = (1/Rf)*(Gvd/Gid)*(Ti/(1+Ti))  % Control-to-output voltage transfer function
Tvu = H*Gvc;                        % Uncompensated voltage loop gain
Kpv_boost = 15.62;                  % Kp of voltage controller
Kiv_boost = 25396;                  % Ki of voltage controller
Gcv = Kpv_boost+Kiv_boost/s;        % Voltage controller
Tv = Tvu*Gcv;         % Compensated voltage loop gain

Tv_cl = (1/H)*(Tv/(1+Tv));    % Inner closed loop gain
Gvg_cl = (Gvg-(Gvd/Gid)*Gig*(Ti/(1+Ti)))/(1+Tv);     % Closed-loop line-to-output gain
Zout_cl = (Zout-(Gvd/Gid)*Giz*(Ti/(1+Ti)))/(1+Tv);     % Closed-loop output impedance


figure(5)
bode(Tvu)
hold on
bode(Tv)
legend('Uncompensated voltage loop gain','Compensated voltage loop gain')

figure(6)
bode(Tv_cl)
legend('Outer closed loop gain')

figure(7)
bode(Gvg_cl)
legend('Closed-loop line-to-output gain')

figure(8)
bode(Zout_cl)
legend('Closed-loop output impedance bode plot')

