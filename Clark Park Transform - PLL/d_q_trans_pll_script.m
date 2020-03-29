Vm=230*sqrt(2);   %Peak value of voltage per phase
Ts=1/10000;       %Sampling time
alfa=sqrt(10);    %Constant ratio of Kp and Ki values of PI controller

Kpll=(1/Vm)*(1/(alfa*Ts)) %Kp value
Tpll=alfa^2*Ts    %PLL time constant
Kipll=Kpll/Tpll   %Ki vslue

