%% PART 1a - LEAD Controller Design

Ra = 3;
Ke = 0.01;
Kt = 0.01;
I = 6e-4;
b = 1e-4;
thetades = pi/2;
tau_l = 0.01;
d_bar = (Ra/Kt) * tau_l;
Vlim = 5;

% define G of the system (the dc motor)
A = Kt/(Ra * I);
B = (b + Ke*Kt/Ra)/I;
G = zpk([], [0, -B], A);

% design LEAD controller
close all
initial_crossover = 20;

% ----PART 1c - Tune initial_crossover START----
% initial_crossover = 20;
% ----PART 1c - Tune initial_crossover END----

K = 1/abs(evalfr(G, initial_crossover*j))

% verify KG crosses 0dB at init crossover
KG = K*G;
figure
bode(KG, {0.1, 100})
grid on

figure
alpha = 0.1;
KG_Alpha = K*G/sqrt(alpha)
[GM, PM, Wcg, Wcp] = margin(KG_Alpha)
grid on
% Wcp holds the crossover frequency of the KG/sqrt(alpha)
omega_bar = Wcp;
T = 1/(sqrt(alpha) * omega_bar)

% define our LEAD Controller
C1 = K * tf([T 1], [alpha*T 1])
figure
bode(G, C1*G)
title('LEAD Controller Bode Plots')
legend('G', 'C1*G')
grid on

figure
margin(C1*G)
grid on

% plots for output 1
figure
bode(G, K*G, C1*G)
title('LEAD Controller/Plant Bode Plots')
legend('G', 'K*G', 'C1*G')
grid on

%% PART 1b - PI Controller Design

% crossover frequency of CG is omega_bar
% design PI and create complete controller
Ti = 1/(omega_bar/10)

% ---PART 1c - Tune Ti START---
% Ti = 0.445
% ---PART 1c - Tune Ti END---

C2 = tf([Ti 1], [Ti 0]);
C = C1*C2;

% make sure gain crossover and phase margin are the close to C1*G
close all 
figure
margin(C*G)
% PM = 50 degrees, crossover freq = 36 rad/s
grid on

% make sure tracking error is no greater than 2% of the max value of
% disturbance
freqResponse = G/(1 + C*G)
figure
bode(freqResponse)
% it is below -34 dB for all frequencies
grid on

% get TR and TD
T_R = minreal((C*G)/(1+C*G))
T_D = minreal(G/(1+C*G))

% get their step responses to get y(t)
time = linspace(0,5,1000);
stepResponse_R = step(thetades * T_R, time);
stepResponse_D = step(d_bar * T_D, time);

figure
subplot(2,1,1)
plot(time, stepResponse_R)
title("Reference Signal Step Response (y(t) from r(t) contribution)")
xlabel("Time (s)")
ylabel("Theta (rad)")
grid on

subplot(2,1,2)
plot(time, stepResponse_D)
title("Disturbance Signal Step Response (y(t) from d(t) contribution)")
xlabel("Time (s)")
ylabel("Theta (rad)")
grid on

% stepinfo to get time domain characteristics. The following are the
% results:
% Ts = 0.1975
% %OS = 31.19
T_R_StepResponseCharacteristics = stepinfo(thetades * T_R)

%% Part 2 - Integrator Antiwindup

Kaw = 1/Ti;

% tune Kaw (found 1.3*1/Ti to be the best for fast settling time)
%Kaw = 1.3*1/Ti;
