%% PART 1 - Asymptotic Tracking with Disturbance Rejection: Numerical Simulation

M = 1000;
B = 1;
g = 9.81;
a = B/M;

vdes = 14;
theta = -pi/6;
dbar = g*sin(theta);

% define G, the plant's TF
G = zpk([],-a,1);

% p1, p2 intialized to values we found in the lab report
p1 = 1/a
p2 = 15/7
% try tuning the values of p1, p2
p1 = 20

TI = p1
K = p2

% define C, the controller's TF
C = zpk(-1/TI,0,K);

% T = Y(s)/R(s) when assuming D(s) = 0
T = minreal(C*G * inv(1+C*G))
[zT,pT,kT] = zpkdata(T);
polesOfT = cell2mat(pT)

% settling time determined by this function is 1.812
% we measure settling time for (0 < t < 15s) 
% empirically through the cursor measurement
% tool in the scope to be 1.822 (pretty much same number)
TStepResponseCharacteristics = stepinfo(T)

%% For creating graphs for lab report
% have to run simulink diagram before this is able to be run if you want to
% run
close all
figure
plot(out.time,out.uGraph)
title('u(t) when p1=20, p2=15/7')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
legend('car acceleration')
grid on

figure
plot(out.time,out.errorGraph)
title('Tracking Error when p1=20, p2=15/7')
xlabel('Time (s)')
ylabel('Error')
legend('0.02 * vdes', 'abs(error)')
grid on

figure
plot(out.time,out.outputGraph)
title('y(t) when p1=20, p2=15/7')
xlabel('Time (s)')
ylabel('Speed (m/s)')
legend('car speed', 'vdes')
grid on

