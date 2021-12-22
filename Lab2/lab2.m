%% Part 1 - Numerical Linearization and Stability Assessment
clc

La = 0.05;
Ra = 3;
M = 0.1;
km = 0.1;
g = 9.81;

ybar = 0.1;
xbar = [ybar; 0; ybar*sqrt(g)];
ubar = 3*ybar*sqrt(g);

[A,B,C,D] = linmod('lab2_1',xbar,ubar)

A1 = [0 1 0; (2*km*xbar(3)^2)/(M*xbar(1)^3) 0 (-2*km*xbar(3))/(M*xbar(1)^2); 0 0 -Ra/La];
B1 = [0; 0; 1/La];
C1 = [1 0 0];
D1 = 0;

% error between experimental and theoretical models
errorA = norm(A-A1)
errorB = norm(B-B1)

% get transfer function and poles of the TF
G = tf(ss(A1,B1,C1,D1));
zpk_G = zpk(G)

[z1,p1,k1] = zpkdata(zpk_G);
poles = cell2mat(p1)

% find eigenvalues of A1
eigA = eig(A1)

%% Part 2 - Feedback Control of the Magnetic Levitation System

z = 10;
p = 100;
K = 70
CONTROLLER = zpk(-z,-p,K);

[z2,p2,k2] = zpkdata(1-CONTROLLER*G);
% TF has a pole on the right hand plane (a zero of 1-CG), when K = 70
% which makes the response exponentially increase and therefore unstable
controller_poles = cell2mat(z2)

% Making K = 100 moves all zeros in the OLHP, and therefore the system is
% BIBO stable
K = 100
CONTROLLER = zpk(-z,-p,K);
[z2,p2,k2] = zpkdata(1-CONTROLLER*G);
controller_poles = cell2mat(z2)

% Bounds of initial conditions where the controller works
% 18 is upper bound
% 0.05 is lower bound
upper_bound_init = 18
lower_bound_init = 0.05

%% graph for lab report of controller with K = 100

plot(out.t,out.output)
xlabel('Time (s)')
ylabel('Distance from Electromagnet (m)')
