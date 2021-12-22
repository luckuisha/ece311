%% Part 1 - LTI System Representations and Conversions

% DECLARE VARIABLES FOR LAB
La = 0.02;
Ra = 3;
Ke = 0.01;
Kt = 0.01;
I = 6e-4;
b = 1e-4;

A = [0 1 0; 0 -b/I Kt/I; 0 -Ke/La -Ra/La];
B = [0; 0; 1/La];
C = [0 1 0];
D = 0;

A1 = [0 1; 0 -(b + (Ke*Kt)/Ra)/I];
B1 = [0; Kt/(Ra*I)];
C1 = [0 1];
D1 = 0;

motor = ss(A,B,C,D);
motor_simplified = ss(A1,B1,C1,D1);
G_motor = tf(motor)
G_motor_simplified = tf(motor_simplified)
zpk_motor = zpk(motor)

[z,p,k] = zpkdata(zpk_motor);
poles = cell2mat(p)

[num,den] = tfdata(G_motor);
num = cell2mat(num)
den = cell2mat(den)

[num1,den1] = tfdata(G_motor_simplified);
num1 = cell2mat(num1)
den1 = cell2mat(den1)

%{
grid on
bode(G_motor)
grid

The poles for G_motor are located at -0.2223 and -149.9444. They both lie on the 
negative s axis but they're quite far apart from each other. The magnitude of the
-0.2223 pole is larger than the -149.9444 pole.

If the input is the unit step function and poles are on the left side of the s-axis,
the motor will converge to a value rather than exponentially increase. 
Since both poles are real and magnitude of the -0.2 pole is larger,
the shape of the step response will be an exponential decay reflected on
the t-axis with a gradual approach to steady-state.
%}

%% Part 2 - Numerical Simulation of LTI Systems
close all

% SHOW THAT SIMPLIFIED MOTOR IS A GOOD APPROXIMATION OF ACTUAL MOTOR MODEL
T = linspace(0,30,1000);

subplot(3,1,1)
Y1 = step(motor,T);
plot(T,Y1)
title("Motor Step Response")
xlabel("Time (s)")
ylabel("ThetaDot (rad/s)")

subplot(3,1,2)
Y2 = step(motor_simplified,T);
plot(T,Y2)
title("Simplified Motor Step Response")
xlabel("Time (s)")
ylabel("ThetaDot (rad/s)")

subplot(3,1,3)
plot(T,Y1-Y2)
title("Difference in Magnitude of 2 Responses")
xlabel("Time (s)")
ylabel("ThetaDot (rad/s)")
%{
We can see from this plot, the simplfied motor approximates the actual
motor model very well. It's only off by 0.04 at largest.
%}

% ASYMPTOTIC VALUE OF MOTOR SPEED IN RESPONSE TO UNIT STEP
approxAsympMotorSpeed = Y1(end,:)
theoreticalAsympMotorSpeed = evalfr(G_motor,0)
%{
The approximate asymptotic value of the motor speed is 24.9682 rad/s in
response to a unit step input and the theoretical value is 25 rad/s.
The values are very close to each other.

Using the Final Value Theorem,
theoreticalAsympMotorSpeed = lim s->0 (sF(s))
                           = lim s->0 (s * 1/s * G_motor)
                           = evaluate G_motor at 0
%}

% PLOT ARMATURE CURRENT
figure
[y,t,x] = step(motor, T);
plot(T, x(:,3))
title("Armature Current")

% PLOT RESPONSE TO SIN(T)
figure
X0 = [0;-1;.5];
outputResponse = lsim(motor,sin(T),T,X0);
plot(T,outputResponse)
title("Motor Response to sin(t)")

approximateAmpOfOscillation = findpeaks(outputResponse)
approximateAmpOfOscillation = approximateAmpOfOscillation(end,:)
%{
The approximate amplitude of oscillation in steady-state of the motor speed
in response to input sin(t), is around 5.4326. We calculated this by
finding the peaks in the output response and getting the last peak to get
the stead-state amplitude of oscillation.
%}

% EVALUATE MOTOR TRANSFER FUNCTION AT s = i
valAti = evalfr(G_motor,i);
theoreticalAmpOfOscillation = abs(valAti)
%{
The magnitude is 5.4251 which is approximately equal to the value found
above (5.4326).
%}

%% Part 3 - Definition of LTI System Blocks in Simulink and Numerical Simulation

% Simulink model in lab1_1.slx

%{
The simplified transfer function model approximated the full transfer
function model very well as the lines are practically the same on the
graphs.
%}

%% Part 4 - Proportional Control of the Permanent Magnet DC Motor

% Simulink model in lab1_2.slx

%{
The steady-state tracking error when the gain was 0.1, is 0.2857. The steady-state 
tracking error when the gain was 1, is 0.03846. We measured within the tracking 
error scope in Simulink using the cursor measurement tool. The error
decreases as K increases. The rate of convergence of the tracking error
is also faster when K increases.

As the system goes to steady-state, the error approaches very close to 0,
showing that it is adequate to regulate the speed of a DC motor. To make
it even more accurate, the controller can also look at the history of
errors and make corrections based off this information (integral
controller) or the last error and correct off this (derivative controller).
%}
