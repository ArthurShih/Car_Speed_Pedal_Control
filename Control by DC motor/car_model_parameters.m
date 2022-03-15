clear;close all;
%% Parameters
R = 1;
L = 0.5;
Kt = 0.023;
Ke = 0.023;
Gr = 2.5;
r = 3.2*1e-2;
m = 1.5;
A = 2.34/64;
rho = 1.204;
Cd = 0.3;
g = 9.81;
A_sys = [-R/L -Ke*Kt/L;Gr^2/(m*r^2) 0];
B_sys = [Kt/L;0];
C_sys = [0 r/Gr];
D_sys = 0;

%% System Identification
% My linearization
[num_mylin, den_mylin] = ss2tf(A_sys,B_sys,C_sys,D_sys);
tf_mylin = tf(num_mylin, den_mylin);
% System Identification App
num_id = 2.398;
den_id = [1 2.004 4.314];
tf_id = tf(num_id, den_id);
bode(tf_mylin);
hold on;
bode(tf_id);
legend("tf mylin","tf id");
tf_sys = tf_mylin;