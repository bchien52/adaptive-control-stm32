clear; clc;

% Parameters
Ts = 0.005;
s = tf('s');

Gm_s = (2347*s + 21248) / (s^4 + 36.16*s^3 + 492.2*s^2 + 5750*s + 21248);
%Gm_s = 3343 / (s^3 + 36.16*s^2 + 492.2*s + 3403);

% ZOH Discretization
Gm_z = c2d(Gm_s, Ts, 'zoh');

% Display the Discrete-Time Transfer Function
Gm_z 
