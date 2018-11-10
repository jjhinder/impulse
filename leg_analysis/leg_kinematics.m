%% Analyse leg kinematics
% Author:   Alex Dietsche
% Date: 10/11/2018

%% Parameters 
% (from Design Exploration and Kinematic Tuning of a Power 
%  Modulating Jumping Monopod paper)
close, clc, clear

% Leg geometry
A = [-0.02433, -0.015363];
B = [0.0, 0.0];             % origin
C = [0.038243, 0.011044];
D = [0.008843, -0.001726];
F = [-0.0210218, 0.000153];
G = [0.043186, 0.024799];
H = [0.064137, 0.024563];
K = [0.029413, 0.008778];
L = [-0.04621, -0.067287];
M = [-0.056208, -0.067095];
P_0 = [0.00014, -0.07271];

pivots = [A; B; C; D; F; G; H; K; L; M; P_0];

visualize_leg(pivots)









