%% Analyse leg kinematics
% Author:   Alex Dietsche
% Date: 10/11/2018

%% Parameters 
% (from Design Exploration and Kinematic Tuning of a Power 
%  Modulating Jumping Monopod paper)
close, clc, clear

% Pivots points
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

% Visualization
visualize_leg(pivots)


% Link lengths
a_1 = norm(F-B);
a_2 = norm(D-B);
b_1 = norm(L-H);
b_2 = norm(C-G);
b_3 = norm(L-C);
c_1 = norm(K-A);
c_2 = norm(C-A);
d_1 = norm(L-M);
l_0 = norm(B-A);
l_1 = norm(H-F);
l_2 = norm(M-K);
l_3 = norm(G-D);

% Angles
alpha_1 = acos(dot((D-B), (F-B)) / (norm(D-B) * norm(F-B)));
beta_1 = acos(dot((L-H), (C-G)) / (norm(L-H) * norm(C-G)));
beta_2 = acos(dot((C-G), (L-C)) / (norm(C-G) * norm(L-C)));
gamma_1 = acos(dot((C-A), (K-A)) / (norm(C-A) * norm(K-A)));
epsilon_0 = atan(A(2) / A(1));

% Initial angle
t_1 = atan(F(2) / F(1));

% Solve analytically
syms t_2 t_3 t_4 t_5 t_6 t_7
eqns = [
    a_1 * cos(t_1) + l_2 * cos(t_2) + b_1 * cos(t_3) + d_1 * cos(t_4) + l_3 * cos(t_5) + c_1 * cos(t_6) + l_0 * cos(epsilon_0) == 0,
    a_1 * sin(t_1) + l_2 * sin(t_2) + b_1 * sin(t_3) + d_1 * sin(t_4) + l_3 * sin(t_5) + c_1 * sin(t_6) + l_0 * sin(epsilon_0) == 0,
    a_2 * cos(t_1 - alpha_1) + l_3 * cos(t_7) + b_2 * cos(t_3 + beta_1) + c_2 * cos(t_6 + gamma_1) + l_0 * cos(epsilon_0) == 0,
    a_2 * sin(t_1 - alpha_1) + l_3 * sin(t_7) + b_2 * sin(t_3 + beta_1) + c_2 * sin(t_6 + gamma_1) + l_0 * sin(epsilon_0) == 0,
    a_2 * cos(t_1 - alpha_1) + l_3 * cos(t_7) + b_2 * cos(t_3 + beta_1) + b_3 * cos(t_3 + beta_1 - beta_2) + d_1 * cos(t_4) + l_2 * cos(t_5) + c_1 * cos(t_6) + l_0 * cos(epsilon_0) == 0,
    a_2 * sin(t_1 - alpha_1) + l_3 * sin(t_7) + b_2 * sin(t_3 + beta_1) + b_3 * sin(t_3 + beta_1 - beta_2) + d_1 * sin(t_4) + l_2 * sin(t_5) + c_1 * sin(t_6) + l_0 * sin(epsilon_0) == 0
    ];

vars = [t_2 t_3 t_4 t_5 t_6 t_7];
solutions = solve(eqns, vars);








