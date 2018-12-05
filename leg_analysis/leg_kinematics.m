%% Analyse leg kinematics
% Author:   Alex Dietsche
% Date: 10/11/2018

%% Parameters 
% (from Design Exploration and Kinematic Tuning of a Power 
%  Modulating Jumping Monopod paper)
close, clc, clear

A = [-0.024330, -0.015363];
B = [0, 0];             % origin
C = [0.038243, 0.011044];
D = [0.008843, -0.001726];
F = [-0.007718, 0.000153];
G = [0.064137, 0.0245630];
H = [0.043186, 0.0247990];
K = [0.029413, 0.008778];
L = [-0.04621, -0.067287];
M = [-0.056208, -0.067095];
P_0 = [0.00014, -0.07271];


% Pivots points
% A = [-0.02433, -0.015363];
% B = [0.0, 0.0];             % origin
% C = [0.038243, 0.011044];
% D = [0.008843, -0.001726];
% F = [-0.0210218, 0.000153];
% G = [0.043186, 0.024799];
% H = [0.064137, 0.024563];
% K = [0.029413, 0.008778];
% L = [-0.04621, -0.067287];
% M = [-0.056208, -0.067095];
% P_0 = [0.00014, -0.07271];

pivots = [A; B; C; D; F; G; H; K; L; M; P_0];

% Visualization
% visualize_leg(pivots)


% Link lengths
a_1 = norm(F-B);
a_2 = norm(D-B);
b_1 = norm(L-G);
b_2 = norm(C-H);
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
beta_1 = acos(dot((C-H), (L-C)) / (norm(C-H) * norm(L-C)));
beta_2 = acos(dot((C-H), (L-G)) / (norm(C-H) * norm(L-G)));
gamma_1 = acos(dot((K-A), (C-A)) / (norm(K-A) * norm(C-A)));
epsilon_0 = atan(A(2) / A(1));

% Initial angle
t_1 = atan2(F(2), F(1));
l = H-F;
t_2_0 = atan2(l(2), l(1));
l = C-H;
t_3_0 = atan2(l(2), l(1));
l = M-L;
t_4_0 = atan2(l(2), l(1));
l = K-M;
t_5_0 = atan2(l(2), l(1));
l = A-K;
t_6_0 = atan2(l(2), l(1));
l = G-D;
t_7_0 = atan2(l(2), l(1));

% Solve analytically
syms t_2 t_3 t_4 t_5 t_6 t_7
eqns = [
    a_1 * cos(t_1) + l_1 * cos(t_2) + b_2 * cos(t_3) + c_2 * cos(t_6 - gamma_1) + l_0 * cos(epsilon_0),
    a_1 * sin(t_1) + l_1 * sin(t_2) + b_2 * sin(t_3) + c_2 * sin(t_6 - gamma_1) + l_0 * sin(epsilon_0),
    a_1 * cos(t_1) + l_1 * cos(t_2) + b_2 * cos(t_3) + b_3 * cos(t_3 - beta_1) + d_1 * cos(t_4) + l_2 * cos(t_5) + c_1 * cos(t_6) + l_0 * cos(epsilon_0),
    a_1 * sin(t_1) + l_1 * sin(t_2) + b_2 * sin(t_3) + b_3 * sin(t_3 - beta_1) + d_1 * sin(t_4) + l_2 * sin(t_5) + c_1 * sin(t_6) + l_0 * sin(epsilon_0),
    a_2 * cos(t_1 + alpha_1) + l_3 * cos(t_7) + b_1 * cos(t_3 - beta_2) + d_1 * cos(t_4) + l_2 * cos(t_5) + c_1 * cos(t_6) + l_0 * cos(epsilon_0),
    a_2 * sin(t_1 + alpha_1) + l_3 * sin(t_7) + b_1 * sin(t_3 - beta_2) + d_1 * sin(t_4) + l_2 * sin(t_5) + c_1 * sin(t_6) + l_0 * sin(epsilon_0),
    ];

vars = [t_2 t_3 t_4 t_5 t_6 t_7];
solutions = solve(eqns, vars);





