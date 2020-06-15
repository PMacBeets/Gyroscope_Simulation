clear all
close all

syms t a(t) b(t) y(t) d(t)
syms F_Gx F_Gy F_Gz 
syms m_r R_i h R_0 H L r g rho m_a m_b m_c m_f

% Substitute Values If Required
%r = 1; R_0 = 10; R_i = 6; H = 30; h = 3; L = 15; m_r = 50; g = 9.8; rho = 1;

% Rotation Matrices
Rr3 = [cos(d) -sin(d) 0; sin(d) cos(d) 0; 0 0 1];
R3r = transpose(Rr3);

R10 = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
R21 = [1 0 0; 0 cos(b) -sin(b); 0 sin(b) cos(b)];
R32 = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
R01 = transpose(R10);
R12 = transpose(R21);
R23 = transpose(R32);

% Define Mass Of Bodies
m_a = rho*pi()*r^2*H;
m_b = 2*rho*pi()^2*r^2*R_0;
m_c = m_b;
m_f = m_a + m_b + m_c;

% Define Inertia Tensor

Ir_G_rotor = m_r*[(3*R_i^2 + h^2)/12, 0 0; 0, (3*R_i^2 + h^2)/12, 0;...
    0, 0, (R_i^2)/2];
I3_G_rotor = Rr3*Ir_G_rotor*R3r;

I3_G_a = m_a*[(3*r^2 + H^2)/12, 0, 0; 0, (3*r^2 + H^2)/12, 0; 0, 0, (r^2)/2];
I3_G_b = m_b*[(5/8)*r^2 + (1/2)*R_0^2, 0, 0; 0, (3/4)*r^2 + R_0^2, 0;...
    0, 0, (5/8)*r^2 + (1/2)*R_0^2];
I3_G_c = m_c*[(5/8)*r^2 + (1/2)*R_0^2, 0, 0; 0, (5/8)*r^2 + (1/2)*R_0^2, 0;...
    0, 0, (3/4)*r^2 + R_0^2];
I3_G_frame = I3_G_a + I3_G_b + I3_G_c;

r3_OG = [0; 0; L];

del_x = 0;
del_y = 0;
del_z = L;

delta_matrix = [del_z^2, 0, 0; 0, del_z^2, 0; 0, 0, 0];
I3_O_frame = I3_G_frame + m_f*delta_matrix;

% Sum of Forces at Joint G  -------------------------------------------------------

F_3_G = [F_Gx; F_Gy; F_Gz];

G_0_r = [0; 0; -m_r*g];

G_3_r = R23*R12*R01*G_0_r;

sum_F_r = F_3_G + G_3_r;

w3_32 = [0; 0; diff(y)];
w2_21 = [diff(b); 0; 0];
w1_10 = [0; 0; diff(a)];
w3_3 = w3_32 + R23*w2_21 + R23*R12*w1_10;

r3_OG_dot = cross(w3_3, r3_OG);
r3_OG_ddot = diff(r3_OG_dot) + cross(w3_3, r3_OG_dot);

Sol28 = -sum_F_r + m_r*r3_OG_ddot;
Sol28 = simplify(Sol28);
%disp('Solution 28:     p3_OG_r =')
%disp(subs(Sol28))

%% Sum of Moments at Joint G  -------------------------------------------------------

syms M_Gx M_Gy

M_3_g = [M_Gx; M_Gy; 0];

wr_r3 = [0; 0; diff(d)];

wr_r = wr_r3 + w3_3;

wr_r_dot = diff(wr_r);

sum_M_G = M_3_g;

h3_G_dot = I3_G_rotor*wr_r_dot + cross(wr_r, (I3_G_rotor*wr_r));


Sol29 = -sum_M_G + h3_G_dot;
Sol29 = simplify(expand(Sol29));
      

%disp('Solution 29:     h3_G_dot =')
%disp(vpa(subs(Sol29)))



%% Sum of Forces at Joint O   -----------------------------------------------------

syms F_Ox F_Oy F_Oz

F_3_O = [F_Ox; F_Oy; F_Oz];

G_0_f = [0; 0; -m_f*g];

G_3_f = R23*R12*R01*G_0_f;

sum_F_f = F_3_O - F_3_G + G_3_f;

Sol30 = simplify(-sum_F_f + m_f*r3_OG_ddot);
%disp('Solution 30:     p3_OG_f=')
%disp(vpa(subs(Sol30)))

%% Sum of Moments at Joint O -------------------------------------------------------

sum_M_f = - M_3_g + cross(r3_OG, (-F_3_G)) + cross(r3_OG, G_3_f);

w3_3_dot = diff(w3_3);

h3_O_dot = I3_O_frame*w3_3_dot + cross(w3_3, (I3_O_frame*w3_3));

Sol31 = -sum_M_f + h3_O_dot;

%disp('Solution31:    h3_O_dot=')
%disp(vpa(subs(Sol31)))

%% Extract Equations Of Motion

% Turn 1*1 symfun into 3*3 - Dont know why this works

Sol28 = Sol28(1);
Sol29 = Sol29(1);
Sol30 = Sol30(1);
Sol31 = Sol31(1);

% Solve in terms of F_Gx, F_Gy and F_Gz
F_Gx = solve(Sol28(1) == 0, F_Gx);
F_Gy = solve(Sol28(2) == 0, F_Gy);
F_Gz = solve(Sol28(3) == 0, F_Gz);

% Solve in terms of M_Gx, M_Gy and F_Gz
M_Gx = solve(Sol29(1) == 0, M_Gx);
M_Gy = solve(Sol29(2) == 0, M_Gy);

% Equations Of Motions 
EOM1 = subs(Sol31(1));
EOM2 = subs(Sol31(2));
EOM3 = subs(Sol29(3)); %== 0
EOM4 = subs(Sol31(3)); %== 0

% Clean Up For Reading
syms a_d b_d y_d d_d a_dd b_dd y_dd d_dd

EOM1 = subs(EOM1,{a(t),b(t),y(t),d(t),...
      diff(a(t), t),diff(b(t), t),diff(y(t), t),diff(d(t),t),...
      diff(a(t),t,t),diff(b(t),t,t),diff(y(t),t,t),diff(d(t),t,t)},...
      {'a','b','y','d','a_d','b_d','y_d','d_d','a_dd','b_dd','y_dd','d_dd'})
  
EOM2 = subs(EOM2,{a(t),b(t),y(t),d(t),...
      diff(a(t), t),diff(b(t), t),diff(y(t), t),diff(d(t),t),...
      diff(a(t),t,t),diff(b(t),t,t),diff(y(t),t,t),diff(d(t),t,t)},...
      {'a','b','y','d','a_d','b_d','y_d','d_d','a_dd','b_dd','y_dd','d_dd'})
  
EOM3 = subs(EOM3,{a(t),b(t),y(t),d(t),...
      diff(a(t), t),diff(b(t), t),diff(y(t), t),diff(d(t),t),...
      diff(a(t),t,t),diff(b(t),t,t),diff(y(t),t,t),diff(d(t),t,t)},...
      {'a','b','y','d','a_d','b_d','y_d','d_d','a_dd','b_dd','y_dd','d_dd'});
  
EOM4 = subs(EOM4,{a(t),b(t),y(t),d(t),...
      diff(a(t), t),diff(b(t), t),diff(y(t), t),diff(d(t),t),...
      diff(a(t),t,t),diff(b(t),t,t),diff(y(t),t,t),diff(d(t),t,t)},...
      {'a','b','y','d','a_d','b_d','y_d','d_d','a_dd','b_dd','y_dd','d_dd'})

EOM = [EOM1; EOM2; EOM3; EOM4];
var = [a_dd b_dd y_dd d_dd];
 
% Match Coefficients
[C, B] = equationsToMatrix(EOM,var);
S =  simplify((inv(C)* B));

% Substitute Values & Extract Equations Of Motion
S(1) = subs(S(1));
S(2) = subs(S(2));
S(3) = subs(S(3));
S(4) = subs(S(4));

a_dd = subs(S(1),{a(1),b(1),y(1),d(1)},{'a','b','y','d'})
b_dd = subs(S(2),{a(1),b(1),y(1),d(1)},{'a','b','y','d'})
y_dd = subs(S(3),{a(1),b(1),y(1),d(1)},{'a','b','y','d'})
d_dd = subs(S(4),{a(1),b(1),y(1),d(1)},{'a','b','y','d'})





