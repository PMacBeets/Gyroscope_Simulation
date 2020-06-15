clc
clear all
syms r Ro Ri H h mr p L
syms t
syms a(t) b(t) y(t) d(t)
syms g


%% Differentiation
a_d = diff(a,t);
b_d = diff(b,t);
y_d = diff(y,t);
d_d = diff(d,t);

a_2d = diff(a,t,t);
b_2d = diff(b,t,t);
y_2d = diff(y,t,t);
d_2d = diff(d,t,t);


%% Rotation Matrices
R_01 = [cos(a) -sin(a) 0; sin(a) cos(a) 0 ; 0 0 1];
R_10 = transpose(R_01);
R_12 = [1 0 0 ; 0 cos(b) -sin(b); 0 sin(b) cos(b)];
R_21 = transpose(R_12);

% Frame Attatched to Gyro Frame
R_23 = [cos(y) -sin(y) 0;sin(y) cos(y) 0; 0 0 1];
R_32 = transpose(R_23);

% Frame attatched to Rotor
R_34 = [cos(d) -sin(d) 0;sin(d) cos(d) 0; 0 0 1];
R_43 = transpose(R_34);

R_20 = R_21 * R_10;
R_30 = R_32 * R_20;

R_31 = R_32 * R_21;

%% Inertia 
I3_G_rot = mr*[1/12*(3*Ri^2+h^2), 0, 0  ; 0, 1/12*(3*Ri^2+h^2), 0 ; 0, 0, 1/2*Ri^2];
I3_G_a = mrod*[1/12*(3*r+H^2), 0, 0     ; 0, 1/12*(3*r^2+H^2), 0  ; 0, 0, 1/2*r^2];
I3_G_b = mtorus*[5/8*r^2+1/2*Ro^2, 0, 0; 0, 3/4*r^2+Ro^2, 0    ; 0, 0, 5/8*r^2+1/2*Ro^2];
I3_G_c = mtorus*[5/8*r^2+1/2*Ro^2, 0, 0; 0, 5/8*r^2+1/2*Ro^2, 0 ; 0, 0, 3/4*r^2+Ro^2];

I3_G_f = I3_G_a + I3_G_b + I3_G_c;

syms F_r_x3 F_r_y3 F_r_z3
syms F_o_x3 F_o_y3 F_o_z3
syms M_r_x3 M_r_y3 

%% Newton Euler Equation Solutions

F_r_x3 = -L*b_dd*sin(y) + L*a_dd*sin(b)*cos(y) + 2*L*a_d*b_d*cos(b)*cos(y) + L*a_d^2*cos(b)*sin(b)*sin(y);
F_r_y3 = -L*b_dd*cos(y) - L*a_dd*sin(B)*sin(y) + 2*L*a_d*b_d*cos(b)*sin(y) + L*a_d^2*sin(b)*cos(b)*cos(y);
F_r_z3 = -L*b_d^2 - L*a_d^2*sin(b)^2;



F3_r = [F_r_x3; F_r_y3;F_r_z3];
F3_O = [F_o_x3; F_o_y3;F_o_z3];
M3_r = [M_r_x3; M_r_y3;0];





