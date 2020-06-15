clear all
close all

% Q25 -----------------------------------------------
syms t a(t) b(t) y(t) d(t)

syms m_r R_i h R_0 H L r

Ir_G_rotor = m_r*[(3*R_i^2 + h^2)/12, 0 0; 0, (3*R_i^2 + h^2)/12, 0;...
    0, 0, (R_i^2)/2];

Rr3 = [cos(d) -sin(d) 0; sin(d) cos(d) 0; 0 0 1];

R3r = transpose(Rr3);

I3_G_rotor = Rr3*Ir_G_rotor*R3r;

I3_G_rotor = simplify(I3_G_rotor);

r = 1; R_0 = 10; R_i = 6; H = 30; h = 3; L = 15; m_r = 50;

% vpa(subs(I3_G_rotor))

% Q26 -------------------------------------------
rho = 1;

m_a = rho*pi()*r^2*H;

I3_G_a = m_a*[(3*r^2 + H^2)/12, 0, 0; 0, (3*r^2 + H^2)/12, 0; 0, 0, (r^2)/2];

m_b = 2*rho*pi()^2*r^2*R_0;

I3_G_b = m_b*[(5/8)*r^2 + (1/2)*R_0^2, 0, 0; 0, (3/4)*r^2 + R_0^2, 0;...
    0, 0, (5/8)*r^2 + (1/2)*R_0^2];

m_c = m_b;

I3_G_c = m_c*[(5/8)*r^2 + (1/2)*R_0^2, 0, 0; 0, (5/8)*r^2 + (1/2)*R_0^2, 0;...
    0, 0, (3/4)*r^2 + R_0^2];

I3_G_frame = I3_G_a + I3_G_b + I3_G_c;

% vpa(subs(I3_G_frame))

% Q27 ------------------------------------------------

m_f = m_a + m_b + m_c;

r3_OG = [0; 0; L];

del_x = 0;

del_y = 0;

del_z = L;

delta_matrix = [del_z^2, 0, 0; 0, del_z^2, 0; 0, 0, 0];

I3_O_frame = I3_G_frame + m_f*delta_matrix;

% vpa(subs(I3_O_frame))

% Q28 -------------------------------------------------------

g = 9.8;

syms F_Gx F_Gy F_Gz 

R10 = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
R21 = [1 0 0; 0 cos(b) -sin(b); 0 sin(b) cos(b)];
R32 = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
R01 = transpose(R10);
R12 = transpose(R21);
R23 = transpose(R32);

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

r3_OG_ddot_neat = subs(r3_OG_ddot,{a(t),b(t),y(t),...
      diff(a(t), t),diff(b(t), t),diff(y(t), t),...
      diff(a(t),t,t),diff(b(t),t,t),diff(y(t),t,t)},...
      {'a','b','y','a_d','b_d','y_d','a_dd','b_dd','y_dd'});
  
p3_OG_r = simplify(m_r*r3_OG_ddot_neat);


% Q29 -------------------------------------------------------

syms M_Gx M_Gy

M_3_g = [M_Gx; M_Gy; 0];

wr_r3 = [0; 0; diff(d)];

wr_r = wr_r3 + w3_3;

wr_r_dot = diff(wr_r);

h3_G_dot = I3_G_rotor*wr_r_dot + cross(wr_r, (I3_G_rotor*wr_r));

h3_G_dot_neat = vpa(subs(h3_G_dot,{a(t),b(t),y(t),d(t),...
      diff(a(t), t),diff(b(t), t),diff(y(t), t),diff(d(t),t),...
      diff(a(t),t,t),diff(b(t),t,t),diff(y(t),t,t),diff(d(t),t,t)},...
      {'a','b','y','d','a_d','b_d','y_d','d_d','a_dd','b_dd','y_dd','d_dd'}));


vpa(simplify(expand(subs(- M_3_g + h3_G_dot))))



% Q30 -----------------------------------------------------

syms F_Ox F_Oy F_Oz

F_3_O = [F_Ox; F_Oy; F_Oz];

G_0_f = [0; 0; -m_f*g];

G_3_f = R23*R12*R01*G_0_f;

sum_F_f = F_3_O - F_3_G + G_3_f;

p3_OG_f = simplify(m_f*r3_OG_ddot_neat);

% Q31 -------------------------------------------------------

sum_M_f = - M_3_g + cross(r3_OG, (-F_3_G)) + cross(r3_OG, G_3_f);

w3_3_dot = diff(w3_3);

h3_O_dot = I3_O_frame*w3_3_dot + cross(w3_3, (I3_O_frame*w3_3));

h3_O_dot_neat = subs(h3_O_dot,{a(t),b(t),y(t),d(t),...
      diff(a(t), t),diff(b(t), t),diff(y(t), t),diff(d(t),t),...
      diff(a(t),t,t),diff(b(t),t,t),diff(y(t),t,t),diff(d(t),t,t)},...
      {'a','b','y','d','a_d','b_d','y_d','d_d','a_dd','b_dd','y_dd','d_dd'});


vpa(h3_O_dot_neat);


% Gyro 

EQ1 = -p3_OG_r + r3_OG_ddot_neat;
EQ2 = -M_3_g + h3_G_dot_neat;
%EQ3 = -p3_OG_f + 








