clear all
close all

% Q25 -----------------------------------------------
syms t a(t) b(t) y(t) d(t)
syms F_Gx F_Gy F_Gz 

syms m_r R_i h R_0 H L r g rho m_a m_b m_c m_f


m_a = rho*pi()*r^2*H;
m_b = 2*rho*pi()^2*r^2*R_0;
m_c = m_b;

m_f = 0.023;
R_i = 3.5;
R_0 = 4*10^-2;
r = .5*10^-2;
H = 10*10^-2;
L = .5 * H;
h = .5*10^-2;

% solve in terms of rho
rho_f = solve(m_f == m_a + m_b + m_c,rho)
vpa(subs(rho_f))

m_a = rho_f*pi()*r^2*H;
m_b = 2*rho_f*pi()^2*r^2*R_0;
m_c = m_b;
vpa(subs(m_a + m_b + m_c))




