clc 
clear all

syms a(t) b(t) g(t) T(t) x(t) y(t) % alpha, beta, gamma, Theta, x, y
syms da db dg  R

%T = 0;  % Angle Theta

aa = diff(a(t),t);
bb = diff(b(t),t);
gg = diff(g(t),t);



aaa = diff(aa,t);
bbb = diff(bb,t);
ggg = diff(gg,t);


% Rotation Matrices
R_01 = [cos(T) 0 -sin(T); 0 1 0; sin(T) 0 cos(T)];
R_12 = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
R_23 = [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)];
R_34 = [1 0 0; 0 cos(g) -sin(g); 0 sin(g) cos(g)];

R_10 = transpose(R_01);
R_21 = transpose(R_12);
R_32 = transpose(R_23);
R_43 = transpose(R_34);

% Angular Velocities
w0_10 = [0;0;0];
w1_21 = [0;0;aa];
w2_32 = [0;bb;00];
w3_43 = [gg;0;0];

w1_10 = w0_10;
w2_21 = w1_21;
w3_32 = w2_32;
w4_43 = w3_43;

w4_40 = simplify(w4_43 + R_43*w3_32 + R_43*R_32*w2_21 +R_43*R_32*R_21*w1_10)

syms rx ry rz
%h = R_01*R_12*R_23*[0;0;R] % Check R value
r1_OA = [x;y;R*cos(b)]; % Define x and Y as the COM of Disk
r3_AC = [0;0;-R];
r1_AC = R_12*R_23*r3_AC
r4_AC = R_43*r3_AC
r4_AC = r4_AC(t);
r1_AC = simplify(R_12*R_23*R_34*r4_AC);
r4_AP = [rx;ry;rz]; % Constants

rr1_OA = diff(r1_OA)
rr4_AP = simplify(diff(r4_AP,t) + cross(w4_40,r4_AP))
rr1_AP = simplify(R_12*R_23*R_34*rr4_AP)

r1_OC = r1_OA + r1_AC
rr1_OC = simplify(subs(rr1_OA+rr1_AP,{'rx','ry','rz'},{r4_AC(1),r4_AC(2),r4_AC(3)}))

subs(rr1_OC,{a(t),b(t),g(t),x(t),y(t),T(t)...
    diff(a(t), t),diff(b(t), t), diff(g(t), t),diff(x(t), t), diff(y(t), t),diff(T(t), t)...
   diff(a(t), t, t),diff(b(t), t, t), diff(g(t), t, t),diff(x(t), t, t), diff(y(t), t, t),diff(T(t), t, t)},...
     {'a','b','g','x','y','T','aa','bb','gg','xx','yy',0,'aaa','bbb','ggg','xxx','yyy',0})