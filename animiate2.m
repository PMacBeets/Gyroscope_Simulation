clc
clear all

%% Constants

 syms t;
 VIDEO = 1;

grav = 9.81
diam = 50;
R = diam/2;
m = 1;

Constants = [grav,diam,R,m];

% angles
Th = pi/10; % angle of plane
a = pi/2;
b = 0;
g = 0;

aa= 0;
bb = 0;
gg = -1;
xx = R*cos(a)*gg;
yy = R*sin(a)*gg;
%Plane Dimensions
leng = 600;  % length of plane
width = 600;  % width of plane

%Initial position of center of disk
x1 = 500;
y1 = 500;

%% Intitial Conditions comply with constraint equations

%% Solve EOM

x_init = [a;b;g;x1;y1;aa;bb;gg;xx;yy];

% IC's for clockwise (negative rotation)
% x_init = [pi/20; pi/11; pi/20;0; -pi/10;0;0;-1000];
tspan = [0 10];                                 % start and finish times
options = odeset('RelTol',1e-7,'AbsTol',1e-7);  % solver options
sol = ode45(@eom, tspan, x_init, options);      % solve the eoms
dt = 0.02;                                      % set time step
t = tspan(1) : dt : tspan(2);                   % create time vector
X = deval(sol,t);                               % evaluate solution

 %% Plotting States
    h = plot(t,X)
    xlabel('time')
    ylabel('states')
    leg = legend('$\alpha$','$\beta$','$\gamma$','$x$','$y$','$\dot{\alpha}$','$\dot{\beta}$','$\dot{\gamma}$','$\dot{x}$','$\dot{y}$');
    set(leg,'Interpreter','latex')
    set(h(1), 'color', 'b');
    set(h(2), 'color', 'r');
    set(h(3), 'color', 'c');
    set(h(4), 'color', 'g');
    set(h(5), 'color', 'm');
    set(h(6), 'color', 'b');
    set(h(6), 'LineStyle', '--');
    set(h(7), 'color', 'r');
    set(h(7), 'LineStyle', '--');
    set(h(8), 'color', 'c');
    set(h(8), 'LineStyle', '--');
    set(h(9), 'color', 'g');
    set(h(9), 'LineStyle', '--');
    set(h(10), 'color', 'm');
    set(h(10), 'LineStyle', '--');
    figure

%% Rotation Matrices

R_01 = [cos(Th) 0 -sin(Th); 0 1 0; sin(Th) 0 cos(Th)];
R_12 = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
R_23 = [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)];
R_34 = [1 0 0; 0 cos(g) -sin(g); 0 sin(g) cos(g)];

R_10 = transpose(R_01);
R_21 = transpose(R_12);
R_32 = transpose(R_23);
R_43 = transpose(R_34);

R_04 = R_01*R_12*R_23*R_34;
R_40 = transpose(R_04);


%% Plane
xp = leng*cos(Th);
yp = width;
zp = leng*sin(Th);

[X_p,Y_p] = meshgrid(1:10:xp,1:10:yp);
Z_p = X_p*sin(Th);

% %% Disk
% % Convert x & z co-ordinates from generalised co-ordinates to drawing
% % Generlaised co-ordinates; normal vector is along x
% % Drawing; normal vector is along z
normal_itit_4 = [-1;0;0];
normal = ((R_04)*normal_itit_4)';
R_04_vec = zeros(1,length(t)); %Trackrotation matrix if required
diam = 50;
% for 1r_0A

r1_OA = [x1;y1;diam*cos(b)]
center = (R_01*r1_OA)';

 %% Draw
 
 %Draw Plane
surf(X_p,Y_p,Z_p); hold on
xlabel('x')
ylabel('y')
zlabel('z')

lim = max(width,leng)
xlim([-diam lim+diam])
ylim([-diam lim+diam])
zlim([-diam lim+diam])

% Draw Disk
plotDisk3D(center,normal,diam,0)

%% Animate

    if VIDEO
        fps = 1/dt;
        MyVideo = VideoWriter('Gyro_animation_Combined','MPEG-4');
        MyVideo.FrameRate = fps;
        open(MyVideo);
    end

%% Animation Loop
handle = figure;
surf(X_p,Y_p,Z_p); hold on
xlabel('x')
ylabel('y')
zlabel('z')

lim = max(width,leng)
xlim([-diam lim+diam])
ylim([-diam lim+diam])
zlim([-diam lim+diam])
for i = 1:length(t)
    cla

    % EOM to angle
    a = X(1,i);
    b = X(2,i);
    g = X(3,i);
    x = X(4,i);
    y = X(4,i);
    
    R_01 = [cos(Th) 0 -sin(Th); 0 1 0; sin(Th) 0 cos(Th)];
    R_12 = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
    R_23 = [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)];
    R_34 = [1 0 0; 0 cos(g) -sin(g); 0 sin(g) cos(g)];
    R_04 = R_01*R_12*R_23*R_34;
    
    normal_itit_4 = [1;0;0];
    normal = (R_04*normal_itit_4)';
    
    R_04_vec(i) = R_04(2,2);

    % for 1r_0A

    r1_OA = [x;y;diam*cos(b)];
    center = ((R_01)*r1_OA)';
    
    surf(X_p,Y_p,Z_p); hold on

    plotDisk3D(center,normal,diam,g)
  
    % Graphing axis stuff
   
        if VIDEO
            writeVideo(MyVideo,getframe(handle));
        else
            pause(dt)
        end
 end

    if VIDEO
    close(MyVideo)
    end
    % notifies via terminal when recording is finished
fprintf("Animation recorded\n");


% equations of motion taken from our "ExtractEquations.m" file
function xdot = eom(t,X)
% for ode 45
    a = X(1);
    b = X(2);
    g = X(3);
    x = X(4);
    y = X(5);
    aa = X(6);
    bb = X(7);
    gg = X(8);
    xx = X(9);
    yy = X(10);
    
    %Constants = [grav,diam,R,m];
%     grav = Constants(1);
%     diam = Constants(2);
%     R = Constants(3);
%     m = Constants(4);
grav = 9.81;
diam = 50;
R = diam/2;
m = 1;
Th = pi/10;
% equations of motion

l1 = -R*aa*gg*m*sin(a); 
l2 = R*aa*gg*m*cos(a);
 
aaa = 0;
bbb = -(cos(b)*(R*sin(b)*aa^2 - 2*R*gg*aa + 4*R*sin(b)*bb^2 + 4*grav*cos(Th)*bb))/(R*(4*cos(b)^2 - 5));
ggg = -(2*(l1*cos(a) + l2*sin(a)))/(R*m);
xxx = l1/m;
yyy = l2/m;
 

 
% matrix of xdots to be used
    xdot = [aa;bb;gg;xx;yy;aaa;bbb;ggg;xxx;yyy];
end




