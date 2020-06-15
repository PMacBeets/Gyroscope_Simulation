clear all
close all
clc

syms t;

%% DO I WANT TO RECORD THE VIDEO
    VIDEO = 1;

%% SETUP THE PROBLEM
% TODO: initial conditions, time duration
x_init = [0; .1; .1; 1.; 0;0;0;50];                      % initial conditions
tspan = [0 8];                                  % start and finish

options = odeset('RelTol',1e-7,'AbsTol',1e-7);  % solver options
sol = ode45(@eom, tspan, x_init, options);      % SOLVE the eoms


%% EVAULATE THE SOLUTION
dt = 0.03;                                      % set time step
t = tspan(1) : dt : tspan(2);                   % creat time vector
X = deval(sol,t);                               % deval
%
% not necessary?

 %% PLOT THE STATES
    plot(t,X)
    xlabel('time')
    ylabel('states')
    h = legend('$\alpha$','$\beta$','$\gamma$','$\delta$','$\dot{\alpha}$','$\dot{\beta}$','$\dot{\gamma}$','$\dot{\delta}$');
    set(h,'Interpreter','latex')

%% CREATE Gyroscope
    figure();
% Horizontal Toroid
    tz_centre = 15;
    t1_R = 10; % outer radius of torus
    t1_r = 1; % inner tube radius

    t1_theta = linspace(0,2*pi,72);
    t1_phi = linspace(0,2*pi,36);
    [t1_Phi,t1_Theta] = meshgrid(t1_phi,t1_theta);

    t1_x = (t1_R + t1_r.*cos(t1_Theta)).*cos(t1_Phi);
    t1_y = (t1_R + t1_r.*cos(t1_Theta)).*sin(t1_Phi);
    t1_z = tz_centre + t1_r.*sin(t1_Theta);

    %surf(t1_x, t1_y, t1_z); % plot surface toroid 1
    hold on

% Vertical Toroid
    t2_R = 12; % outer radius of torus
    t2_r = 1; % inner tube radius

    t2_theta = linspace(0,2*pi,72);
    t2_phi = linspace(0,2*pi,36);
    [t2_Phi,t2_Theta] = meshgrid(t2_phi,t2_theta);

    t2_x = (t2_R + t2_r.*cos(t2_Theta)).*cos(t2_Phi);
    t2_y =  t2_r.*sin(t2_Theta);
    t2_z = tz_centre + (t2_R + t2_r.*cos(t2_Theta)).*sin(t2_Phi);

    surf(t2_x, t2_y, t2_z); % plot surface toroid 2
    hold on

% Rotor
    disc_h = 1;
    disc_R = 7;
    theta = 0:0.01:2*pi;

    % upper disc
    disc1_x = disc_R*cos(theta);
    disc1_y = disc_R*sin(theta);
    disc1_z = tz_centre + (zeros(size(disc1_x)) + disc_h);
    fill3(disc1_x,disc1_y,disc1_z,disc1_x);
    hold on

    % lower disc
    disc2_x = disc_R*cos(theta);
    disc2_y = disc_R*sin(theta);
    disc2_z = tz_centre - (zeros(size(disc2_x)) + disc_h);
    fill3(disc2_x,disc2_y,disc2_z,disc2_x);
    hold on

    % rotor side
    [disc3_x, disc3_y, disc3_z] = cylinder(ones(1,disc_h).*disc_R) ;
    disc3_z = tz_centre + disc_h*(2*disc3_z - 1); %test this
    surf(disc3_x, disc3_y, disc3_z);
    hold on

% Axle components
    axle_R = 0.5;

% outside frame
    oAxle_h = 3;
    t2_max = t2_R + t2_r;
    %upper
    [oAxle1_x, oAxle1_y, oAxle1_z] = cylinder(ones(1,oAxle_h).*axle_R);
    oAxle1_z = tz_centre + (oAxle_h*oAxle1_z + t2_max);
    surf(oAxle1_x, oAxle1_y, oAxle1_z);
    hold on

    %lower
    [oAxle2_x, oAxle2_y, oAxle2_z] = cylinder(ones(1,oAxle_h).*axle_R);
    oAxle2_z = tz_centre - (oAxle_h*oAxle2_z + t2_max);
    surf(oAxle2_x, oAxle2_y, oAxle2_z);
    hold on

% inside frame
% height b/w vertical toroid(t2) and disc h height, then tz_centre shift
    inAxle_h = (t2_R-t2_r) - disc_h;
    % upper
    [inAxle1_x, inAxle1_y, inAxle1_z] = cylinder(ones(1,inAxle_h).*axle_R);
    inAxle1_z = tz_centre + (inAxle_h*inAxle1_z + disc_h);
    surf(inAxle1_x, inAxle1_y, inAxle1_z);
    hold on;

    % lower
    [inAxle2_x, inAxle2_y, inAxle2_z] = cylinder(ones(1,inAxle_h).*axle_R);
    inAxle2_z = tz_centre - (inAxle_h*inAxle2_z + disc_h);
    surf(inAxle2_x, inAxle2_y, inAxle2_z);
    hold on;


%% ---------------------------------------------------------------------------
 %% SETUP VIDEO IF REQUIRED
    if VIDEO
        fps = 1/dt;
        MyVideo = VideoWriter('Gyro_animation','MPEG-4');
        MyVideo.FrameRate = fps;
        open(MyVideo);
    end
    
%% CREATE ANIMATION
handle = figure;
hold on % ; grid on
for i = 1:length(t)
    cla

    % TODO: EOM to angle
    a = X(1,i);
    b =  X(2,i);
    y = X(3,i);
    d = X(4,i);

    % TODO: Rotations Matrices
    Rx = @(t) [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)];
    Ry = @(t) [cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)];
    Rz = @(t) [cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];

    R01 = Rz(a);
    R12 = Rx(b);
    R23 = Rz(y);
    R34 = Rz(d);

    R03 = R01*R12*R23;
    R04 = R03*R34;
    
    %% Rotation Matrices
    R_01 = [cos(a) -sin(a) 0; sin(a) cos(a) 0 ; 0 0 1];
    %R_10 = transpose(R_01);
    R_12 = [1 0 0 ; 0 cos(b) -sin(b); 0 sin(b) cos(b)];
    %R_21 = transpose(R_12);

    % Frame Attatched to Gyro Frame
    R_23 = [cos(y) -sin(y) 0;sin(y) cos(y) 0; 0 0 1];
    %R_32 = transpose(R_23);

    % Frame attatched to Rotor
    R_34 = [cos(d) -sin(d) 0;sin(d) cos(d) 0; 0 0 1];
    %R_43 = transpose(R_34);


    %R03 = R_01 * R_12 * R_23;
    %R40 = R03 * R_34;


    % TODO: Applying Rotation Function

        % Frame (torus and outer axles)
    [t1_x_rot, t1_y_rot, t1_z_rot] = rotation(t1_x, t1_y, t1_z, R03);
    [t2_x_rot, t2_y_rot, t2_z_rot] = rotation(t2_x, t2_y, t2_z, R03);
    [oAxle1_x_rot, oAxle1_y_rot, oAxle1_z_rot] = rotation(oAxle1_x, oAxle1_y, oAxle1_z, R03);
    [oAxle2_x_rot, oAxle2_y_rot, oAxle2_z_rot] = rotation(oAxle2_x, oAxle2_y, oAxle2_z, R03);

        % Rotor (disk + inner axles)
    [inAxle1_x_rot, inAxle1_y_rot, inAxle1_z_rot] = rotation(inAxle1_x, inAxle1_y, inAxle1_z, R04);
    [inAxle2_x_rot, inAxle2_y_rot, inAxle2_z_rot] = rotation(inAxle2_x, inAxle2_y, inAxle2_z, R04);
    [disc1_x_rot, disc1_y_rot, disc1_z_rot] = rotation(disc1_x, disc1_y, disc1_z, R04);
    [disc2_x_rot, disc2_y_rot, disc2_z_rot] = rotation(disc2_x, disc2_y, disc2_z, R04);
    [disc3_x_rot, disc3_y_rot, disc3_z_rot] = rotation(disc3_x, disc3_y, disc3_z, R04);

    % TODO: Generating the Shape
    surf(t1_x_rot, t1_x_rot, t1_z_rot); hold on
    surf(t2_x_rot, t2_y_rot, t2_z_rot); hold on
    surf(disc3_x_rot, disc3_y_rot, disc3_z_rot); hold on
    surf(oAxle1_x_rot, oAxle1_y_rot, oAxle1_z_rot); hold on
    surf(oAxle2_x_rot, oAxle2_y_rot, oAxle2_z_rot); hold on
    surf(inAxle1_x_rot, inAxle1_y_rot, inAxle1_z_rot); hold on
    surf(inAxle2_x_rot, inAxle2_y_rot, inAxle2_z_rot); hold on

    fill3(disc1_x_rot,disc1_y_rot,disc1_z_rot,disc1_x_rot); hold on
    fill3(disc2_x_rot,disc2_y_rot,disc2_z_rot,disc2_x_rot); hold on

    % Graphing axis stuff
    axis square
    view(3)
    axis(20*[-1 1 -1 1 -1 1])
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    
        if VIDEO
            writeVideo(MyVideo,getframe(handle));
        else
            pause(dt)
        end
 end

    if VIDEO
    close(MyVideo)
    end



%% -------------------------- Functions ---------------------------------------
function [Xf,Yf,Zf]=rotation(Xi,Yi,Zi,R)

    I=size(Xi,1);
    J=size(Xi,2);

    Xf=zeros(I,J);
    Yf=zeros(I,J);
    Zf=zeros(I,J);

    for ii=1:I
        for jj=1:J
            vector=[Xi(ii,jj);Yi(ii,jj);Zi(ii,jj)];
            vector=R*vector;
                Xf(ii,jj)=vector(1);
                Yf(ii,jj)=vector(2);
                Zf(ii,jj)=vector(3);
        end
    end

end



function xdot = eom(t,x)

% for ode 45
    a = x(1);
    b = x(2);
    y = x(3);
    d = x(4);
    a_d = x(5);
    b_d = x(6);
    y_d = x(7);
    d_d = x(8);

% constants for equarions of motion

% equations of motion separated in num and denom

%a  %alpha_dd_num = ;
    %alpha_dd_den = ;
    a_dd = (1688849860263936*cos(y)*(((18015*pi)/2 + 66825912621352060825/562949953421312)*(b_d*y_d*cos(y) - a_d*b_d*cos(b)*cos(y) + a_d*y_d*sin(b)*sin(y)) - 7350*sin(b)*sin(y) + (825*b_d*d_d*cos(y))/2 + 900*b_d*y_d*cos(y) + (y_d + a_d*cos(b))*(b_d*cos(y) + a_d*sin(b)*sin(y))*(15*pi + 16821072580431790825/562949953421312) - ((18015*pi)/2 + 30627964525063665375/281474976710656)*(y_d + a_d*cos(b))*(b_d*cos(y) + a_d*sin(b)*sin(y)) - sin(b)*sin(y)*(4410*pi + 2041864301670911025/35184372088832) - (21675*a_d^2*cos(b)*sin(b)*sin(y))/2 - 22575*a_d*b_d*cos(b)*cos(y) + (825*a_d*d_d*sin(b)*sin(y))/2 + 900*a_d*y_d*sin(b)*sin(y)))/(sin(b)*(15212315116327403520*pi + 220300613098904131275)) + (844424930131968*sin(y)*(7350*sin(b)*cos(y) - ((18015*pi)/2 + 30627964525063665375/281474976710656)*(a_d*b_d*cos(b)*sin(y) - b_d*y_d*sin(y) + a_d*y_d*sin(b)*cos(y)) + (825*b_d*d_d*sin(y))/2 + 900*b_d*y_d*sin(y) + (y_d + a_d*cos(b))*(b_d*sin(y) - a_d*sin(b)*cos(y))*(15*pi + 16821072580431790825/562949953421312) - (y_d + a_d*cos(b))*((18015*pi)/2 + 66825912621352060825/562949953421312)*(b_d*sin(y) - a_d*sin(b)*cos(y)) + sin(b)*cos(y)*(4410*pi + 2041864301670911025/35184372088832) + (21675*a_d^2*cos(b)*sin(b)*cos(y))/2 - 22575*a_d*b_d*cos(b)*sin(y) - (825*a_d*d_d*sin(b)*cos(y))/2 - 900*a_d*y_d*sin(b)*cos(y)))/(sin(b)*(7606157558163701760*pi + 101795331192614970525));


    %beta_dd_num = ;
    %beta_dd_den = ;
    b_dd = (844424930131968*cos(y)*(7350*sin(b)*cos(y) - ((18015*pi)/2 + 30627964525063665375/281474976710656)*(a_d*b_d*cos(b)*sin(y) - b_d*y_d*sin(y) + a_d*y_d*sin(b)*cos(y)) + (825*b_d*d_d*sin(y))/2 + 900*b_d*y_d*sin(y) + (y_d + a_d*cos(b))*(b_d*sin(y) - a_d*sin(b)*cos(y))*(15*pi + 16821072580431790825/562949953421312) - (y_d + a_d*cos(b))*((18015*pi)/2 + 66825912621352060825/562949953421312)*(b_d*sin(y) - a_d*sin(b)*cos(y)) + sin(b)*cos(y)*(4410*pi + 2041864301670911025/35184372088832) + (21675*a_d^2*cos(b)*sin(b)*cos(y))/2 - 22575*a_d*b_d*cos(b)*sin(y) - (825*a_d*d_d*sin(b)*cos(y))/2 - 900*a_d*y_d*sin(b)*cos(y)))/(7606157558163701760*pi + 101795331192614970525) - (1688849860263936*sin(y)*(((18015*pi)/2 + 66825912621352060825/562949953421312)*(b_d*y_d*cos(y) - a_d*b_d*cos(b)*cos(y) + a_d*y_d*sin(b)*sin(y)) - 7350*sin(b)*sin(y) + (825*b_d*d_d*cos(y))/2 + 900*b_d*y_d*cos(y) + (y_d + a_d*cos(b))*(b_d*cos(y) + a_d*sin(b)*sin(y))*(15*pi + 16821072580431790825/562949953421312) - ((18015*pi)/2 + 30627964525063665375/281474976710656)*(y_d + a_d*cos(b))*(b_d*cos(y) + a_d*sin(b)*sin(y)) - sin(b)*sin(y)*(4410*pi + 2041864301670911025/35184372088832) - (21675*a_d^2*cos(b)*sin(b)*sin(y))/2 - 22575*a_d*b_d*cos(b)*cos(y) + (825*a_d*d_d*sin(b)*sin(y))/2 + 900*a_d*y_d*sin(b)*sin(y)))/(15212315116327403520*pi + 220300613098904131275);


    %gamma_dd_num = ;
    %gamma_dd_den = ;
    y_dd = (1113996714244946015*b_d^2*sin(2*y) - 1113996714244946015*a_d^2*sin(2*y) + 8956422460662608360*a_d*b_d*sin(b) + 2227993428489892030*a_d^2*cos(b)^2*cos(y)*sin(y) - 4455986856979784060*a_d*b_d*sin(b)*cos(y)^2 + 3377699720527872*pi*a_d*b_d*sin(b))/(3377699720527872*pi + 6728429032172716330) - (1688849860263936*cos(b)*cos(y)*(((18015*pi)/2 + 66825912621352060825/562949953421312)*(b_d*y_d*cos(y) - a_d*b_d*cos(b)*cos(y) + a_d*y_d*sin(b)*sin(y)) - 7350*sin(b)*sin(y) + (825*b_d*d_d*cos(y))/2 + 900*b_d*y_d*cos(y) + (y_d + a_d*cos(b))*(b_d*cos(y) + a_d*sin(b)*sin(y))*(15*pi + 16821072580431790825/562949953421312) - ((18015*pi)/2 + 30627964525063665375/281474976710656)*(y_d + a_d*cos(b))*(b_d*cos(y) + a_d*sin(b)*sin(y)) - sin(b)*sin(y)*(4410*pi + 2041864301670911025/35184372088832) - (21675*a_d^2*cos(b)*sin(b)*sin(y))/2 - 22575*a_d*b_d*cos(b)*cos(y) + (825*a_d*d_d*sin(b)*sin(y))/2 + 900*a_d*y_d*sin(b)*sin(y)))/(sin(b)*(15212315116327403520*pi + 220300613098904131275)) - (844424930131968*cos(b)*sin(y)*(7350*sin(b)*cos(y) - ((18015*pi)/2 + 30627964525063665375/281474976710656)*(a_d*b_d*cos(b)*sin(y) - b_d*y_d*sin(y) + a_d*y_d*sin(b)*cos(y)) + (825*b_d*d_d*sin(y))/2 + 900*b_d*y_d*sin(y) + (y_d + a_d*cos(b))*(b_d*sin(y) - a_d*sin(b)*cos(y))*(15*pi + 16821072580431790825/562949953421312) - (y_d + a_d*cos(b))*((18015*pi)/2 + 66825912621352060825/562949953421312)*(b_d*sin(y) - a_d*sin(b)*cos(y)) + sin(b)*cos(y)*(4410*pi + 2041864301670911025/35184372088832) + (21675*a_d^2*cos(b)*sin(b)*cos(y))/2 - 22575*a_d*b_d*cos(b)*sin(y) - (825*a_d*d_d*sin(b)*cos(y))/2 - 900*a_d*y_d*sin(b)*cos(y)))/(sin(b)*(7606157558163701760*pi + 101795331192614970525));



    %delta_dd_num = ;
    %delta_dd_den = ;
    d_dd = -(1113996714244946015*((b_d^2*sin(2*y))/2 - a_d^2*sin(b)^2*cos(y)*sin(y) - a_d*b_d*sin(b)*cos(y)^2 + a_d*b_d*sin(b)*sin(y)^2))/(1688849860263936*pi + 3364214516086358165);


% matrix of xdots to be used
    xdot = [a_d;
            b_d;
            y_d;
            d_d;
            a_dd;
            b_dd;
            y_dd;
            d_dd];
end
