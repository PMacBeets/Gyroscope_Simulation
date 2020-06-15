%% CREATE Gyroscope
    figure();
% Horizontal Toroid
    tz_centre = 0;
    t1_R = 10; % outer radius of torus
    t1_r = 1; % inner tube radius

    t1_theta = linspace(0,2*pi,72);
    t1_phi = linspace(0,2*pi,36);
    [t1_Phi,t1_Theta] = meshgrid(t1_phi,t1_theta);

    t1_x = (t1_R + t1_r.*cos(t1_Theta)).*cos(t1_Phi);
    t1_y = (t1_R + t1_r.*cos(t1_Theta)).*sin(t1_Phi);
    t1_z = tz_centre + t1_r.*sin(t1_Theta);

    surf(t1_x, t1_y, t1_z); % plot surface toroid 1
    hold on

% Vertical Toroid
    t2_R = 12; % outer radius of torus
    t2_r = 1; % inner tube radius

    t2_theta = linspace(0,2*pi,72);
    t2_phi = linspace(0,2*pi,36);
    [t2_Phi,t2_Theta] = meshgrid(t2_phi,t2_theta);

    t2_x = (t2_R + t2_r.*cos(t2_Theta)).*cos(t2_Phi);
    t2_y = tz_centre + t2_r.*sin(t2_Theta);
    t2_z = (t2_R + t2_r.*cos(t2_Theta)).*sin(t2_Phi);

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
    fill3(disc1_x,disc1_y,disc1_z,disc1_x)
    hold on

    % lower disc
    disc2_x = disc_R*cos(theta);
    disc2_y = disc_R*sin(theta);
    disc2_z = tz_centre - (zeros(size(disc2_x)) + disc_h);
    fill3(disc2_x,disc2_y,disc2_z,disc2_x)
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
    
    