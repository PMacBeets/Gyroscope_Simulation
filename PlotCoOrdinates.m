 %% SETUP THE PROBLEM
    init = [pi/2; pi/2; pi/2;0; 0;0;0;5];                     % initial conditions
    tspan = [0 8];                                 % start and finish times
    options = odeset('RelTol',1e-7,'AbsTol',1e-7'); % solver options
    sol = ode45(@eom3,tspan,x_init,options);         % SOLVE the eoms

    %% EVAULATE THE SOLUTION
    dt = 0.03;                                  % set time step                        
    t = tspan(1):dt:tspan(2);                   % creat time vector
    X = deval(sol,t);                           % deval

    %% PLOT THE STATES
    plot(t,X)
    xlabel('time')
    ylabel('states')
    h = legend('$\alpha$','$\beta$','$\gamma$','$\delta$','$\dot{\alpha}$','$\dot{\beta}$','$\dot{\gamma}$','$\dot{\delta}$');
    set(h,'Interpreter','latex')