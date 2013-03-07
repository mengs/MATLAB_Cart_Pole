clear; % clear all variables in the workspace
close all; % close all figure windows
clc; % clear screen

% load the system parameters for the planar cart pole model
load planarCartPoleParameters.mat

% planar cart pole model used for simulation
modelName = 'statespace_planarCartPole';


% time settings
T = 0.002; % time step
ti = 0; % initial time
tf = 10; % final time --> CHANGE THIS TO SIMULATE FOR LONGER OR SHORTER TIMES

% Time vector
t = ti:T:tf;

% Initial Condition
% State: 
% cart position (m), 
% pole angle (rad), 
% cart velocity (m/s), 
% pole angular velocity (rad/s)
x0 = [ 0; 0; 0; 0 ]; % start from rest

% PD control gains --> CHANGE THESE GAINS FOR YOUR CONTROLLER
Kp_c = 57.455; % P gain for cart
Kd_c = 1; % D gain for cart
Kp_p = 1e4*1.0375; % P gain for pole
Kd_p = 10.1; % D gain for pole


% Assign initial condition for the state
x(:,1) = x0;
% dx = zeros(4,length(t));
% Des = zeros(2,length(t));
% Errs = zeros(2,length(t));
% u = zeros(1,length(t));
s = 0;

% Simulate
for k=1:1:length(t),
    
    % Desired trajectories 
    % --> EDIT THIS FUNCTION TO CHANGE THE DESIRED TRAJECTORIES
%     xDes(:,k) = getDesiredTrajectory_planarCartPole(t(k));
    [Des(:,k), s] = getDesiredTrajectory_planarCartPole(x(:,k), s);

    % Control law (PD Control)
    % --> EDIT THIS, IF NECESSARY
%     u(:,k) = [Kp_c, Kp_p, Kd_c, Kd_p] * (xDes(:,k) - x(:,k));
    Errs(:,k) = Des(:,k) - x([3 2],k);
    if k == 1
        dErrs = [0;0];
    else
        dErrs = (Errs(:,k)-Errs(:,k-1)) / T;
    end
    u(k) = Kp_c*Errs(1,k) + Kd_c*dErrs(1) + Kp_p*Errs(2,k) + Kd_p*dErrs(2);

    % Runge-Kutta RK45 numerical integration
    k1 = feval(modelName,x(:,k),u(:,k))*T;
    k2 = feval(modelName,x(:,k)+k1/2,u(:,k))*T;
    k3 = feval(modelName,x(:,k)+k2/2,u(:,k))*T;
    k4 = feval(modelName,x(:,k)+k3,u(:,k))*T;

    % Vector of derivative of state
    dx(:,k) = (k1+2*k2+2*k3+k4)/(6*T);
    
    % State vector
    if (k < length(t))
        x(:,k+1) = x(:,k) + dx(:,k)*T;
    end

end   


% Visualize the planar cart pole
speedFactor = 50; % speed of visualization
systemParams = []; % load default system parameters
visualize_planarCartPole(t,x,systemParams,speedFactor);
