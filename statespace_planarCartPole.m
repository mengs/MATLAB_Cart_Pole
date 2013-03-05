function dx =  statespace_planarCartPole(x,u,systemParams)

if (nargin < 3)
    load planarCartPoleParameters.mat
else
    if (~isempty(systemParams))
        lp = systemParams.lp;
        Ip = systemParams.Ip;
        mc = systemParams.mc;
        mp = systemParams.mp;
        g = systemParams.g;
    else
        load planarCartPoleParameters.mat
    end
end
if (nargin < 2)
    u = 0;
end
if (nargin < 1)
    error('There must be at least one input argument: state x');
end


% Generalized Coordinates
q = x(1:2);
% Velocity of Generalized Coordinates
dq = x(3:4);

xc = q(1); % position of cart
theta = q(2); % pole angle w.r.t. gravity

dxc = dq(1); % velocity of cart
dtheta = dq(2); % pole angular velocity


% Mass/Inertia Matrix
M = [   [           mc + mp, -lp*mp*cos(theta)]
        [ -lp*mp*cos(theta),      mp*lp^2 + Ip]  ];
        
% Coriolis and Centrifugal matrix    
C = [   [ 0, dtheta*lp*mp*sin(theta)]
        [ 0,                       0]    ];
    
% Vector of Gravitational Forces    
G = [                    0
        -g*lp*mp*sin(theta)     ];
  
% Force Input to the system with saturation
uMax = 500;
if abs(u) > uMax,
    u = uMax * sign(u);
end
U = [u; 0];   

% Viscous Friction Force
Dv = 0.1; % N/m/s
D = [Dv*dxc; 0];


% Acceleration of the generalized coordinates
ddq = M \ ( U + D - G - C*dq );


% Check fall, stop simulation if the pole hits the floor
if (abs(theta) >= pi/2)
    dq = 0*dq;
    ddq = 0*ddq;
end

% Return derivate of state, i.e., velocity and acceleration
dx = [dq; ddq];               

