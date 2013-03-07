function [Des, s] = getDesiredTrajectory_planarCartPole(x, s)

x_cart = x(1);
x_pole = x(2);
v_cart = x(3);

if s==0 && x_pole<=-pi/50 % pole cannot lean so backwards, if reach the boundry stop negative u and call next step
    s = 1;
% elseif s==0 && x_cart>=2 % when cart reach 4 meter 
%     s = 0.5;
elseif s==1 && x_cart>=4.54 % when cart reach 4 meter 
    s = 2;
elseif s==2 && x_cart>=6.02 % when cart reach 6 meter 
    s = 3;
elseif s==3 && x_cart>=10 % when cart reach 10 meter (finishing line)
    s = 4;
elseif s==4 && v_cart<=0 % the cart already finish the race stop the cart, deal with oscillating
    s = 5;
elseif s==5 && v_cart>=0
    s = 6;
end

if s == 0 % start here
    v_cart_des = -5;% negative u to make car backward 
    x_pole_des = -pi/14; % negative u  to make car backward 
% elseif s == 0.5 % start here
%     v_cart_des = 5;% negative u to make car backward 
%     x_pole_des = -pi/15; % negative u  to make car backward 

elseif s == 1 
    v_cart_des = 5; % desire forwards to get positive u to run the cart
    x_pole_des = -pi/13; % pole forwards to get positive u
elseif s == 2 
    v_cart_des = 18; % desire forwards to get positive u 
    x_pole_des = 0; % get the pole back the center
elseif s == 3
    v_cart_des = 4.8;% desire backwards to get negative u to stop the cart
    x_pole_des = pi/15;% pole backwards to get negative u
elseif s == 4 
    v_cart_des = 0; % get negative u
    x_pole_des = pi/15; % pole backwards to get negative u
elseif s == 5
    v_cart_des = 0;
    x_pole_des = -pi/180; % stop the cart from run backwards
elseif s == 6
    v_cart_des = 0;
    x_pole_des = 0;
end

% if t==0 || (x_cart<0 && v_cart<0)
%     v_cart_des = -5;
%     x_pole_des = -pi/50;
% elseif x_pole<=-pi/50 && v_cart<0
%     v_cart_des = 5;
%     x_pole_des = -pi/50;
% elseif x_cart < 4
%     v_cart_des = 5;
%     x_pole_des = -pi/50;
% elseif x_cart>=4 && x_cart<6
%     v_cart_des = 10;
%     x_pole_des = 0;
% elseif x_cart>=6 && x_cart<10
%     v_cart_des = 5;
%     x_pole_des = pi/50;
% else
%     v_cart_des = 0;
%     x_pole_des = 0;
% end

Des = [v_cart_des; x_pole_des];


% % Cart Position (m)
% if t < 100
%     xc = -0.1;
% else
%     xc = 10;
% end
% 
% % Pole Angle (rad)
% theta = 0*t;
% 
% % Cart Velocity (m/s)
% if t < 100
%     dxc = -0.5;
% else
%     dxc = 2.5;
% end
% 
% % Pole Angular Velocity (rad/s)
% dtheta = 0*t;
% 
% % Desired state
% xDes = [ xc; theta; dxc; dtheta ];