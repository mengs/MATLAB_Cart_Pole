function visualize_planarCartPole(t,x,systemParams,speedFactor,fh)

if(nargin < 5)
    fh = figure(1);
    axis([-1 12 0 1.3]);    
end

set(fh,'units','inches');
pos = get(fh,'position');
set(fh,'position',[pos(1:2),10.6,1.4]);

if(nargin < 4)
    speedFactor = 10;
end

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

set(fh,'DoubleBuffer','on');

numSteps = size(x,2);

ind = [1:speedFactor:numSteps,numSteps];

% Goal
xGoal = 10;
xMax = 12;

% Initialize time to goal
tGoal = t(end) + 1;

% Initialize time to fail
failTime = t(end) + 1;

% Failed flag
failed = 0;

% find the time at which goal is crossed
indGoal = find(x(1,:) >= xGoal);
if ~isempty(indGoal),
    tGoal = t(indGoal(1));
else
    failed = 1;
end

% find the time at which max distance is crossed
indMax = find(x(1,:) > xMax);
if ~isempty(indMax),
    failed = 1;
    failTime = t(indMax(1));
end

% find the time at which the pole falls
indFall = find(abs(x(2,:)) >= pi/2);
if ~isempty(indFall),
    failed = 1;
    if t(indFall(1)) < failTime,
        failTime = t(indFall(1));
    end
end

for j=1:1:length(ind),
    
    i = ind(j);

    clf(fh); hold off;
    axis([-1 12 0 1.3]);    
    set(gca,'Units','inches');
    set(gca,'position',[0.3,0.35,10,1]);
    xlabel('Position (m)');
        
    xc = x(1,i);
    theta = x(2,i);
    dxc = x(3,i);
    dtheta = x(4,i);
    
    % cart        
    cartLen = 0.4;
    cartHt = 0.05;
    xcart = [ xc - cartLen/2, xc + cartLen/2, xc + cartLen/2, xc - cartLen/2, xc - cartLen/2 ];
    ycart = [ 0, 0, cartHt, cartHt, 0 ];
    
    figure(fh); hold on;
    plot(xcart,ycart,'b','Linewidth',2);
    axis([-1 12 0 1.3]);    
    
    % pole
    xp = xc - lp*sin(theta);
    yp = cartHt + lp*cos(theta);
    
    plot([xc,xp],[cartHt,yp],'r','Linewidth',2);
    plot(xp,yp,'ro','MarkerSize',15,'MarkerFaceColor','r');
    axis([-1 12 0 1.3]);    
    
    % GOAL
    plot([xGoal,xGoal],[0, 1.3],'g','linewidth',2);
    text(xGoal,1.2,'Finish');
    
    % Time to cross the goal
    if ((t(i) >= tGoal) && ~failed)
        text(4,1.2,sprintf('RACE TIME: %.3f s!',tGoal));
    end
    
    drawnow;
    pause(0.1);
    
    % failure cases
    if (t(i) >= failTime)
        break;
    end
    
end

if (failed)
    text(4,1.2,'FAILED RACE!');
end