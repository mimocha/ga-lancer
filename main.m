% Revision 0.01.0
% Class Work - Completed: 11.24.2017
% Final version from textbook -- based on example "mobile10demo.m"

%% Define Variables
close all

% Loads map from variable VehicleMap (Loaded manually)
% Matrix should be in format:
% [Ax1 Ax2 ... xn ; Ay1 Ay2 ... yn ; Bx1 Bx2 ... Bxn ; By1 By2 ... Byn ];
% Where each coordinate pair (Axn, Ayn) or (Bxn, Byn) makes up one vertex
% of the road edge.
% G_MAP -> GLOBAL MAP
gmap = VehicleMap'; 

% Get starting coordinate
x = (gmap(1,1) + gmap(3,1))/2;
y = (gmap(2,1) + gmap(4,1))/2;
% Coordinate matrix for local coordinate calculations
xyxy = [x;y;x;y];
% Define trajectory matrix
trajectory = [x;y];

% Calculating theta for car and map rotation
theta = atan2(gmap(2,2)-gmap(2,1) , gmap(1,2)-gmap(1,1)) - pi/2;
% R -> Car rotation matrix
R = [cos(theta),-sin(theta) ; sin(theta),cos(theta)];
% Rm -> Map rotation matrix
Rm = [R', zeros(2) ; zeros(2), R'];

% Converts Global Map to Local Map
% L_MAP -> LOCAL MAP
lmap = Rm*(gmap - xyxy(:,ones(1,length(gmap)))); 

% Simulation Parameters
dt = 0.1;			% Simulation timestep size (seconds)
speed = 10;			% Vehicle speed
D = 0.5;			% Distance between car front - back axle
roadwidth = 3;		% Width of the road (no effect on sim)
eyeheight = 3;		% Height of POV (no effect on sim)
viewangle = 3;		% Viewing POV angle (no effect on sim)
lookahead = 3;		% Line camera reading distance (affects sim)
senseangle = 10;	% Width of line camera (affects sim)

% Converts Local Map to Near Map (Car POV)
% N_MAP -> NEAR MAP
nmap = lmap;
nmap([2,4],:) = nmap([2,4],:).*(nmap([2,4],:)>0) + 0.01;
nmap([1,3],:) = nmap([1,3],:)./nmap([2,4],:);
nmap([2,4],:) = -eyeheight./nmap([2,4],:);

% Define Car model & create new Car
Car = 	[0,0; 1,-0.5; 0,1.5; -1,-0.5; 0,0;...
		 nan,nan; -senseangle,lookahead; senseangle,lookahead]';
newCar = R*Car;

theta = theta + 25*pi/180;

%% Figure & Plotting
% Global Coordinate View
subplot(2,2,1); 
plot(gmap([1,3],:)', gmap([2,4],:)'); 
axis equal
hold on;
hnewcar1 = plot(newCar(1,:)+x, newCar(2,:)+y,...
				trajectory(1,:), trajectory(2,:), 'r:');
hold off;

% Local Coordinate View
subplot(2,2,2); 
hlmap2 = plot(lmap([1,3],:)', lmap([2,4],:)'); 
axis equal
hold on;
plot(Car(1,:), Car(2,:));
hold off;
grid;

% Car Point of View
subplot(2,2,3);
hlmap3 = plot(nmap([1,3],:)', nmap([2,4],:)');
set(gca, 'xlim', viewangle*roadwidth*[-0.5,0.5],...
		 'ylim', [-eyeheight,0]);


%% Simulation Loop
while (1)
	% Calculates angle & position of Car, relocates and rotates world
	R = [cos(theta),-sin(theta) ; sin(theta),cos(theta)];
	newCar = R*Car;
	Rm = [R', zeros(2) ; zeros(2), R'];
	xyxy = [x;y;x;y];
	lmap = Rm*(gmap - xyxy(:,ones(1,length(gmap))));

	% Update global & local map
	set(hlmap2(1), 'xdata', lmap(1,:), 'ydata', lmap(2,:));
	set(hlmap2(2), 'xdata', lmap(3,:), 'ydata', lmap(4,:));
	set(hnewcar1(1), 'xdata', newCar(1,:)+x, 'ydata', newCar(2,:)+y);
	
	% Updates Car POV
	nmap = lmap;
	nmap([2,4],:) = nmap([2,4],:).*(nmap([2,4],:)>0) + 0.01;
	nmap([1,3],:) = nmap([1,3],:)./nmap([2,4],:);
	nmap([2,4],:) = -eyeheight./nmap([2,4],:);
	set(hlmap3(1), 'xdata', nmap(1,:), 'ydata', nmap(2,:));
	set(hlmap3(2), 'xdata', nmap(3,:), 'ydata', nmap(4,:));

	% Splits the track into left and right edge
	viewmap12 = lmap(1:2,:); % BLU LINE (Right)
	viewmap34 = lmap(3:4,:); % RED LINE (Left)
	linesense = [-senseangle,senseangle ; lookahead,lookahead];

	% Checks only vertices within X radius (senseangle - Sensor Width)
	viewmap12 = viewmap12(:, abs(viewmap12(1,:)) < senseangle );
	viewmap34 = viewmap34(:, abs(viewmap34(1,:)) < senseangle );

	% Gets the closest and furthest vertices of each line, by checking the
	% Y coordinate distance, lookahead. In effect, this function gets the
	% next vertex in front of the line camera, and the vertex just behind.
	% Will cause error if any step returns a null value; as linsenseind
	% interpolation requires atleast 2 pairs of vertices.
	% 12 MIN
	ind = find(viewmap12(2,:) >= lookahead);
	[~,mind] = min(viewmap12(2,ind));
	cross12 = viewmap12(:,ind(mind));
	% 12 MAX
	ind = find(viewmap12(2,:) < lookahead);
	[~,mind] = max(viewmap12(2,ind));
	cross12 = [cross12,viewmap12(:,ind(mind))];
	% 34 MIN
	ind = find(viewmap34(2,:) >= lookahead);
	[~,mind] = min(viewmap34(2,ind));
	cross34 = viewmap34(:,ind(mind));
	% 34 MAX
	ind = find(viewmap34(2,:) < lookahead);
	[~,mind] = max(viewmap34(2,ind));
	cross34 = [cross34,viewmap34(:,ind(mind))];

	% Checks if the previous function returned a value,
	% and used the interpolation function interpl() to find where the lines
	% intersected. (Line camera and road edge)
	if (~isempty(cross12(1,:)))
		% ~isempty() (~ for NOT) runs faster than checking length(>0)
		linesenseind = interp1(cross12(2,:), cross12(1,:), lookahead);
	else
		linesenseind = [];
	end

	if (~isempty(cross34(1,:)))
		% ~isempty() (~ for NOT) runs faster than checking length(>0)
		linesenseind = [linesenseind, interp1(cross34(2,:), cross34(1,:), lookahead)];
	end

	% If line crossed on atleast one side, plot sensor data output.
	% Would cause error if only one side of the lines crossed.
	if (~isempty(linesenseind))
		subplot(2,2,4);
		textx = [linesenseind(2),...
				 mean(linesenseind),...
				 linesenseind(1)];
		texty = [1, 0.1, 1];
		texts = {string(round(textx(1),3)),...
				 string(round(textx(2),3)),...
				 string(round(textx(3),3))};
		plot(linesenseind([1,1],:), [0.9,0.9;0,0], 'o',...
			 textx(2), 0, 'rp');
		axis([-10,10,0,1.2]);
		line(linesenseind([1,1],:), [0.9,0.9;0,0]);
		text(textx, texty, texts,...
			'HorizontalAlignment', 'Center')
	end
	
	% If lines did not cross, end simulation
	if (isempty(linesenseind))
		break;
	end
	
	% Determines the steering by taking the average of the sensor output.
	% Updates the car, and plot its trajectory.
	steering = -mean(linesenseind)*10;
	theta = theta + speed*dt/D*tan(steering*pi/180);
	x = x + speed*dt*cos(theta + pi/2);
	y = y + speed*dt*sin(theta + pi/2);
	trajectory = [trajectory,[x;y]];
	set(hnewcar1(2), 'xdata', trajectory(1,:), 'ydata', trajectory(2,:));
	
	% Draw, repeat.
	drawnow; 
	%pause(0.1);
end