% Revision 0.02.0
% Class Work - Completed: 12.15.2017
% Self improved versions, starting from this revision.
%
% Changes:
% Added new variables IntGR IntGL IntLR IntLL for intersection coordinates
% Renamed [lookahead -> sensorY] | [senseangle -> sensorX]
% Aesthetically reformatted Car Model definition
% Pointers for updating axis objects
% Predefines sensor data plot
% Overhauled sensor calculations:
%	Fixed bug where simulation ends if tracks end or break
%	Fixed with line intersection algorithm: polyxpoly()
%	Also optimized and changed code to use pointers instead

clear
close all

%% Define Variables

% Loads map automatically from file gmap.mat
% Matrix should be in format:
% [Ax1 ... xn ; Ay1 ... yn ; Bx1 ... Bxn ; By1 ... Byn ];
% Where each coordinate pair (Axn, Ayn) or (Bxn, Byn) makes up one vertex
% of the road edge.

% gmap = VehicleMap';
load('gmap.mat') % G_MAP -> GLOBAL MAP
x = (gmap(1,1) + gmap(3,1))/2; % Get starting coordinate
y = (gmap(2,1) + gmap(4,1))/2; % Get starting coordinate
xyxy = [x;y;x;y]; % Coordinate matrix for local coordinate calculations
trajectory = [x;y]; % Define trajectory matrix
IntGR = [0 0]; % Intersect Global Right
IntGL = [0 0]; % Intersect Global Left

theta = atan2(gmap(2,2)-gmap(2,1) , gmap(1,2)-gmap(1,1)) - pi/2; % Theta for car and map rotation
R = [cos(theta),-sin(theta) ; sin(theta),cos(theta)]; % R -> Car rotation matrix
Rm = [R', zeros(2) ; zeros(2), R']; % Rm -> Map rotation matrix

% Converts Global Map to Local Map
lmap = Rm*(gmap - xyxy(:,ones(1,length(gmap)))); % L_MAP -> LOCAL MAP
IntLR = [0 0]; % Intersect Local Right
IntLL = [0 0]; % Intersect Local Left

% Simulation Parameters
dt = 0.05;			% Simulation timestep size (seconds)
speed = 5;			% Vehicle speed
D = 0.5;			% Distance between car front - back axle
roadwidth = 3;		% Width of the road (no effect on sim)
eyeheight = 3;		% Height of POV (no effect on sim)
viewangle = 3;		% Viewing POV angle (no effect on sim)
sensorY = 3;	% lookahead -> sensorY  | Line camera reading distance (affects sim)
sensorX = 10;	% senseangle -> sensorX | Width of line camera (affects sim)

% Converts Local Map to Near Map (Car POV)
% N_MAP -> NEAR MAP
nmap = lmap;
nmap([2,4],:) = nmap([2,4],:).*(nmap([2,4],:)>0) + 0.01;
nmap([1,3],:) = nmap([1,3],:)./nmap([2,4],:);
nmap([2,4],:) = -eyeheight./nmap([2,4],:);

% Define Car model & create new Car
Car = [0    1   0   -1 0 nan -sensorX 0       nan sensorX 0;
	   0 -0.5 1.5 -0.5 0 nan  sensorY sensorY nan sensorY sensorY];
newCar = R*Car;

theta = theta + 25*pi/180;

%% Figure & Plotting
% Global Coordinate View
subplot(2,2,1); 
plot(gmap([1,3],:)', gmap([2,4],:)'); 
% PMGL/R => Pointer Marker Global Left/Right
% Use object handles (pointers) to quickly redraw them
pmgl = mapshow(IntGL(1), IntGL(2), 'DisplayType', 'Point', 'Marker', 'o');
pmgr = mapshow(IntGR(1), IntGR(2), 'DisplayType', 'Point', 'Marker', 'o');
axis equal
hold on;
hnewcar1 = plot(newCar(1,:)+x, newCar(2,:)+y,...
				trajectory(1,:), trajectory(2,:), 'r:');
hold off;

% Local Coordinate View
subplot(2,2,2); 
hlmap2 = plot(lmap([1,3],:)', lmap([2,4],:)'); 
% PMLL/R => Pointer Marker Local Left/Right
pmll = mapshow(IntLL(1), IntLL(2), 'DisplayType', 'Point', 'Marker', 'o');
pmlr = mapshow(IntLR(1), IntLR(2), 'DisplayType', 'Point', 'Marker', 'o');
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

% Sensor Data View
subplot(2,2,4);
linesenseind = [5 -5]; % Placeholder Data
hlmap4 = plot(mean(linesenseind), 0, 'rp');
set(gca, 'xlim', [-10 10], 'ylim', [0 1.2], 'xdir', 'reverse');
pl = line(linesenseind([1,1],:), [0.9,0.9;0,0], 'marker', 'o');
textx = [linesenseind(2),...
		 mean(linesenseind),...
		 linesenseind(1)];
texty = [1, 0.1, 1];
texts = {string(round(textx(1),3)),...
		 string(round(textx(2),3)),...
		 string(round(textx(3),3))};
pt = text(textx, texty, texts,...
	'HorizontalAlignment', 'Center');

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

	%% Sensor Calculations
	% Sensor format
	%		Sensor Right -> [ RightX CenterX ; RightY CenterY ]
	%		Sensor Left  -> [ LeftX  CenterX ; LeftY  CenterY ]
	sensorR = [Car(1,10), Car(1,11); Car(2,10), Car(2,11)];
	sensorL = [Car(1,7) , Car(1,8) ; Car(2,7) , Car(2,8)];
	
	% The following checks if sensor intersects with track
	%	Checks for Global & Local Track seperately
	%		(Need more elegant solution to check NULL answers)
	%	If doesn't intersect, returns value of sensor's edge
	%		ie, like sensor detects no track; return longest distance possible
	%
	%	Intersection = polyxpoly(sensorX, sensorY, mapX, mapY)
	%		Each variable is a matrix corresponding to points of a line
	%		Returns [x-pos y-pos] -> each is an N by 1 matrix
	%		N for number of intersect points
	%
	%	Length = 0
	%		-> return sensor's edge (No Intersect)
	%		Length = 1
	%			-> return Intersect (1 Intersect)
	%		Length > 1
	%			-> return first entry (Multi Intersects)
	
	% Intersects sensor with LOCAL MAP
	LENGTH_CHECK = length(polyxpoly(sensorR(1,:), sensorR(2,:), lmap(1,:), lmap(2,:))); % Checks length of results
	if (LENGTH_CHECK == 0)
		IntLR = sensorR(:,1)';
	else
		if (LENGTH_CHECK == 1)
			[IntLR(1), IntLR(2)] = polyxpoly(sensorR(1,:), sensorR(2,:), lmap(1,:), lmap(2,:));
		else
			[tmpx, tmpy] = polyxpoly(sensorR(1,:), sensorR(2,:), lmap(1,:), lmap(2,:));
			IntLR = [tmpx(1), tmpy(1)];
		end
	end
	LENGTH_CHECK = length(polyxpoly(sensorL(1,:), sensorL(2,:), lmap(3,:), lmap(4,:)));
	if (LENGTH_CHECK == 0)
		IntLL = sensorL(:,1)';
	else
		if (LENGTH_CHECK == 1)
			[IntLL(1), IntLL(2)] = polyxpoly(sensorL(1,:), sensorL(2,:), lmap(3,:), lmap(4,:));
		else
			[tmpx, tmpy] = polyxpoly(sensorL(1,:), sensorL(2,:), lmap(3,:), lmap(4,:));
			IntLL = [tmpx(1), tmpy(1)];
		end
	end
	
	sensorR = [newCar(1,10), newCar(1,11); newCar(2,10), newCar(2,11)] + [x;y];
	sensorL = [newCar(1,7) , newCar(1,8) ; newCar(2,7) , newCar(2,8)]  + [x;y];
	
	% Intersects sensor with GLOBAL MAP + [x,y] to adjust for Global Pos
	LENGTH_CHECK = length(polyxpoly(sensorR(1,:), sensorR(2,:), gmap(1,:), gmap(2,:)));
	if (LENGTH_CHECK == 0)
		IntGR = sensorR(:,1)';
	else
		if (LENGTH_CHECK == 1)
			[IntGR(1), IntGR(2)] = polyxpoly(sensorR(1,:), sensorR(2,:), gmap(1,:), gmap(2,:));
		else
			[tmpx, tmpy] = polyxpoly(sensorR(1,:), sensorR(2,:), gmap(1,:), gmap(2,:));
			IntGR = [tmpx(1), tmpy(1)];
		end
	end
	LENGTH_CHECK = length(polyxpoly(sensorL(1,:), sensorL(2,:), gmap(3,:), gmap(4,:)));
	if (LENGTH_CHECK == 0)
		IntGL = sensorL(:,1)';
	else
		if (LENGTH_CHECK == 1)
			[IntGL(1), IntGL(2)] = polyxpoly(sensorL(1,:), sensorL(2,:), gmap(3,:), gmap(4,:));
		else
			[tmpx, tmpy] = polyxpoly(sensorL(1,:), sensorL(2,:), gmap(3,:), gmap(4,:));
			IntGL = [tmpx(1), tmpy(1)];
		end
	end
	
	linesenseind = [IntLR(1) IntLL(1)];
	
	%% Updates
	% Updates Intersect marker location
	set(pmgl, 'xdata', IntGL(1), 'ydata', IntGL(2));
	set(pmgr, 'xdata', IntGR(1), 'ydata', IntGR(2));
	set(pmll, 'xdata', IntLL(1), 'ydata', IntLL(2));
	set(pmlr, 'xdata', IntLR(1), 'ydata', IntLR(2));
	
	% Update Sensor Data View
	set(hlmap4, 'xdata', mean(linesenseind)*5);
	set(pl(1), 'xdata', linesenseind([1,1]));
	set(pl(2), 'xdata', linesenseind([2,2]));
	textx = [linesenseind(2) mean(linesenseind) linesenseind(1)];
	texts = {string(round(textx(1),3)),...
			 string(round(textx(2),3)),...
			 string(round(textx(3),3))};
	set(pt(1), 'string', texts(1), 'position', [textx(1) 1]);
	set(pt(2), 'string', texts(2), 'position', [textx(2) 0.1]);
	set(pt(3), 'string', texts(3), 'position', [textx(3) 1]);
	
	% Determines the steering by taking the average of the sensor output.
	% Updates the car, and plot its trajectory.
	steering = -mean(linesenseind)*10;
	theta = theta + speed*dt/D*tan(steering*pi/180);
	x = x + speed*dt*cos(theta + pi/2);
	y = y + speed*dt*sin(theta + pi/2);
	trajectory = [trajectory,[x;y]];
	if (length(trajectory) > 1000) % Limits length of trajectory
		trajectory(:,1) = [];
	end
	set(hnewcar1(2), 'xdata', trajectory(1,:), 'ydata', trajectory(2,:));
	
	% Draw, repeat.
	drawnow;
end