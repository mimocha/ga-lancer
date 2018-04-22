% Revision 0.03.0
% Personal Project - Completed: 01.19.2018
%
% Changes:
%	Implemented simple Feed-Forward Neural Network
%	Changed sensor type to 5 range detection
%	Merged two sides of track into one
%	Overhauled range detection algorithm
%	Removed POV view -> Replaced with Sensor Input
%	Removed Sensor Data -> Replaced with Neural Network Output
%	Overhauled Simulation Loop

clear
close all

%% Define Variables

% Loads map automatically from file MAP.mat
% Matrix should be in format:
% [Ax1 ... xn ; Ay1 ... yn ; Bx1 ... Bxn ; By1 ... Byn ];
% Where each coordinate pair (Axn, Ayn) or (Bxn, Byn) makes up one vertex
% of the road edge.

load('MAP.mat') % G_MAP -> GLOBAL MAP
x = (MAP(1,1) + MAP(3,1))/2; % Get starting coordinate
y = (MAP(2,1) + MAP(4,1))/2; % Get starting coordinate
xyxy = [x;y;x;y]; % Coordinate matrix for local coordinate calculations
trajectory = [x;y]; % Define trajectory matrix

% Neural Network Weights & Biases predefined - optimized via Tensorflow
% with backpropogation algorithm.
NNW = [[ 0.1596465   0.44959277 -0.00815194 -0.47613883 -0.16558288];
		[ 0.09106452  0.36212254  0.8983053   0.38876307  0.07883844]];
NNb = [-0.78498626; 0.00216872];

theta = atan2(MAP(2,2)-MAP(2,1) , MAP(1,2)-MAP(1,1)) - pi/2; % Theta for car and map rotation
R = [cos(theta),-sin(theta) ; sin(theta),cos(theta)]; % R -> Car rotation matrix
Rm = [R', zeros(2) ; zeros(2), R']; % Rm -> Map rotation matrix

% Converts Global Map to Local Map
lmap = Rm*(MAP - xyxy(:,ones(1,length(MAP)))); % L_MAP -> LOCAL MAP

IntG = zeros(2,5); % Global Intersect
IntL = zeros(2,5); % Local Intersect

% Simulation Parameters
dt = 0.05;				% Simulation timestep size (seconds)
D = 0.5;				% Distance between car front - back axle
sensorR = 10;			% Sensor Radius
sensorD = zeros(1,5);	% Distance from Sensor (Placeholder)

% Define Car model & create new Car
Car = [0    1   0   -1 0 nan;
	   0 -0.5 1.5 -0.5 0 nan];
CarSensor = [-sensorR 0 nan -sensorR*cosd(45) 0 nan       0 0 nan sensorR*cosd(45) 0 nan sensorR 0;
			        0 0 nan  sensorR*sind(45) 0 nan sensorR 0 nan sensorR*sind(45) 0 nan       0 0];
Car = [Car CarSensor];
CarGlobal = R*Car;

%% GUI
	%% Global Coordinate View
	subplot(2,2,1);
	plot(MAP([1,3],:)', MAP([2,4],:)');
	% PMG => Pointer Marker Global
	pmg = mapshow(IntG(1,:), IntG(2,:), 'DisplayType', 'Point', 'Marker', 'o');
	axis equal
	hold on;
	hcar = plot(CarGlobal(1,:)+x,   CarGlobal(2,:)+y,...
				 trajectory(1,:),   trajectory(2,:), 'r:');
			 % Handle Car Local
	hold off;

	%% Local Coordinate View
	subplot(2,2,2);
	hlmap = plot(lmap([1,3],:)', lmap([2,4],:)'); % Handle Local Map
	% PML => Pointer Marker Local
	pml = mapshow(IntL(:,1), IntL(:,2), 'DisplayType', 'Point', 'Marker', 'o');
	axis ([-20 20 -20 20])
	hold on;
	plot(Car(1,:), Car(2,:));
	hold off;
	grid;

	%% Sensor Data Input
	subplot(2,2,3);
	hsdat = plot((1:5), IntL(2,:), 'o'); % Handle Sensor Data
	set(gca, 'xlim', [0 6], 'ylim', [0 sensorR+5]);
	pl = line([1 1 nan 2 2 nan 3 3 nan 4 4 nan 5 5],...
			  [1 0 nan 2 0 nan 3 0 nan 4 0 nan 5 0]);
	pt = text(zeros(1,5), IntL(2,:), {'1', '2', '3', '4', '5'},...
		'HorizontalAlignment', 'Center');
	
	%% Neural Network Output
	subplot(2,2,4);
	steerplot = [-1;0];
	speedplot = [-1;0];
	hnno = plot(steerplot(1,:), steerplot(2,:), 'r-', speedplot(1,:), speedplot(2,:), 'b-'); % Handle Neural Network Output
	set(gca, 'xlim', [0 500], 'ylim', [-5 20]);
	grid on

%% Simulation Loop
while (1)
	%% World Dynamic
	% Updates world MAP and current CAR location and orientation
	
	R = [cos(theta),-sin(theta) ; sin(theta),cos(theta)];
	CarGlobal = R*Car;
	Rm = [R', zeros(2) ; zeros(2), R'];
	xyxy = [x;y;x;y];
	lmap = Rm*(MAP - xyxy(:,ones(1,length(MAP))));

	%% Sensor Intersect Stuff
	% Sensor handles intersection with polyxpoly()
	% Loops 5 times for each sensor.
	% Code to handle multiple intersection is extremely inelegant and
	% inefficient. Forgot min() max() function existed natively in MATLAB.
	% Can handle multiple intersections
	sensorG = [CarGlobal(1,7:20) ; CarGlobal(2,7:20)] + [x;y];
	sensorL = [Car(1,7:20) ; Car(2,7:20)];
	
	% The following checks if sensor intersects with track
	%	Checks for Global & Local MAP seperately
	%	If doesn't intersect, returns value of sensor's edge
	%		ie, sensor detects no track; return longest distance possible
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
	
	for i=1:5
		a = 3*i-2; % Extended end of sensor
		b = a+1;   % Origin of sensor
		%% GLOBAL MAP
		LENGTH_L = length(polyxpoly(sensorG(1,a:b), sensorG(2,a:b), MAP(1,:), MAP(2,:)));
		LENGTH_R = length(polyxpoly(sensorG(1,a:b), sensorG(2,a:b), MAP(3,:), MAP(4,:)));
		switch (LENGTH_L + LENGTH_R)
			case 0
				IntG(:,i) = sensorG(:,a);
			case 1
				if (LENGTH_L)
					[IntG(1,i), IntG(2,i)] = polyxpoly(sensorG(1,a:b), sensorG(2,a:b), MAP(1,:), MAP(2,:));
				else
					[IntG(1,i), IntG(2,i)] = polyxpoly(sensorG(1,a:b), sensorG(2,a:b), MAP(3,:), MAP(4,:));
				end
			otherwise
				%% If Intersects Multiple Points
				dmin = sensorR; % Temporary min distance counter
				imax = 0; % Temporary index
				if (LENGTH_L && LENGTH_R)
					%% IF Intersects L & R tracks
					dminl = [sensorR 0]; % Splits LR for distance
					dminr = [sensorR 0];
					[tmpx, tmpy] = polyxpoly(sensorG(1,a:b), sensorG(2,a:b), MAP(1,:), MAP(2,:));
					for j=1:LENGTH_L
						if (dminl(1) >= hypot(tmpx(j)-sensorG(1,2), tmpy(j)-sensorG(2,2)))
							dminl(1) = hypot(tmpx(j)-sensorG(1,2), tmpy(j)-sensorG(2,2));
							dminl(2) = LENGTH_L;
						end
					end
					[tmpx, tmpy] = polyxpoly(sensorG(1,a:b), sensorG(2,a:b), MAP(3,:), MAP(4,:));
					for j=1:LENGTH_R
						if (dminr(1) >= hypot(tmpx(j)-sensorG(1,2), tmpy(j)-sensorG(2,2)))
							dminr(1) = hypot(tmpx(j)-sensorG(1,2), tmpy(j)-sensorG(2,2));
							dminr(2) = LENGTH_L;
						end
					end
					% Checks both L&R then gets the shortest one
					if (dminl(1) >= dminr(1))
						dmin = dminr(1);
						imax = dminr(2);
						[tmpx, tmpy] = polyxpoly(sensorG(1,a:b), sensorG(2,a:b), MAP(3,:), MAP(4,:));
					else
						dmin = dminl(1);
						imax = dminl(2);
						[tmpx, tmpy] = polyxpoly(sensorG(1,a:b), sensorG(2,a:b), MAP(1,:), MAP(2,:));
					end
				else
					%% If Intersects Multiple L but not R
					if (LENGTH_L)
						[tmpx, tmpy] = polyxpoly(sensorG(1,a:b), sensorG(2,a:b), MAP(1,:), MAP(2,:));
						for j=1:LENGTH_L 
							if (dmin >= hypot(tmpx(j)-sensorG(1,2), tmpy(j)-sensorG(2,2))) % Gets the closest intersect
								dmin = hypot(tmpx(j)-sensorG(1,2), tmpy(j)-sensorG(2,2));
								imax = j;
							end
						end
					else
						%% If Intersects Multiple R but not L
						[tmpx, tmpy] = polyxpoly(sensorG(1,a:b), sensorG(2,a:b), MAP(3,:), MAP(4,:));
						for j=1:LENGTH_R 
							if (dmin >= hypot(tmpx(j)-sensorG(1,2), tmpy(j)-sensorG(2,2))) % Gets the closest intersect
								dmin = hypot(tmpx(j)-sensorG(1,2), tmpy(j)-sensorG(2,2));
								imax = j;
							end
						end
					end
				end
				IntG(:,i) = [tmpx(imax), tmpy(imax)];
		end
		%% LOCAL MAP
		LENGTH_L = length(polyxpoly(sensorL(1,a:b), sensorL(2,a:b), lmap(1,:), lmap(2,:)));
		LENGTH_R = length(polyxpoly(sensorL(1,a:b), sensorL(2,a:b), lmap(3,:), lmap(4,:)));
		switch (LENGTH_L + LENGTH_R)
			case 0
				IntL(:,i) = sensorL(:,a);
			case 1
				if (LENGTH_L)
					[IntL(1,i), IntL(2,i)] = polyxpoly(sensorL(1,a:b), sensorL(2,a:b), lmap(1,:), lmap(2,:));
				else
					[IntL(1,i), IntL(2,i)] = polyxpoly(sensorL(1,a:b), sensorL(2,a:b), lmap(3,:), lmap(4,:));
				end
			otherwise
				%% If Intersects Multiple Points
				dmin = sensorR; % Temporary min distance counter
				imax = 0; % Temporary index
				if (LENGTH_L && LENGTH_R)
					%% IF Intersects L & R tracks
					dminl = [sensorR 0]; % Splits LR for distance
					dminr = [sensorR 0];
					[tmpx, tmpy] = polyxpoly(sensorL(1,a:b), sensorL(2,a:b), lmap(1,:), lmap(2,:));
					for j=1:LENGTH_L
						if (dminl(1) >= hypot(tmpx(j), tmpy(j)))
							dminl(1) = hypot(tmpx(j), tmpy(j));
							dminl(2) = LENGTH_L;
						end
					end
					[tmpx, tmpy] = polyxpoly(sensorL(1,a:b), sensorL(2,a:b), lmap(3,:), lmap(4,:));
					for j=1:LENGTH_R
						if (dminr(1) >= hypot(tmpx(j), tmpy(j)))
							dminr(1) = hypot(tmpx(j), tmpy(j));
							dminr(2) = LENGTH_L;
						end
					end
					% Checks both L&R then gets the shortest one
					if (dminl(1) >= dminr(1))
						dmin = dminr(1);
						imax = dminr(2);
						[tmpx, tmpy] = polyxpoly(sensorL(1,a:b), sensorL(2,a:b), lmap(3,:), lmap(4,:));
					else
						dmin = dminl(1);
						imax = dminl(2);
						[tmpx, tmpy] = polyxpoly(sensorL(1,a:b), sensorL(2,a:b), lmap(1,:), lmap(2,:));
					end
				else
					if (LENGTH_L)
						%% If Intersects Multiple L but not R
						[tmpx, tmpy] = polyxpoly(sensorL(1,a:b), sensorL(2,a:b), lmap(1,:), lmap(2,:));
						for j=1:LENGTH_L 
							if (dmin >= hypot(tmpx(j), tmpy(j))) % Gets the closest intersect
								dmin = hypot(tmpx(j), tmpy(j));
								imax = j;
							end
						end
					else
						%% If Intersects Multiple R but not L
						[tmpx, tmpy] = polyxpoly(sensorL(1,a:b), sensorL(2,a:b), lmap(3,:), lmap(4,:));
						for j=1:LENGTH_R 
							if (dmin >= hypot(tmpx(j), tmpy(j))) % Gets the closest intersect
								dmin = hypot(tmpx(j), tmpy(j));
								imax = j;
							end
						end
					end
				end
				IntL(:,i) = [tmpx(imax), tmpy(imax)];
		end
	end
	
	% Update Sensor Data
	for i=1:5
		sensorD(i) = hypot(IntL(1,i),IntL(2,i));
	end
	
	%% Neural Model
	% Simple NN with simple control
	% Current Activation Function: Identity (Do Nothing)
	% Current Output Range: [-1 ; 0] <-> [1 ; 20]
	
	steermodel = (NNW*sensorD' + NNb);
	steering = (steermodel(1))/10;
	speed = (steermodel(2));
	disp(steermodel)
	disp([steering,speed])
	
	%% Car Dynamic
	% Updates CAR dynamics based on NN control
	% Limiting trajectory length creates a moving trajectory
	% Limiting speedplot creates a moving plot
	
	theta = theta + speed*dt/D*steering; % Steering output range [-1:1]
	x = x + speed*dt*cos(theta + pi/2);
	y = y + speed*dt*sin(theta + pi/2);
	trajectory = [trajectory,[x;y]];
	speedplot = [speedplot,[length(trajectory);speed]];
	steerplot = [steerplot,[length(trajectory);steering*10]];
	if (length(trajectory) > 700) % Limits length of trajectory
		trajectory(:,1) = [];
	end
	if (length(speedplot) > 500) % Limits length of speedplot
		speedplot(:,1) = [];
		speedplot(1,:) = (1:500);
	end
	if (length(steerplot) > 500) % Limits length of steerplot
		steerplot(:,1) = [];
		steerplot(1,:) = (1:500);
	end
	
	%% Updates GUI
	
	% Global & Local Map
	set(hlmap(1), 'xdata', lmap(1,:), 'ydata', lmap(2,:));
	set(hlmap(2), 'xdata', lmap(3,:), 'ydata', lmap(4,:));
	set(hcar(1), 'xdata', CarGlobal(1,:)+x, 'ydata', CarGlobal(2,:)+y);
	set(hcar(2), 'xdata', trajectory(1,:), 'ydata', trajectory(2,:));
	% Intersect Marker location
	set(pmg, 'xdata', IntG(1,:), 'ydata', IntG(2,:));
	set(pml, 'xdata', IntL(1,:), 'ydata', IntL(2,:));
	% Sensor Input Data
	set(hsdat(1), 'ydata', sensorD);
	set(pl, 'ydata', [sensorD(1) 0 nan sensorD(2) 0 nan sensorD(3) 0 nan sensorD(4) 0 nan sensorD(5) 0])
	% Sensor Text
	for i=1:5
		set(pt(i), 'string', string(round(sensorD(i),2)), 'position', [i,12]);
	end
	% Neural Output
	set(hnno(1), 'xdata', speedplot(1,:),'ydata', speedplot(2,:));
	set(hnno(2), 'xdata', steerplot(1,:),'ydata', steerplot(2,:));
	drawnow;
end