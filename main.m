% Revision 0.01.0
% Class Work - Completed: 11.24.2017
% Final version from textbook -- based on example "mobile10demo.m"

close all

% G_MAP -> GLOBAL MAP
gmap = VehicleMap'; 
x = (gmap(1,1) + gmap(3,1))/2;
y = (gmap(2,1) + gmap(4,1))/2;
xyxy = [x;y;x;y];
trajectory = [x;y];

theta = atan2(gmap(2,2)-gmap(2,1) , gmap(1,2)-gmap(1,1)) - pi/2;
R = [cos(theta),-sin(theta) ; sin(theta),cos(theta)];
Rm = [R', zeros(2) ; zeros(2), R'];

% L_MAP -> LOCAL MAP
lmap = Rm*(gmap - xyxy(:,ones(1,length(gmap)))); 

dt = 0.1;
speed = 10;
D = 0.5;
roadwidth = 3;
eyeheight = 3;
viewangle = 3;
lookahead = 3;
senseangle = 10;

% N_MAP -> NEAR MAP (POV)
nmap = lmap;
nmap([2,4],:) = nmap([2,4],:).*(nmap([2,4],:)>0) + 0.01;
nmap([1,3],:) = nmap([1,3],:)./nmap([2,4],:);
nmap([2,4],:) = -eyeheight./nmap([2,4],:);

Car = 	[0,0; 1,-0.5; 0,1.5; -1,-0.5; 0,0;...
		 nan,nan; -senseangle,lookahead; senseangle,lookahead]';
newCar = R*Car;

theta = theta + 25*pi/180;

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

while (1)
	R = [cos(theta),-sin(theta) ; sin(theta),cos(theta)];
	newCar = R*Car;
	Rm = [R', zeros(2) ; zeros(2), R'];
	xyxy = [x;y;x;y];
	lmap = Rm*(gmap - xyxy(:,ones(1,length(gmap))));

	set(hlmap2(1), 'xdata', lmap(1,:), 'ydata', lmap(2,:));
	set(hlmap2(2), 'xdata', lmap(3,:), 'ydata', lmap(4,:));
	set(hnewcar1(1), 'xdata', newCar(1,:)+x, 'ydata', newCar(2,:)+y);
	
	nmap = lmap;
	nmap([2,4],:) = nmap([2,4],:).*(nmap([2,4],:)>0) + 0.01;
	nmap([1,3],:) = nmap([1,3],:)./nmap([2,4],:);
	nmap([2,4],:) = -eyeheight./nmap([2,4],:);
	set(hlmap3(1), 'xdata', nmap(1,:), 'ydata', nmap(2,:));
	set(hlmap3(2), 'xdata', nmap(3,:), 'ydata', nmap(4,:));

	viewmap12 = lmap(1:2,:); % BLU LINE
	viewmap34 = lmap(3:4,:); % RED LINE
	linesense = [-senseangle,senseangle ; lookahead,lookahead];

	viewmap12 = viewmap12(:, abs(viewmap12(1,:)) < senseangle );
	viewmap34 = viewmap34(:, abs(viewmap34(1,:)) < senseangle );

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

	if (isempty(linesenseind))
		break;
	end

	steering = -mean(linesenseind)*10;
	theta = theta + speed*dt/D*tan(steering*pi/180);
	x = x + speed*dt*cos(theta + pi/2);
	y = y + speed*dt*sin(theta + pi/2);
	trajectory = [trajectory,[x;y]];
	set(hnewcar1(2), 'xdata', trajectory(1,:), 'ydata', trajectory(2,:));

	drawnow; 
	%pause(0.1);
end