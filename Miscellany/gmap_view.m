% Script to plot MAP matrix files
% Originally completed: 12.18.2017
%
% MAP in the form:
% [Ax1 ... xn ; Ay1 ... yn ; Bx1 ... Bxn ; By1 ... Byn ];
% Where each coordinate pair (Axn, Ayn) or (Bxn, Byn) makes up one vertex.
% The first two rows make up one side of the track,
% and the last two row makes up the other side of the track.
%
% Also includes codes for testing polyxpoly() intersection function.

clear

load('gmap.mat');
figure(1);
plot([0 0 100 100], [0 100 100 0]);
hold on
plot(gmap(1,:),gmap(2,:),'r',...
	gmap(3,:), gmap(4,:),'b');

% Generates green diagonal line across the map
xline = [10 70];
yline = xline;
plot(xline, yline, 'g');

% Intersects it with the RED track
% Returns value as matrix
[xint, yint] = polyxpoly(xline, yline, gmap(1,:), gmap(2,:));
mapshow(xint, yint, 'DisplayType', 'Point', 'Marker', 'o');

% Generates another line
APOINT = [10 20 ; 20 30 ; 30 40 ; 40 30 ; 50 20];

mapshow(APOINT(:,1), APOINT(:,2), 'DisplayType', 'Point', 'Marker', 'o');

Test = [80 80 nan 90 75;
		   10 30 nan 10 30];
plot(Test(1,1:2), Test(2,1:2));
plot(Test(1,4:5), Test(2,4:5));

[rx, ry] = polyxpoly(Test(1,:), Test(2,:), gmap(1,:), gmap(2,:))
[bx, by] = polyxpoly(Test(1,:), Test(2,:), gmap(3,:), gmap(4,:))

mapshow(rx, ry, 'DisplayType', 'Point', 'Marker', 'o');
mapshow(bx, by, 'DisplayType', 'Point', 'Marker', 'o');