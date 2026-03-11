%% "Laser scan" line fit example
% Clear workspace, command window, and close all figures
clear all
clc
close all

% load example data
exampleHelperROSLoadMessages
scan
figure()
plot(scan)

% get data from scan
cart = readCartesian(scan);
figure()
plot(cart(:,2), cart(:,1), '.') % note - y before x..

x = cart(:,2); % x-pos
d = cart(:,1); % depth
%% Fitting one line..

% only left side
xleft = x(100:200)
dleft = d(100:200)
plot(xleft, dleft, '.')

mdl = fitlm(xleft,dleft);
coef=mdl.Coefficients.Estimate
figure()
plot(x,d, '.'), hold on
plot(x, coef(1) + coef(2)*x, 'r')

%% Multi-line method..
% Transform scan to image
% where pixels where there is edge are 1
% otherwise 0
% Provided as input to the Hough Transform
N = 100 % grid size

xoffset = min(x)
xscale = (max(x)-min(x))
xi = (x - xoffset)/xscale; % scale to 0-1
xi = round(xi*(N-1))+1; % scale and integer 1 to N
di = (d - min(d))/(max(d)-min(d)); % scale to 0-1
di = round(di*(N-1))+1;
ind = sub2ind([N N], xi, di) % matrix indices

mat = zeros(N); % NxN matrix
mat(ind) = 1; % with points..
figure()
imshow(mat), colorbar()

%% Hough transform
[H,theta,rho] = hough(mat);

% Hough peaks
Nlines = 5; % Note - Nlines peaks/lines !
P = houghpeaks(H,Nlines); 
figure()
imshow(H,[],'XData',theta,'YData',rho,'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;
plot(theta(P(:,2)),rho(P(:,1)),'s','color','white');

%% Hough lines
lines = houghlines(mat,theta,rho,P,'FillGap',5,'MinLength',7);
figure()
imshow(mat), hold on,
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
end

lines(3)
