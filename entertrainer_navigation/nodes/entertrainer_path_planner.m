%% Plan Path Between Two Points

%% 
% Create a Binary Map using the binaryOccupancyMap function

image = imread('map.png'); % Uploads a color image as uint8 integers from a png file that I intend to be the map.
grayimage = rgb2gray(image); % converts the color image to a gray image defined in uint8 integers
bwimage = grayimage < 254; % converts the gray image to a binary logical matrix for a black and white image that can be read by the function
BW2 = imfill( bwimage ,'holes');
% down sized the map so the map relatively matched the dimensions given
% with 65mm map vertical length and 17mm airway entry width. 
map = binaryOccupancyMap(BW2,20);
imshow(BW2)% create state space bounds to be the same as map limits.
map.show
StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];
% Create a state space in the Euclidean Plane.

ss = stateSpaceDubins(StateBounds);
% set the min turning radius to 1.5m based on the snake robot kinematics
ss.MinTurningRadius = 0.8;

%% 
% Create an |occupanyMap|-based state validator using the created state space.

sv = validatorOccupancyMap(ss);


sv.Map = map;
%% 
% Set validation distance for the validator.

sv.ValidationDistance = .5;

%% 
% Create the path planner and increase max connection distance.

planner = plannerRRT(ss,sv);
planner.MaxConnectionDistance = 0.8;
planner.MaxNumTreeNodes = 1e6;
planner.MaxIterations = 1e6;
%% 
% Set the start and goal states.

start = [15,10,0];
goal = [40,45,0];
%% 
% Plan a path with default settings.

for i = 1:1
    rng(i,'twister'); % for repeatable result
    [pthObj,solnInfo] = planner.plan(start,goal);

    path = pthObj.States;
    thetas = pthObj.States(:,3);
    d_thetas = diff(thetas);
    writematrix(path, sprintf('path.csv', i));
end

%% 
% Visualize the results.

map.show; hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
plot(pthObj.States(:,1), pthObj.States(:,2),'r-','LineWidth',2) % draw path
xlabel('x[mm]');
ylabel('y[mm]');
%% 