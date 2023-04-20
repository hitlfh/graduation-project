ObstList = []; % Obstacle point list

%给定障碍物世界坐标(x,y)
for i = -25:25
    ObstList(end+1,:) = [i,30];
end
for i = -6:6
    ObstList(end+1,:) = [i, 0];
end
for i = -25:-6
    ObstList(end+1,:) = [i, 5];
end
for i = 6:25
    ObstList(end+1,:) = [i, 5];
end
for i = 0:5
    ObstList(end+1,:) = [6, i];
end
for i = 0:5
    ObstList(end+1,:) = [-6, i];
end

% 注意，变换地图时除了在这里还要在VehicleCollisionCheck中进行修改
ObstLine = []; % Park lot line for collision check
tLine = [-25, 30 , 25, 30]; %start_x start_y end_x end_y
ObstLine(end+1,:) = tLine;
tLine = [-25, 5, -6, 5];
ObstLine(end+1,:) = tLine;
tLine = [-6, 5, -6, 0];
ObstLine(end+1,:) = tLine;
tLine = [-6, 0, 6, 0];
ObstLine(end+1,:) = tLine;
tLine = [6, 0, 6, 5];
ObstLine(end+1,:) = tLine;
tLine = [6, 5, 25, 5];
ObstLine(end+1,:) = tLine;
tLine = [-25, 5, -25, 30];
ObstLine(end+1,:) = tLine;
tLine = [25, 5, 25, 30];
ObstLine(end+1,:) = tLine;

%定义两个结构体 Vehicle 和 Configure
Vehicle.WB = 3.7;  % [m] wheel base: rear to front steer
Vehicle.W = 2.6; % [m] width of vehicle
Vehicle.LF = 4.5; % [m] distance from rear to vehicle front end of vehicle
Vehicle.LB = 1.0; % [m] distance from rear to vehicle back end of vehicle
Vehicle.MAX_STEER = 0.6; % [rad] maximum steering angle 
Vehicle.MIN_CIRCLE = Vehicle.WB/tan(Vehicle.MAX_STEER); % [m] mininum steering circle radius

% ObstList and ObstLine
Configure.ObstList = ObstList;
Configure.ObstLine = ObstLine;

% Motion resolution define
Configure.MOTION_RESOLUTION = 0.1; % [m] path interporate resolution
Configure.N_STEER = 20.0; % number of steer command
Configure.EXTEND_AREA = 0; % [m] map extend length
Configure.XY_GRID_RESOLUTION = 2.0; % [m]
Configure.YAW_GRID_RESOLUTION = deg2rad(15.0); % [rad]
% Grid bound
Configure.MINX = min(ObstList(:,1))-Configure.EXTEND_AREA;
Configure.MAXX = max(ObstList(:,1))+Configure.EXTEND_AREA;
Configure.MINY = min(ObstList(:,2))-Configure.EXTEND_AREA;
Configure.MAXY = max(ObstList(:,2))+Configure.EXTEND_AREA;
Configure.MINYAW = -pi;
Configure.MAXYAW = pi;
% Cost related define
Configure.SB_COST = 0; % switch back penalty cost
Configure.BACK_COST = 1.5; % backward penalty cost
Configure.STEER_CHANGE_COST = 1.5; % steer angle change penalty cost
Configure.STEER_COST = 1.5; % steer angle penalty cost
Configure.H_COST = 10; % Heuristic cost

%设置初始及终点
Start = [22, 20, pi];
End = [2, 2, pi];

% 使用完整约束有障碍情况下用A*搜索的最短路径最为hybrid A*的启发值
ObstMap = GridAStar(Configure.ObstList,End,Configure.XY_GRID_RESOLUTION);
Configure.ObstMap = ObstMap;
%cla %  从当前坐标区删除包含可见句柄的所有图形对象。

[x,y,th,D,delta] = HybridAStar(Start,End,Vehicle,Configure);

if isempty(x)
    disp("Failed to find path!")
else
    VehicleAnimation(x,y,th,Configure,Vehicle)
end