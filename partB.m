
function Main()
clc;clear;

% Load data, to be played back. 
file='DataUsr_p021.mat';   
% load the data, now.
load(file); 
% It will load a variable named data (it is a structure)  
% Use the data.
ExploreData(data);
end
% ----------------------------------------
function ExploreData(data)
AA=API_MTRN4010a(1);      % init API, JUST once

%tic();
plot_envirnment = InitCertainPartOfMyProgram(data); %figure 11 -background 
X_0             = data.pose0;            %platform's initial pose; [x0;y0;heading0] [meters;meters;radians]
n_events        = data.n ;               % how many events?
table           = data.table;            % table of events.
event0          = table(:,1);            % first event.
t0              = event0(1);             % initial time (the time of event0).
t0              = 0.0001*double(t0);     % "event" is of integer type, that is why we convert t0 to double ( i.e. to "real") and over to s
vw              = [0;0];                 % The last [speed,heading rate] measurement.
pos_buffer      = zeros(3,n_events,'single');     % A buffer for recording my results (estimated poses).  size =3 * n_events.


%............................
% LiDARs' parameters (position and oriantation in car's frame, FoV, etc)
Lidar1Cfg    = data.LidarsCfg.Lidar1;  %Info about LiDAR installation (position and orientation,in platform's coordinate frame.). 
% info: 'LiDAR's pose in UGV's CF=[Lx;Ly;Alpha], in [m,m,RADIANS]'
%............................

%initial som variables:
OOIGCF= [];%zeros(20,2); 
lm= [];
OOI= [];
measured_rates = [];
OOIGCF_local_cart=[];
landmarks = data.Context.Landmarks;

X_kin_mod = zeros(2,n_events);

for i = 1: 2000;%n_events % can use for loop but then buttons do not work.

            pos_buffer(:,i) = X_0;  %record current pose; so, we can analyze the estimated trajectory after the loop ends.
            % get event description.
            event     = table(:,i);                %event #i    -->(:,i) gives me columns                   
            sensorID  = event(3);
            index     = event(2);                  % where to read the actual measurement, from that sensor recorder.% source (i.e. which sensor generated this event?)
            
            tNow      = 0.0001*double(event(1));   % when was this measurement taken? Time in tNow is expressed in seconds.
            dt        = tNow-t0;                   % dt since last event ( "dt" is needed for prediction steps).
            t0        = tNow ;                     % remember current time, so we can calculate dt in next iteration.            
            
            
            X_0             = MyKinematicModel(X_0,vw,dt); 
            X_kin_mod(1,i)  = X_0(1);
            X_kin_mod(2,i)  = X_0(2);
            theta           = X_0(3);
            measured_rates = [measured_rates; theta];
            %the UGV current pos, part_C:
                % length of the arrow
                L = 2;
                % Calculate the x and y components of the arrow
                u = L*cos(theta);
                v = L*sin(theta);
                %plot the ugv dynamicly with a arrow sticking out:
             set(plot_envirnment,'xdata',X_0(1),'ydata',X_0(2),'UData',u,'VData',v);

             % measurement is from which sensor?
             switch sensorID   
                
                case 1         %  it is a scan from  LiDAR#1, and scan from LiDAR#2! (both, as they are synchronized)
                %fprintf('Event (LiDAR), dt=[%.1fms], at t=[%.3f]\n',dt*1000,tNow);   % print some info, for debugging our code.
                
                % both LiDARs are sinchronized.
                scan1 = data.scans(:,index);  %get the col of scan1, each col is a scan (301 scans).
                
                %[Xe,P] = processLiDAR(Xe,P, scan1,Lidar1Cfg,data.Context.Landmarks,AA);  

    
            
                mask1 = 16383;   %  0x7FFF % extract lowest 14 bits (actual ranges, in cm).
                ranges = bitand(scan1, mask1);        % in cms, uint16
                ranges = single(ranges)*0.01; 
                [ii12,props,numOOIs] = AA.FindSmallSegmentsFS(ranges,0.3);  %props = OOI-center
      
              if (numOOIs>0)
                OOIGCF_local_cart = OOIGCF_local_cartesian(props,X_0,Lidar1Cfg);
                OOIGCF = OOIs_from_local_to_global(props,X_0,Lidar1Cfg)  ;      
              
                OOIGCF=OOIGCF';

                tolerance = 0.8 ; % in the same units you use for OOIs and Landmarks position, e.g., in meters.
                [na,aa,iiO,uuL] = AA.MyDA( OOIGCF(1,:),OOIGCF(2,:),landmarks(1,:), landmarks(2,:), tolerance);
              

            %if na > 3
            %disp('lanmarks');
            lm =  landmarks(:,uuL) ;
            %disp('OOI');
            OOI = OOIGCF( :,iiO ) ;
            
            
%                 for j = 1 : na
%                     X_0;
%                      lm = [lm ; landmarks(:,uuL(j)) ];
%                      OOI = [OOI ; OOIGCF( :,iiO(j)) ];
%                     
%                 end
              %break;
                
            
            %end

              end




               pause(0.05);
               continue;                       %"done, next event!"
                
                %...............................
                case 2                          %  It is speed encoder + gyro  (they are packed together, synchonized readings)
                
                vw   = data.vw(:,index);        % speed and gyroZ, last updated copy. keep the variable "vw" updated to the last reading of those sensors.
 
                continue;
             end 
end

% for i = 1: n_events % can use for loop but then buttons do not work.
% 
%             pos_buffer(:,i) = X_0;  %record current pose; so, we can analyze the estimated trajectory after the loop ends.
%             % get event description.
%             event     = table(:,i);                %event #i    -->(:,i) gives me columns                   
%             sensorID  = event(3);
%             index     = event(2);                  % where to read the actual measurement, from that sensor recorder.% source (i.e. which sensor generated this event?)
%             
%             tNow      = 0.0001*double(event(1));   % when was this measurement taken? Time in tNow is expressed in seconds.
%             dt        = tNow-t0;                   % dt since last event ( "dt" is needed for prediction steps).
%             t0        = tNow ;                     % remember current time, so we can calculate dt in next iteration.            
%             
%             
%             %part A
%             % apply Kinematic model here, using the last values of inputs ( in vw), and the last value of the system's state, X_0
% 
%             X_0             = MyKinematicModel(X_0,vw,dt); 
%             X_kin_mod(1,i)  = X_0(1);
%             X_kin_mod(2,i)  = X_0(2);
%             theta           = X_0(3);
%       
%             %the UGV current pos, part_C:
%                 % length of the arrow
%                 L = 2;
%                 % Calculate the x and y components of the arrow
%                 u = L*cos(theta);
%                 v = L*sin(theta);
%                 %plot the ugv dynamicly with a arrow sticking out:
%              set(plot_envirnment(9),'xdata',X_0(1),'ydata',X_0(2),'UData',u,'VData',v);
% 
%          
%                 
% 
% end

%PoseGuess0=[double(X_0(1))+1;double(X_0(2));double(X_0(3))] % using the data.pose0 as my guess. 
PoseGuess0 = [3 ; 4 ; 0.2];
%OOIs_GCF2 = TransformCF01(data.pose0,0.45,local_OOIs'); 
lm';
OOI';


%OOIs_center = detected_OOIs_center(global_pos_OOI1);
%disp((OOIs_center));

    
    
OOIGCF_local_cart=OOIGCF_local_cart';
   
 


PoseGuess0 = [3 ; 4 ; 0.2]
PoseS = EstimateLidarPose(PoseGuess0,OOIGCF_local_cart,lm);
disp('solution (estimated Pose)'); disp(PoseS');



%ShowVerification1(data,X_kin_mod,global_pos_OOI1,OOIs_center,lm, OOI );




end  


% -----------------------------------------------------------------------------------------------------------

function c = CostA(poseV,OOIs_LCF, Landmarks)
% PoseV:  proposed vehicle's pose.
% disp (lm);

Lx=0.45;
OOIs_GCF = TransformCF01(poseV,Lx,OOIs_LCF);  

d= Landmarks -OOIs_GCF
d = (d.*d); 
d=sqrt(d(1,:)+d(2,:));    
c=sum(d)    

end
%-----------------------------------------------------------------------------------------------------------
function pp =  TransformCF01(X,Lx,pp)  
h=X(3);
c=cos(h); s=sin(h);
R= [ [ c,-s];[s,c]];
pp=R*pp;
T = [Lx;0];
T = X(1:2)+R*T;
pp(1,:)=pp(1,:)+T(1);
pp(2,:)=pp(2,:)+T(2);
end
%-----------------------------------------------------------------------------------------------------------
% -----------------------------------------------------------------------------------------------------------
function PoseS = EstimateLidarPose(PoseGuess0,OOI,lm)

Op=optimset('TolFun',0.01);
tic();
[PoseS,fval,exitflag] = fminsearch(@(X) CostA(X,OOI,lm), PoseGuess0,Op)  %Solution

dt=toc()*1000


end
% -----------------------------------------------------------------------------------------------------------
function plot_envirnment = InitCertainPartOfMyProgram(data)

    % create some figure for your stuff.
    figure(4); clf();    % global CF.
    
    Landmarks=data.Context.Landmarks;
    % plot centres of landmarks. 
    plot(Landmarks(1,:),Landmarks(2,:),'ko','markersize',3)

    % plot interior of walls (they have ~20cm thickness; but the provided info just includes the ideal center of the walls
    % your LiDAR scans will appear at ~ 10cm from some of those lines.    
    % Wall transversal section:  :  wall left side [<--10cm-->|<--10cm-->] the other  wall side. 
    hold on;
    Walls = data.Context.Walls;
    plot(Walls(1,:),Walls(2,:),'color',[0,1,0]*0.7,'linewidth',3);
    legend({'Centers of landmarks','Walls (middle planes) '});
    
    title('Global CF (you should show some results here)');
    xlabel('X (m)'); 
    ylabel('Y (m)');
    p0=data.pose0;
    plot(p0(1),p0(2),'r*','markersize',10);
    
    
    plot_envirnment = CreateFigureToShowScansInPolar();
end
% ---------------------------------------------------------------------------------
function ShowVerification1(data,X_kin_mod,global_pos1,OOIs_center,matchedPoints1, matchedPoints2 )
figure(4)
hold on;
p=data.verify.poseL;
plot(p(1,:),p(2,:),'r.');
%legend({'Landmarks','Walls (middle planes)','Initial pose','Ground truth (subsampled)'});
hold on;
%plotT he position component of the predicted poses,assuming bias=0,ingreen color.
plot(X_kin_mod(1,:),X_kin_mod(2,:), 'go'); 
%hold on;
%plot (global_pos1(:,1),global_pos1(:,2),'b+');

%hold on;


end
% ---------------------------------------------------------------------------------

function [ooi_positions] = Detect_OOIs(scan) %OOIs in cartesian LiDaR-CF
    % Constants
    mask2 = 49152; % 1100000000000000
    intensities = bitand(scan, mask2);
    max_diameter = 0.20; % m
    max_distance = 0.20; %m
    
    % Get the ranges in cartesian LiDaR_CF:
    [x, y] = Ranges_in_cartesian_LiDaR_CF(scan); % func working with ranges and angles
    points = [x, y];
    
    % Initialize distance vector
    distances = zeros(size(points, 1)-1, 1); % 300x1 vector dist. between a point and all the others, in cartesian
    
    % Compute distances between consecutive points
    for i = 1:size(points, 1)-1 
        distances(i) = norm(points(i+1,:) - points(i,:));
    end
    
   
    index_array = [];
    prev_index = 0; % Initialize previous index
    
    % Loop through distances and find the indexes for where we have cut
    for i = 1:length(distances)
        if distances(i) > max_distance
            % Add previous indexes and current index to index_array
            index_array = [index_array, (prev_index+1):i];
        end
        % Update previous index
        prev_index = i;
    end
    
    % NOW find the diameter for each sequence of dots and find OOIs given criteria:
    prev = 1;
    ooi_x = [];
    ooi_y = [];
    for i = 1:length(index_array)
        j = index_array(i);
        diameter = norm(points(j, :) - points(prev+1, :)); % diameter
        if diameter < max_diameter
            for k = prev:j
                if intensities(k) > 0                 %check if color>0
                    ooi_x = [ooi_x; mean(x(prev:j))];
                    ooi_y = [ooi_y; mean(y(prev:j))];
                    break; % break out of the loop when a valid intensity is found
                end
            end
        end
        prev = j+1;
    end
    ooi_positions = [ooi_x, ooi_y];
end
    
% ---------------------------------------------------------------------------------
function [x,y] = Ranges_in_cartesian_LiDaR_CF(scan)   %used in two fuctions above, returns pos of dots in cartesan local CF
    mask1  = 16383;  
    ranges = single(bitand(scan,mask1))*0.01; 
    angles = [-75:0.5:75]';
    angles = deg2rad(angles);
    % Convert to Cartesian coordinates using the LiDAR's geometry
    x = ranges .* cos(angles);
    y = ranges .* sin(angles);
end
% ---------------------------------------------------------------------------------
function [OOI_global_pos] = local_to_globalk(scan, X_0,X_kin_mod,LidarCfg) 
    %the heading of the vehicle:
    theta = X_0(3);
    % lidar position in GCF (assuming scalar):
    x_origin = X_kin_mod(1); 
    y_origin = X_kin_mod(2); 
    %Lidar pos in UGV:
    lx = LidarCfg.Lx;
    ly = LidarCfg.Ly;
    alpha = LidarCfg.Alpha;
    %the rotation matrix LiDar to UVG:
    R_LiDaR= [cos(alpha) -sin(alpha);
              sin(alpha) cos(alpha)];
    %the rotation matrix UVG to Global:
    R = [cos(theta) -sin(theta);
        sin(theta) cos(theta)];
    % now get the OOI in local LiDaR_CF using detect_ooi function
    [ooi_positions] = Detect_OOIs(scan);  %return OOI_pos in cartesian local LiDaR_CF 
    % loop over detected OOIs and find the these Points in UVG_CF
    OOI_p_local = zeros(size(ooi_positions, 1),2);
    for i = 1:size(ooi_positions, 1)
        OOI_p_local(i,:) = R_LiDaR * [ooi_positions(i, 1),ooi_positions(i, 2)]' +[lx;ly]; % x_v pos of ooi in vehicle CF
    end
     % Find the global position of the OOI
     OOI_global_pos = R * OOI_p_local' + [x_origin;y_origin ];
     OOI_global_pos = OOI_global_pos'; %//was wrong dim. global pos give n*1 while shoud be n*2
end
% ---------------------------------------------------------------------------------
function [scans_global_pos] = scan_local_to_global(scan,id, T,X_kin_mod,LidarCfg) %Retrun the all dots in Global_CF
    %the heading of the vehicle:
    theta = T(3);
    % lidar position in GCF (assuming scalar):
    x_origin = X_kin_mod(1); 
    y_origin = X_kin_mod(2); 
    %Lidar pos in UGV:
    lx = LidarCfg.Lx;
    ly = LidarCfg.Ly;
%-----------------------------
    % alpha =  LidarCfg.Alpha for lidar1 and + some corrector for lidar2:
    if id == 1
        alpha = LidarCfg.Alpha;
    end
    if id == 2
        tun = corrct_angle(); % -6;    % =5 for p020  and -6 for p2021
        alpha = LidarCfg.Alpha + deg2rad(tun);
    end
%-----------------------------

    %the rotation matrix:
    R = [cos(theta) -sin(theta);
        sin(theta) cos(theta)];
    
    %alpha = 3.1416;
    R_LiDaR= [cos(alpha) -sin(alpha);
              sin(alpha) cos(alpha)];
    
    % now get the scans (dots) in local CF using detect_ooi function
    [x,y] = Ranges_in_cartesian_LiDaR_CF(scan);  %return scan_pos in cartesian local LiDaR_CF (X_l,y_l)
    p_local = zeros(301,2);
    for i = 1: size(x)
        p_local(i,:) = R_LiDaR * [x(i),y(i)]'+[lx;ly]; 
    end
   
    scans_global_pos = R * p_local' + [x_origin;y_origin ];
    scans_global_pos = scans_global_pos'; %//was wrong dim. global pos give n*1 while shoud be n*2
end
% ---------------------------------------------------------------------------------
%Part D
function [OOIs_center] = detected_OOIs_center(OOI_global_pos) %in GCF
    
     %----First I need to rearrange the OOI_global_pos such that the closest points are in sequence:
    % Compute pairwise distances between points
    distances = pdist2(OOI_global_pos, OOI_global_pos);
    % Sort distances and get corresponding indices
    [sortedDistances, sortedIndices] = sortrows(distances);
    % Rearrange points according to sorted indices
    OOI_global_pos_arranged = OOI_global_pos(sortedIndices, :);

    %---find the oois that represent the same landmark by taking the distance
    %between them.
    distances1 = zeros(size(OOI_global_pos_arranged, 1)-1, 1); 
    for i = 1:size(OOI_global_pos_arranged, 1)-1 
        distances1(i) = norm(OOI_global_pos_arranged(i+1,:) - OOI_global_pos_arranged(i,:));
    end
 
    index_array = [];
    prev_index = 0; % Initialize previous index
    %--- Loop through distances and find the indexes for where we have cut
    for i = 1:length(distances1)
        if distances1(i) > 1.5
            % Add previous indexes and current index to index_array
            index_array = [index_array, (prev_index+1):i];
        end
        % Update previous index
        prev_index = i;
    end
    prev = 1;
    OOIs_center_x=[];
    OOIs_center_y=[];
    %--Update the OOIs_center coordinates:
    for i = 1:length(index_array)
        j = index_array(i);
        OOIs_center_x = [OOIs_center_x; mean(OOI_global_pos_arranged(prev:j,1))];
        OOIs_center_y = [OOIs_center_y; mean(OOI_global_pos_arranged(prev:j,2))];
        prev = j + 1;
    end
   OOIs_center = [OOIs_center_x,OOIs_center_y];
end  


% ---------------------------------------------------------------------------------
function [matchedPoints1, matchedPoints2] = Data_assosiation(Landmarks,OOIs_center)
%Here I will run through all OOIs_center and data.Context.Landmarks to
%assosiate landmarkes with their corresponding detected OOI.
% Compute pairwise distances between points from both sets
%disp(size(OOIs_center));
Landmarks = Landmarks';
distances = pdist2(Landmarks, OOIs_center);
% Set a threshold distance for matching points (between 0.5m and 1.5m)
threshold = 1;
% Find pairs of points whose distances are below the threshold
[indices1, indices2] = find(distances <= threshold);
% Extract the matched points from each set
matchedPoints1 = Landmarks(indices1, :);
matchedPoints2 = OOIs_center(indices2, :);


end


% ---------------------------------------------------------------------------------

function figur_handles = CreateFigureToShowScansInPolar()
   
    figure(4); 
    hold on;
    movegui(gcf, [80 230]);
    hold on;
    h = quiver(0,0,'O','LineWidth', 2,'MaxHeadSize', 0.5,'MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','k');
    %h5 = quiver(0,0,'s','LineWidth', 1,'MarkerFaceColor','r','MarkerEdgeColor','k');
    title('OOIs globl CF');  xlabel('X (m)');  ylabel('Y (m)'); axis([-5,21,-5,26]); grid on;
   

   
    figur_handles = h;
end    
% ---------------------------------------------------------------------------------


function X = MyKinematicModel(X,vw,dt)
bias = 0;
       vw_tun = vw + bias;  %take the gyroscope values and correct it, been used in X(3)

     % Here, you implement your discrete time model; e.g. using Euler's approach.
       X(1) = X(1) + vw(1) * cos(X(3)) * dt; % x=x0+v*dt
       X(2) = X(2) + vw(1) * sin(X(3)) * dt;
       X(3) = X(3) + vw_tun(2) * dt;
end  
% ---------------------------------------------------------------------------------


function OOIGCF = OOIs_from_local_to_global (props,Xe,Lidar1Cfg)
    % * props: (OOIs' properties)
    %   props (:,3) = angle values of OOI-center 
    %   props (:,2) = ranges values of OOI-center
    
    angles = props (:,3);
    angles = deg2rad(angles);
    ranges =  props (:,2);
    % Convert to Cartesian coordinates using the LiDAR's geometry
    OIIxs = ranges .* cos(angles);  % OOI's x in sensor CF
    OIIys = ranges .* sin(angles); % OOI's y in sensor CF
    
    % Now Convert to GCF

    theta = Xe(3);

    % lidar position in GCF (assuming scalar):
    x_origin = Xe(1); 
    y_origin = Xe(2); 
    %Lidar pos in UGV:
    lx = Lidar1Cfg.Lx;
    ly = Lidar1Cfg.Ly;
    alpha = Lidar1Cfg.Alpha;
    %the rotation matrix LiDar to UVG:
    R_LiDaR= [cos(alpha) -sin(alpha);
              sin(alpha) cos(alpha)];
    %the rotation matrix UVG to Global:
    R = [cos(theta) -sin(theta);
        sin(theta) cos(theta)];

    % now get the OOI in local LiDaR_CF using detect_ooi function
   ooi_positions = [OIIxs, OIIys];

    % loop over detected OOIs and find the these Points in UVG_CF
    OOI_p_local = zeros(size(ooi_positions, 1),2);
    for i = 1:size(ooi_positions, 1)
        OOI_p_local(i,:) = R_LiDaR * [ooi_positions(i, 1),ooi_positions(i, 2)]' +[lx;ly]; % x_v pos of ooi in vehicle CF
    end
     % Find the global position of the OOI
     OOI_global_pos = R * OOI_p_local' + [x_origin;y_origin ];
     OOI_global_pos = OOI_global_pos'; %//was wrong dim. global pos give n*1 while shoud be n*2
    
    OOIGCF = OOI_global_pos;
    OOIGCF_local = [OIIxs;OIIys] ;

end


function OOIGCF_local_cart = OOIGCF_local_cartesian(props,Xe,Lidar1Cfg)
    
    angles = props (:,3);
    angles = deg2rad(angles);
    ranges =  props (:,2);
    % Convert to Cartesian coordinates using the LiDAR's geometry
    OIIxs = ranges .* cos(angles);  % OOI's x in sensor CF
    OIIys = ranges .* sin(angles); % OOI's y in sensor 
    OOIGCF_local_cart = [OIIxs,OIIys];
end

