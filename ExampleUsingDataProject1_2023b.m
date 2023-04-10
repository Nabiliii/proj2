


% ---------------------------------------------------------------------------------
function Main()

% Load data, to be played back. This is just one of the may datasets you may use.
file='DataUsr_007k.mat';   %one of the datasets we can use (simulated data, noise free, for Project 1).
% As it is "noise-free", you can debug your implementation more easily.

% load the data, now.
load(file); 

% It will load a variable named data (it is a structure)  

% Use the data.
ExploreData(data);
end

% ----------------------------------------

function ExploreData(data)

%new- proj2
sv = 0.01;          %Speed measurements: standard deviation: 10cm/second.
sw = 0.069813;             %Gyroscope measurements: standard deviation: 4 deg/s = 0.069813 rad/s

Xe = data.pose0;  
P  = zeros(3,3);

Pu  = [ [ sv^2 ,    0  ];
        [ 0    ,  sw^2 ]
      ];            


hh=InitCertainPartOfMyProgram(data);
ne     = data.n;                % how many events?
EventsList  = data.table;            % table of events.
event0 = EventsList(:,1);            % first event.
t0=event0(1) ; t0=0.0001*double(t0); % initial time (the time of event0).
vw=[0;0];  % Program variable to keep last [speed (aka "longitudinal velocify"),heading rate] measurement.
XX=zeros(3,ne,'single');     % A buffer for recording my results (estimated poses).  size=3 x ne.
Lidar1Cfg=data.LidarsCfg.Lidar1;  %Info about LiDAR installation (position and orientation, ..


    for i=1:ne,
        
        XX(:,i)=Xe;  %record current pose; so, we can analyze the estimated trajectory after the loop ends.
        
        event   = EventsList(:,i);        %event #i                    
        sensorID=event(3);                          % source (i.e. which sensor generated this event?)
        tNow=0.0001*double(event(1));               % when was this measurement taken?
        % Time in tNow is expressed in seconds.
        dt=tNow-t0;    % dt since last event ( "dt" is needed for prediction steps).
        t0=tNow ;      % remember current time, so we can calculate dt in next iteration.            
        
      
        %X=MyKinematicModel(X,vw,dt);
        %vw is the input at time k
        [Xe,P] = MyPredictionStepEKF(Xe,P,vw,dt,Pu); 



    
        
        index = event(2);  % where to read the actual measurement, from that sensor recorder.
            
        switch sensorID    % measurement is from which sensor?
            
            case 1  %  it is a scan from  LiDAR#1, and scan from LiDAR#2! (both, as they are synchronized)
            fprintf('Event (LiDAR), dt=[%.1fms], at t=[%.3f]\n',dt*1000,tNow);   % print some info, for debugging our code.
            
            % both LiDARs are synchronized.
            scan1 = data.scans(:,index);  
            
            % do something with the data.
            
             [Xe,P] = processLiDAR(Xe,P, scan1,Lidar1Cfg);  
 
            
            pause(0.05);
            continue;  %"done, next event!"
            
            %...............................
            case 2  %  It is speed encoder + gyro  (they are packed together, synchonized readings)
            vw=data.vw(:,index);    % speed and gyroZ, last updated copy.
    
            continue;  %"next!" (go to process next even, inmmediately)
            
            otherwise  % It may happen, if the dataset contains measurements from sensors 
    
            continue;
        end;
    end;       % end loop, reading chronologically sensors' events.
% .....................................................

disp('Loop of events ends.');

disp('Showing ground truth (your estimated trajectory should be close).)');
ShowVerification1(data);  

end
% --------------------------------------------------------------------------------

function hh=InitCertainPartOfMyProgram(data)

figure(11); clf();    % global CF.


Landmarks=data.Context.Landmarks;
% plot centres of landmarks. 
plot(Landmarks(1,:),Landmarks(2,:),'ko')

hold on;
Walls = data.Context.Walls;
plot(Walls(1,:),Walls(2,:),'color',[0,1,0]*0.7,'linewidth',3);
legend({'Centers of landmarks','Walls (middle planes) '});

title('Global CF (you should show some results here)');
xlabel('X (m)'); 
ylabel('Y (m)');
p0=data.pose0;
plot(p0(1),p0(2),'r*','markersize',10);
legend({'Landmarks','Walls (middle planes)','initial position'});


hh = CreateFigureToShowScansInPolar();


end

function ShowVerification1(data)


figure(11)
hold on;
p=data.verify.poseL;
plot(p(1,:),p(2,:),'r.');
legend({'Landmarks','Walls (middle planes)','Initial pose','Ground truth (subsampled)'});
end


% ---------------------------------------------------------------------------------

function [Xe,P] = processLiDAR(Xe,P, scan1,Lidar1Cfg,landmarks);  

   
    mask1 = 16383;   %  0xFFF7 % extract lowest 14 bits (actual ranges, in cm).
    mask2 = 49152;   %  0xC000 % extract highest 2 bits ("intensity")
    ranges = bitand(scan1, mask1);        % in cms, uint16
    ranges = single(ranges)*0.01 ;       % now in meters, single precision.
    colors = bitand(scan1,mask2);
    
    
    [nOOIs, list_of_oois] = Detect_OOIs(scan1);
    if (nOOIs<1), return; end;
    
    
    [nDA, ii_ooi, ii_lm] = DA(list_of_oois,Xe,Lidar1Cfg,landmarks); %tell me waht ooi I tell you what landmark, here is indexes returned.
    %nDA = number of DA succesfull
    if (ndA<1), return; end;
    
    for i = 1 : nDA
        [Xe,P ] = Do_update_OOI (Xe,P, list_of_oois(:,ii_ooi(i)), landmarks(:,ii_lm(i)),Lidar1Cfg,ranges );
    
    end


end

% ---------------------------------------------------------------------------------

function h = CreateFigureToShowScansInPolar()
    
    %I create some figures in which to show some animations (e.g. LiDAR
    %scans in native polar representation)

    figure(10); clf();
    aa = [-75: 0.5 : 75];  % LiDAR's FoV  ( -75 to +75 degrees), discretized by angular resolution (0.5 deg).
    r=aa*0;   % same number of ranges  ( i.e. 301 )
    
    % create figures, and graphic objects for showing, later, data dynamically.
    % I decided to use subfigures ("subplot") (just cosmetic details, you
    % decide your style.)
    subplot(211);  h1 = plot(aa,r,'.b');
    title('LiDAR1(shown in Polar)');  xlabel('angle (degrees)');  ylabel('range (m)'); axis([-75,75,0,20]); grid on;
    hold on;  h1b = plot(0,0,'r+');
    legend({'opaque pixels','brilliant pixels'});
        
    
    subplot(212);  
    h2 = plot(aa,r,'.b'); 
    title('LiDAR2(shown in Polar)');  xlabel('angle (degrees)');  ylabel('range (m)'); axis([-75,75,0,20]); grid on;
    hold on;  h2b = plot(0,0,'r+');
    h = [h1,h1b,h2,h2b];
end    
    
%--------------------------------------------------------------------------


function [Xe,P] = MyPredictionStepEKF(Xe,P,vw,dt,Pu)

    J = JacobianX(Xe,vw,dt);
    Ju = JacobianU(Xe,vw,dt);
    Qu = Ju*Pu*Ju';
    P = J*P*J' + Qu;
    Xe = MyKinematicModel(Xe,vw,dt);



end

%--------------------------------------------------------------------------
function X=MyKinematicModel(X,vw,dt)
       %tun = 0;
       %vw_tun = vw + tun;  %take the gyroscope values and correct it, been used in X(3)
     % Here, you implement your discrete time model; e.g. using Euler's approach.
       X(1) = X(1) + vw(1) * cos(X(3)) * dt; % x=x0+v*dt
       X(2) = X(2) + vw(1) * sin(X(3)) * dt;
       X(3) = X(3) + vw(2) * dt;

       %X(3) = X(3) + vw_tun(2) * dt;
end   

%--------------------------------------------------------------------------
function J = JacobianX(Xe,vw,dt)

    J = [ [1 , 0 , -vw(1)*sin(Xe(3))*dt];
          [0 , 1 ,  vw(1)*cos(Xe(3))*dt];
          [0 , 0 ,         1           ]
        ];
end

%--------------------------------------------------------------------------
function Ju = JacobianU(Xe,vw,dt)
    Ju = [ [ cos(Xe(3))*dt  ,  0   ];
           [ sin(Xe(3))*dt  ,  0   ];
           [      0         ,  dt  ]
         ]; 
end

%--------------------------------------------------------------------------
function  [Xe,P ] = Do_update_OOI (Xe,P, OOI, landmark,Lidar1Cfg, ranges )
    H =   find_H (Xe,P, OOI, landmark,Lidar1Cfg );

    %R = 0.25^2; 
    R = [ [0.25^2 ,  0  ]
          [   0   , 3^2 ]];  %remember 3 is in deg here, check
                 
    ym =   ranges;      %measurement of output variable at time k  ->not sure here
    Z = ym - H*Xe;
    S = H * P*H' + R;
    Si = inv(S);
    K = P*H'*Si;
    
    Xe = Xe+K*Z;
    P = P-P*H'*Si*H*P;


end

%--------------------------------------------------------------------------


function H  = find_H (Xe,P, OOI, landmark,Lidar1Cfg )
x_car = Xe(1);
y_car = Xe(2);
phi = Xe(3);

x = x_car + Lidar1Cfg.Lx*cos(phi);  
y = y_car + Lidar1Cfg.Lx*sin(phi);

%so find the pos to the landmark;
xk = landmark(1);    %should be the pos of centerOOI , fix it
yk = landmark(2);

% h = [  sqrt((xk-x)^2 + (yk-y)^2 );   %h1
%         atan2(yk-y , xk-x) -phi   ];  %h2

%y_m = h;

H =[ [ (xk - x)/((x - xk)^2 + (y - yk)^2)^(1/2)   ,  (yk - y)/((x - xk)^2 + (y - yk)^2)^(1/2)   ,    0];
     [ (yk - y)/((x - xk)^2 + (y - yk)^2)         ,  (x - xk)/((x - xk)^2 + (y - yk)^2)         ,   -1]
   ];


end



%--------------------------------------------------------------------------

function [nOOIs,ooi_positions] = Detect_OOIs(scan) %OOIs in cartesian LiDaR-CF
    % Constants
    mask2 = 49152; % 1100000000000000
    intensities = bitand(scan, mask2);
    max_diameter = 0.20; % m
    max_distance = 0.20; %m
    nOOIs = 0;
    
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
                    nOOIs = nOOIs +1;
                    break; % break out of the loop when a valid intensity is found
                end
            end
        end
        prev = j+1;
    end
    ooi_positions = [ooi_x, ooi_y];
end
    
%--------------------------------------------------------------------------
function [x,y] = Ranges_in_cartesian_LiDaR_CF(scan1)   %used in two fuctions above, returns pos of dots in cartesan local CF
    mask1  = 16383;  
    ranges = single(bitand(scan1,mask1))*0.01; 
    angles = [-75:0.5:75]';
    angles = deg2rad(angles);
    % Convert to Cartesian coordinates using the LiDAR's geometry
    x = ranges .* cos(angles);
    y = ranges .* sin(angles);
end

%--------------------------------------------------------------------------
[nDA, ii_ooi, ii_lm] = DA(list_of_oois,Xe,Lidar1Cfg,landmarks); %tell me waht ooi I tell you what landmark, here is indexes returned.
    %nDA = number of DA succesfull

function [matchedPoints1, matchedPoints2] = DA(Landmarks,OOIs_center)
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