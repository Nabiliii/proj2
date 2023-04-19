
% Project 2 part A1 by Abdulrahman Nabil Akkawi z5444959
% ---------------------------------------------------------------------------------
function Main()

file='DataUsr_007k.mat';  
load(file); 

ExploreData(data);
end

% ----------------------------------------

function ExploreData(data)
    AA=API_MTRN4010a(1);      % init API, JUST once
    BB=API_4010_verifyEKF(data); % Consistency plots
    
    plot_envirnment = InitCertainPartOfMyProgram(data); %figure 11 -background 
    
    %new- proj2
    sv = 0.1;          %Speed measurements: standard deviation: 10cm/second.
    sw = deg2rad(4) ;%0.069813;             %Gyroscope measurements: standard deviation: 4 deg/s = 0.069813 rad/s
    
    Xe = data.pose0;  
    P  = zeros(3,3);       %covariance matrix of the state
    
    Pu  = [ [ sv^2 ,    0  ];   %covariance matrix of input
            [ 0    ,  sw^2 ] ];            
    
    
    ne     = data.n;                % how many events?
    EventsList  = data.table;            % table of events.
    event0 = EventsList(:,1);            % first event.
    t0=event0(1) ; t0=0.0001*double(t0); % initial time (the time of event0).
    vw=[0;0];  % Program variable to keep last [speed (aka "longitudinal velocify"),heading rate] measurement.
    XX=zeros(3,ne,'single');     % A buffer for recording my results (estimated poses).  size=3 x ne.
    Lidar1Cfg=data.LidarsCfg.Lidar1;  %Info about LiDAR installation (position and orientation, ..
    
    X_truck = zeros(2,ne);
    
    for i=1:ne
        
        event   = EventsList(:,i);        %event #i                    
        sensorID=event(3);                          % source (i.e. which sensor generated this event?)
        tNow=0.0001*double(event(1));               % when was this measurement taken?
        % Time in tNow is expressed in seconds.
        dt=tNow-t0;    % dt since last event ( "dt" is needed for prediction steps).
        t0=tNow ;      % remember current time, so we can calculate dt in next iteration.            
        
      
        %X=MyKinematicModel(X,vw,dt);

        [Xe,P] = MyPredictionStepEKF(Xe,P,vw,dt,Pu);
    
    
        %X_truck for the finale plot
         X_truck(1,i)  = Xe(1);
         X_truck(2,i)  = Xe(2);
         theta           = Xe(3);
        %the UGV current pos, part_C:
        % length of the arrow
        L = 1.5;
        % Calculate the x and y components of the arrow
        u = L*cos(theta);
        v = L*sin(theta);
        %plot the ugv dynamicly with a arrow sticking out:
        set(plot_envirnment(1),'xdata',Xe(1),'ydata',Xe(2),'UData',u,'VData',v);
        
            
        index = event(2);  % where to read the actual measurement, from that sensor recorder.
            
        switch sensorID    % measurement is from which sensor?
            
            case 1  %  it is a scan from  LiDAR#1, and scan from LiDAR#2! (both, as they are synchronized)
                
            scan1 = data.scans(:,index);  
           
            [Xe,P] = processLiDAR(Xe,P, scan1,Lidar1Cfg,data.Context.Landmarks,AA,plot_envirnment(2));  
            BB.Rec(Xe,P);   % record 
            pause(0.01);
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
    BB.Show(10,'Consistency plots');
    disp('Loop of events ends.');
    
    disp('Showing ground truth (your estimated trajectory should be close).)');
    ShowVerification1(data,X_truck);  

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
% ---------------------------------------------------------------------------------

function ShowVerification1(data,X_truck); 


figure(11)
hold on;
p=data.verify.poseL;
plot(p(1,:),p(2,:),'r.');
hold on;
plot(X_truck(1,:),X_truck(2,:), 'k-'); %plot the data from the trucking
legend({'Landmarks','Walls (middle planes)','Initial pose','Ground truth (subsampled)'});
end


% ---------------------------------------------------------------------------------

function [Xe,P] = processLiDAR(Xe,P, scan1,Lidar1Cfg,landmarks,AA, figure_handle)  

   
    
    mask1 = 16383;   %  0x7FFF % extract lowest 14 bits (actual ranges, in cm).
    mask2 = 49152;   %  0xC000 % extract highest 2 bits ("intensity")
    ranges = bitand(scan1, mask1);        % in cms, uint16
    ranges = single(ranges)*0.01;        % now in meters, single precision.
    colors = bitand(scan1,mask2);
    tic0=tic();

    [ii12,props,numOOIs] = AA.FindSmallSegmentsFS(ranges,0.3);  %props = OOI-center
        % * props: (OOIs' properties)
        %   props (:,3) = angle values of OOI-center 
        %   props (:,2) = ranges values of OOI-center
    
    if (numOOIs<1), return; end;

    OOIGCF = OOIs_from_local_to_global (props,Xe,Lidar1Cfg);
    OOIGCF=OOIGCF';
     

    tolerance = 0.8 ; % in the same units you use for OOIs and Landmarks position, e.g., in meters.
    [na,aa,iiO,uuL] = AA.MyDA( OOIGCF(1,:),OOIGCF(2,:),landmarks(1,:), landmarks(2,:), tolerance);
    if (na<1), return; end;
   
    for i = 1 : na
      ym = sqrt((Xe(1)-OOIGCF( 1,iiO(i)))^2 + (Xe(2)- OOIGCF( 2,iiO(i)))^2 );
     [Xe,P] = Do_update_OOI (Xe,P, OOIGCF( :,iiO(i)), landmarks(:,uuL(i)) , Lidar1Cfg,ym );
    end


     %plot the OOI_center dynamicly on GCF
     OOIs_center_x =  OOIGCF(1,:);
     OOIs_center_y =  OOIGCF(2,:);
     for i = 1:length(OOIs_center_x)
          set(figure_handle,'xdata',OOIs_center_x(1:i), 'ydata',OOIs_center_y(1:i));
     end

end

% ---------------------------------------------------------------------------------

function h = CreateFigureToShowScansInPolar()
    
    figure(11);axis([-5,20,-5,25]);hold on;
    h1 = quiver(0,0,'O','LineWidth', 2,'MaxHeadSize', 0.5,'MarkerSize',10,'MarkerFaceColor','r','MarkerEdgeColor','k');
    hold on;
    h2 = plot(0,0,'r*','markersize',3);                   %ooi_center dynamicly lidar1 %16

    h = [h1,h2];
end    
    
%--------------------------------------------------------------------------
function [Xe,P] = MyPredictionStepEKF(Xe,P,vw,dt,Pu)

    J = JacobianX(Xe,vw,dt);
    Ju = JacobianU(Xe,vw,dt);
    Qu = Ju*Pu*Ju';
    
    Xe = MyKinematicModel(Xe,vw,dt);
    P = J*P*J' + Qu;
   

end

%--------------------------------------------------------------------------
function X=MyKinematicModel(X,vw,dt)
       X(1) = X(1) + vw(1) * cos(X(3)) * dt; % x=x0+v*dt
       X(2) = X(2) + vw(1) * sin(X(3)) * dt;
       X(3) = X(3) + vw(2) * dt;
end   

%--------------------------------------------------------------------------
function J = JacobianX(Xe,vw,dt)   %jacobian matrix of process model

    J = [ [1 , 0 , -vw(1)* sin(Xe(3)) *dt];
          [0 , 1 ,  vw(1)* cos(Xe(3)) *dt];
          [0 , 0 ,          1            ]
        ];
end

%--------------------------------------------------------------------------
function Ju = JacobianU(Xe,vw,dt)   %jacobian matrix of input
    Ju = [ [ cos(Xe(3))*dt  ,  0   ];
           [ sin(Xe(3))*dt  ,  0   ];
           [      0         ,  dt  ]
         ]; 
end

%--------------------------------------------------------------------------
function  [Xe,P ] = Do_update_OOI (Xe,P, OOI, landmark,Lidar1Cfg,ym )

    [H,h] =   find_H (Xe,P, OOI, landmark,Lidar1Cfg );
    R = 0.25^2;
    Z = ym - h;
    S = H * P * H' + R;  % H*P*H'->1x1
    K = P * H' * inv(S);

    Xe = Xe + K * Z;
    P = P - P * H' * inv(S) * H * P;

end

%--------------------------------------------------------------------------
function [H,h]  = find_H (Xe,P, OOI, landmark,Lidar1Cfg )
x_car = Xe(1);
y_car = Xe(2);
phi = Xe(3);

x = x_car + Lidar1Cfg.Lx*cos(phi);   % in GCF
y = y_car + Lidar1Cfg.Lx*sin(phi);

%so find the pos to the landmark;
lm = landmark;%(end,:);
xk = lm(1);    %should be the pos of centerOOI , fix it
yk =lm(2);

h =   sqrt((xk-x)^2 + (yk-y)^2 );  % atan2(yk-y , xk-x) -phi   ];  %h2

%fprintf('x=%.2f, y=%.2f, lmx=%.2f, lmy=%.2f, h=%.2f,   \n', x, y, xk, yk, h);
H =[ -(xk-x) / sqrt((xk-x)^2 + (yk-y)^2 ) ,  -(yk-y) / sqrt((xk-x)^2 + (yk-y)^2 ) , 0];

end

%--------------------------------------------------------------------------

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





end



