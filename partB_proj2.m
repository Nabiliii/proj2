
function Main()
clc;clear;
file='DataUsr_p020.mat';   
load(file); 
ExploreData(data);
end
% -------------------------------------------------------------------------

function ExploreData(data)
    fprintf('Estimating Bias... \n\n');
    
    X_0             = data.pose0;            %platform's initial pose; [x0;y0;heading0] [meters;meters;radians]
    n_events        = data.n ;               % how many events?
    table           = data.table;            % table of events.
    event0          = table(:,1);            % first event.
    t0              = event0(1);             % initial time (the time of event0).
    t0              = 0.0001*double(t0);     % "event" is of integer type, that is why we convert t0 to double ( i.e. to "real") and over to s
    vw              = [0;0];                 % The last [speed,heading rate] measurement.
    Lidar1Cfg    = data.LidarsCfg.Lidar1;  %Info about LiDAR installation (position and orientation,in platform's coordinate frame.). 
    
    %initial som variables:
    measured_phi = [];
    X_kin_mod = zeros(2,n_events);
    X_kin_mod_correct = zeros(2,n_events);
    
    for i = 1:n_events % can use for loop but then buttons do not work.
    
                %pos_buffer(:,i) = X_0;  %record current pose; so, we can analyze the estimated trajectory after the loop ends.
                % get event description.
                event     = table(:,i);                %event #i    -->(:,i) gives me columns                   
                sensorID  = event(3);
                index     = event(2);                  % where to read the actual measurement, from that sensor recorder.% source (i.e. which sensor generated this event?)
                
                tNow      = 0.0001*double(event(1));   % when was this measurement taken? Time in tNow is expressed in seconds.
                dt        = tNow-t0;                   % dt since last event ( "dt" is needed for prediction steps).
                t0        = tNow;                      % remember current time, so we can calculate dt in next iteration.            
                
                
                X_0             = MyKinematicModel(X_0,vw,dt); 
                X_kin_mod(1,i)  = X_0(1);
                X_kin_mod(2,i)  = X_0(2);
                theta           = X_0(3);
                % Calculate angular velocity using diff function
                
                measured_phi = [measured_phi; theta];
                
                 % measurement is from which sensor?
                 switch sensorID   
                    
                    case 1         %  it is a scan from  LiDAR#1, and scan from LiDAR#2! (both, as they are synchronized)
    
                    % both LiDARs are sinchronized.
                    scan1 = data.scans(:,index);  %get the col of scan1, each col is a scan (301 scans).
    
                   pause(0.005);
                   continue;                     
                    
                    %...............................
                    case 2                          %  It is speed encoder + gyro  (they are packed together, synchonized readings)
                    
                    vw   = data.vw(:,index);        % speed and gyroZ, last updated copy. keep the variable "vw" updated to the last reading of those sensors.
                   % i_when_vw_input = [i_when_vw_input; i];
                    continue;
                 end 
    end
    
    %***********************************************
    %get the measurements after the loop
    measured_phi = measured_phi(1:17:end);
    gt = data.verify.poseL(3,:);
    gt = gt';
    % Run the optimization
    bias_initial_guess = 0.1;
    bias_estimate = estimate_bias(bias_initial_guess,measured_phi,gt);
    fprintf('Estimated bias: %.3f deg/s \n\n',bias_estimate);
    disp ('Running vehicle with estimated bias...');
    %***********************************************
    
    %new loop for running vehicle with corrected angular rate:
    X_1             = data.pose0;            %platform's initial pose; [x0;y0;heading0] [meters;meters;radians]
    n_events1        = data.n ;               % how many events?
    table1           = data.table;            % table of events.
    %event1          = table(:,1);            % first event.
    t1              = event0(1);             % initial time (the time of event0).
    t1              = 0.0001*double(t1);     % "event" is of integer type, that is why we convert t0 to double ( i.e. to "real") and over to s
    vw1              = [0;0];  
    plot_envirnment = InitCertainPartOfMyProgram(data); %figure 11 -background 
    for i = 1:n_events1 % can use for loop but then buttons do not work.
            % get event description.
            event1     = table1(:,i);                %event #i    -->(:,i) gives me columns                   
            sensorID  = event1(3);
            index     = event1(2);                  % where to read the actual measurement, from that sensor recorder.% source (i.e. which sensor generated this event?)
            
            tNow1      = 0.0001*double(event1(1));   % when was this measurement taken? Time in tNow is expressed in seconds.
            dt        = tNow1-t1;                   % dt since last event ( "dt" is needed for prediction steps).
            t1        = tNow1;                      % remember current time, so we can calculate dt in next iteration.            
            
            X_1             = MyKinematicModel_corrected(X_1,vw1,dt,bias_estimate); 
            X_kin_mod_correct(1,i)  = X_1(1);
            X_kin_mod_correct(2,i)  = X_1(2);
            phi           = X_1(3);
            
            %the UGV current pos, part_C:
                % length of the arrow
                L = 2;
                % Calculate the x and y components of the arrow
                u = L*cos(phi);
                v = L*sin(phi);
                %plot the ugv dynamicly with a arrow sticking out:
                set(plot_envirnment(1),'xdata',X_1(1),'ydata',X_1(2),'UData',u,'VData',v);

             % measurement is from which sensor?
             switch sensorID   
               case 1         %  it is a scan from  LiDAR#1, and scan from LiDAR#2! (both, as they are synchronized)
                   scan1 = data.scans(:,index);  %get the col of scan1, each col is a scan (301 scans).

                   processLiD_cartiesian_plot(plot_envirnment(2),scan1,X_1,MyKinematicModel_corrected(X_1,vw1,dt,bias_estimate),Lidar1Cfg,1);                                                         
                   pause(0.005);
                   continue;                     
                case 2                          %  It is speed encoder + gyro  (they are packed together, synchonized readings)
                    vw1   = data.vw(:,index);        % speed and gyroZ, last updated copy. keep the variable "vw" updated to the last reading of those sensors.
                    continue;
             end 
    end
    
    ShowVerification1(data,X_kin_mod,X_kin_mod_correct);%,global_pos_OOI1,OOIs_center,lm, OOI );
    plot_for_bias(data,X_kin_mod,X_kin_mod_correct);

end  

% -----------------------------------------------------------------------------------------------------------
function plot_envirnment = InitCertainPartOfMyProgram(data)

    % create some figure for your stuff.
    figure(1); clf();    % global CF.
    
    Landmarks=data.Context.Landmarks;
    % plot centres of landmarks. 
    plot(Landmarks(1,:),Landmarks(2,:),'ko','markersize',3)

    % plot interior of walls (they have ~20cm thickness; but the provided info just includes the ideal center of the walls
    % your LiDAR scans will appear at ~ 10cm from some of those lines.    
    % Wall transversal section:  :  wall left side [<--10cm-->|<--10cm-->] the other  wall side. 
    hold on;
    Walls = data.Context.Walls;
    plot(Walls(1,:),Walls(2,:),'color',[0,1,0]*0.7,'linewidth',3);
    legend({'Centers of landmarks','Walls (middle planes)' });%', 'pos0','car','GT','X','wrong X'});
    
    title('Global CF (you should show some results here)');
    xlabel('X (m)'); 
    ylabel('Y (m)');
    p0=data.pose0;
    plot(p0(1),p0(2),'r*','markersize',10);
    
    
    plot_envirnment = CreateFigureToShowScansInPolar();
end
% ---------------------------------------------------------------------------------
function ShowVerification1(data,X_kin_mod,MyKinematicModel_corrected)%,global_pos1,OOIs_center,matchedPoints1, matchedPoints2 )
figure(1)
hold on;
p=data.verify.poseL;
plot(p(1,:),p(2,:),'r.');
%legend({'Landmarks','Walls (middle planes)','Initial pose','Ground truth (subsampled)'});
hold on;
%plotT he position component of the predicted poses,assuming bias=0,ingreen color.

plot(X_kin_mod(1,:),X_kin_mod(2,:), 'c.','MarkerSize',0.5); 
%hold on;
%plot (global_pos1(:,1),global_pos1(:,2),'b+');

hold on;
plot(MyKinematicModel_corrected(1,:),MyKinematicModel_corrected(2,:), 'b.','MarkerSize',1); 


end
% ---------------------------------------------------------------------------------

function figur_handles = CreateFigureToShowScansInPolar()
   
    figure(1); 
    movegui(gcf, [70 400]);

    hold on;
    h1 = quiver(0,0,'O','LineWidth', 2,'MaxHeadSize', 0.5,'MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','k');
    %h5 = quiver(0,0,'s','LineWidth', 1,'MarkerFaceColor','r','MarkerEdgeColor','k');
    title('OOIs globl CF');  xlabel('X (m)');  ylabel('Y (m)'); axis([-5,21,-5,26]); grid on;
    hold on;  
    h2 = plot(0,0,'m.','MarkerSize',1);     %dots from lidar1             %10
    
    figure(2);
    hold on;
    title('Part E Bias');xlabel('X (m)');  ylabel('Y (m)'); axis([-5,20,-5,20]);grid on;
     movegui(gcf, [640 200]);
   
    figur_handles = [h1,h2];
end    
% ---------------------------------------------------------------------------------

function X = MyKinematicModel(X,vw,dt)
     % Here, you implement your discrete time model; e.g. using Euler's approach.
       X(1) = X(1) + vw(1) * cos(X(3)) * dt; % x=x0+v*dt
       X(2) = X(2) + vw(1) * sin(X(3)) * dt;
       X(3) = X(3) + vw(2) * dt;
end  
% ---------------------------------------------------------------------------------
function X = MyKinematicModel_corrected(X,vw,dt,bias)

       vw_tun = vw + bias;  

     % Here, you implement your discrete time model; e.g. using Euler's approach.
       X(1) = X(1) + vw(1) * cos(X(3)) * dt; % x=x0+v*dt
       X(2) = X(2) + vw(1) * sin(X(3)) * dt;
       X(3) = X(3) + vw_tun(2) * dt;
end  
% ---------------------------------------------------------------------------------
function  processLiD_cartiesian_plot(figure_handle,scan,T,X_kin_mod, LidarCfg,id) %plot scans and OOI in cartes. LiDaR_CF and global
  
    
        %plot dynamicaly the scans in cartesian in global_CF:
        [scans_global_pos] = scan_local_to_global(scan,id, T,X_kin_mod,LidarCfg);
       set(figure_handle,'xdata',scans_global_pos(:,1), 'ydata',scans_global_pos(:,2));


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
function [scans_global_pos] = scan_local_to_global(scan,id, T,X_kin_mod_correct,LidarCfg) %Retrun the all dots in Global_CF
    %the heading of the vehicle:
    theta = T(3);
    % lidar position in GCF (assuming scalar):
    x_origin = X_kin_mod_correct(1); 
    y_origin = X_kin_mod_correct(2); 
    %Lidar pos in UGV:
    lx = LidarCfg.Lx;
    ly = LidarCfg.Ly;
%-----------------------------
    % alpha =  LidarCfg.Alpha for lidar1 and + some corrector for lidar2:
    if id == 1
        alpha = LidarCfg.Alpha;
    end
    if id == 2
        tun = corrct_angle();   % =5 for p020  and -6 for p2021
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

function c = Cost(Bias,measured_phi , GR_thruth_phi)
 
k = diff(measured_phi)*pi/180;
j = diff(GR_thruth_phi)*pi/180;
dt = 0.00276;
d = k/dt + Bias - j/dt;
normen = norm(d);
c = sum(normen);        
end

% ---------------------------------------------------------------------------------
function  bias_estimate = estimate_bias(bias_initial_guess,measured_phi,GR_thruth_phi)

    Op=optimset('TolFun',0.01);
    tic();
    [bias_estimate,fval,exitflag] = fminsearch(@(Bias) Cost(Bias,measured_phi , GR_thruth_phi), bias_initial_guess,Op);  %Solution
    dt=toc()*1000;
    fprintf('Processing tim = %.3f ms \n ',dt);

    

end
% ---------------------------------------------------------------------------------

function plot_for_bias(data,X_kin_mod,X_kin_mod_correct)
    figure(2)
    hold on;
    p=data.verify.poseL;
    plot(p(1,:),p(2,:),'r.','markersize',1);
    %legend({'Landmarks','Walls (middle planes)','Initial pose','Ground truth (subsampled)'});
    hold on;
    plot(X_kin_mod_correct(1,:),X_kin_mod_correct(2,:), 'k-'); %plot the data from the kinematic model

    hold on;
    plot(X_kin_mod(1,:),X_kin_mod(2,:), 'c-','LineWidth',2); %plot the data from the kinematic model

    legend({'ground Truth','kin. model corrected','kin. model biased'});
   
 end


