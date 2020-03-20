close all
clear all

%% Generate Encoder Readings

% ----- Variables ----- %

r_p = 250;
b   = 100;
l   = 125; 
pipe_length = 500;
pipe_angle = 0*pi/180;
height = 0;
plotStuff = 1;
plotinloop = 0;
revols = 20;

pipe_centre = [0  0 0];
%pipe_centre2 = [0 pipe_length*cos(pipe_angle) pipe_length*sin(pipe_angle)];
pipe_centre2 = [0 pipe_length*cos(pipe_angle) pipe_length*sin(pipe_angle)];


f = 100; %f should be devisable by fEnc and fPos otherwise decimate function messes up
dt = 1/f;
vMax = 20; %mm/s

fEnc = 50; %Hz: Needs to be a multiple of fPos
fGps = 5; %Hz
fAcc = 25; %Hz

dtEnc = 1/fEnc; %Encoder Timestep
varEnc = 0.5^2; 

dtGps = 1/fGps;
varGps = 20^2; %mm^2
varPosAngle = atan2(varGps,-varGps);

dtAcc = 1/fAcc;
varAcc = deg2rad(10)^2; %angles in rad


varOmegaAcc = varAcc;
varAlphaAcc = varAcc;
varOmegaGps = varPosAngle; 

w_0 = deg2rad(0.01);
a_0 = deg2rad(90);
a_dot_0 = 0;
w_dot_0 = 0.004;
x_0 = r_p*sin(w_0);
y_0 = 100;
z_0 = r_p*cos(w_0);

p0 = [x_0 y_0 z_0 a_0 w_0];
geo_vars = [r_p b];

%% ------ GENERATE IDEAL POS ------ %%

disp('Generateing Control Inputs...')

[x_ideal, y_ideal, z_ideal, a_ideal, w_ideal,w_dot_ideal, control_input] = genIdealPos_ekf(vMax,f, p0, geo_vars, revols);

x_dec_enc = decimate(x_ideal, ceil(f/fEnc),'fir');
y_dec_enc = decimate(y_ideal, ceil(f/fEnc),'fir');
z_dec_enc = decimate(z_ideal, ceil(f/fEnc),'fir');
a_dec_enc = decimate(a_ideal, ceil(f/fEnc),'fir');
w_dec_enc = decimate(w_ideal, ceil(f/fEnc),'fir');
w_dot_dec_enc = decimate(w_dot_ideal,ceil(f/fEnc),'fir');

wheel_control(:,1) = decimate(control_input(:,1), ceil(f/fEnc),'fir');
wheel_control(:,2) = decimate(control_input(:,2), ceil(f/fEnc),'fir');










wheel_control = wheel_control(1:end-1,:);


if plotStuff == 1
    
   figure
   subplot(1,2,1)
   plot3(x_ideal, y_ideal, z_ideal)
   title('Ideal Position')
   xlabel('X')
   ylabel('Y')
   zlabel('Z')
   
   subplot(1,4,3)
   plot(a_ideal)
   title('Ideal \alpha')
   xlabel('')
   ylabel('\alpha')
   
   subplot(1,4,4)
   plot(w_ideal)
   title('Ideal \omega')
   xlabel('')
   ylabel('\omega')

    
end

disp('Generating Encoder Measurments...')

[x_enc, y_enc, z_enc, a_enc, w_enc, w_dot_enc, wheel_control_error] = genEncoder_ekf(varEnc,vMax, wheel_control, p0, dtEnc,geo_vars);

dead_reck = [x_enc, y_enc, z_enc, a_enc, w_enc, w_dot_enc];

if plotStuff == 1
    
   figure
   subplot(1,2,1)
   plot3(x_enc, y_enc, z_enc)
   title('enc Position')
   xlabel('X')
   ylabel('Y')
   zlabel('Z')
   axis equal
   
   subplot(1,4,3)
   plot(a_enc)
   title('enc \alpha')
   xlabel('')
   ylabel('\alpha')
   
   subplot(1,4,4)
   plot(w_enc)
   title('enc \omega')
   xlabel('')
   ylabel('\omega')

    
end

disp('Generating Accelerometer and GPS Measurments...')

geo(1) = r_p;
geo(2) = b; 
geo(3) = l;
geo(4) = y_0;
geo(6) = pipe_angle;
geo(7) = height;

x_dec_gps = decimate(x_dec_enc, ceil(fEnc/fGps),'fir');
y_dec_gps = decimate(y_dec_enc, ceil(fEnc/fGps),'fir');
z_dec_gps = decimate(z_dec_enc, ceil(fEnc/fGps),'fir');
a_dec_gps = decimate(a_dec_enc, ceil(fEnc/fGps),'fir');
w_dec_gps = decimate(w_dec_enc, ceil(fEnc/fGps),'fir');
w_dot_dec_gps = decimate(w_dot_dec_enc, ceil(fEnc/fGps),'fir');

igps_meas = [(r_p*sin(w_dec_gps)+ sqrt(varGps)*randn(length(w_dec_gps),1).*ones(length(w_dec_gps),1)) y_0*ones(length(w_dec_gps),1)+ sqrt(varGps)*randn(length(w_dec_gps),1).*ones(length(w_dec_gps),1) r_p*cos(w_dec_gps)+ sqrt(varGps)*randn(length(w_dec_gps),1).*ones(length(w_dec_gps),1)];

a_dec_acc = decimate(a_dec_enc, ceil(fEnc/fAcc),'fir');
w_dec_acc = decimate(w_dec_enc, ceil(fEnc/fAcc),'fir');

alpha_meas = a_dec_acc + sqrt(varAcc)*randn(length(a_dec_acc),1);
omega_meas = w_dec_acc + sqrt(varAcc)*randn(length(w_dec_acc),1);
%[alpha_meas, omega_meas, igps_meas] = genAccGPS_ekf(w_dec, a_dec, y_dec, geo, varAcc, varGps);

accel_cart = [r_p*sin(omega_meas) ones(1,length(omega_meas))'*y_0 r_p*cos(omega_meas) alpha_meas omega_meas];
enc_cart = [x_enc y_enc z_enc a_enc w_enc];

if plotStuff == 1
    
   figure
   subplot(1,2,1)
   scatter3(igps_meas(:,1), igps_meas(:,2), igps_meas(:,3))
   title('gps Position')
   xlabel('X')
   ylabel('Y')
   zlabel('Z')
   axis equal
   
   subplot(1,4,3)
   plot(alpha_meas)
   title('acc \alpha')
   xlabel('')
   ylabel('\alpha')
   
   subplot(1,4,4)
   plot(omega_meas)
   title('acc \omega')
   xlabel('')
   ylabel('\omega')
   
   disp('Comparing Measurements...')
   
   figure
   subplot(1,2,1)
   hold on
   scatter3(igps_meas(:,1), igps_meas(:,2), igps_meas(:,3),'g')
   scatter3(x_dec_enc,y_dec_enc, z_dec_enc,'b')
   scatter3(x_enc,y_enc, z_enc,'k')
   axis equal
   
   subplot(1,4,3)
   hold on
   plot(alpha_meas,'m')
   plot(a_enc,'k')
   
   title('Compare \alpha')
   xlabel('')
   ylabel('\alpha')
   
   subplot(1,4,4)
   hold on
   plot(omega_meas,'m')
   plot(w_enc,'k')
   title('Compare \omega')
   xlabel('')
   ylabel('\omega')
   
   figure
   subplot(1,2,1)
   hold on
   scatter3(accel_cart(:,1), accel_cart(:,2), accel_cart(:,3),'m')
   scatter3(x_dec_enc,y_dec_enc, z_dec_enc,'b')
   scatter3(x_enc,y_enc, z_enc,'k')
   title('Acceleration Cart')
   axis equal
   
   subplot(2,2,2)
   hold on
   plot(accel_cart(:,5) - w_dec_acc)
   title('e accel w')

   
end

% if plotinloop==1
%     figure
%     hold on
%     for i = 1:length(x_dec)
%         
%         scatter3(x_dec(i),y_dec(i),z_dec(i),'k');
%         scatter3(x_enc(i),y_enc(i),z_enc(i),'b');
%         scatter3(igps_meas(i,1),igps_meas(i,2),igps_meas(i,3),'g');
%         scatter3(accel_cart(i,1),accel_cart(i,2),accel_cart(i,3),'m');
%         
%         drawnow
%         
%     end
%     
% end

disp('Finished Generating All Measurments!!!')

%% ----- EKF Stuff ----- %%

% ---- Initialisation ---- %

geo_vars = [b, r_p, dtEnc];

%Initial points xyzaw_gps_only
P_asekf_gps = zeros(6);
%P_asekf_gps = diag([1,1,1,1,1,1]);
X_kal_asekf_gps(1,1:6) = [x_0 y_0 z_0 a_0 w_0 w_dot_0]; % check starting pos
variances_asekf_gps = [varEnc varGps];


P_asekf_acc = zeros(6);
%P_asekf_acc = diag([1,1,1,1,1,1]);
X_kal_asekf_acc(1,1:6) = [x_0 y_0 z_0 a_0 w_0 w_dot_0]; % check starting pos
variances_asekf_acc = [varEnc varAcc];


P_ekf = zeros(6);
%P_asekf_acc = diag([1,1,1,1,1,1]);
X_kal_ekf(1,1:6) = [x_0 y_0 z_0 a_0 w_0 w_dot_0]; % check starting pos
variances_ekf = [varEnc varGps varAcc];




% ---- All the EKFS ---- %

h = figure;
aa = 1;
gg = 1;

for i = 2:length(wheel_control_error)+1  
    
    
    enc_meas = wheel_control_error(i-1,:);
    
    X_hat_prev_ekf = X_kal_ekf(i-1,1:6);
    
    
    if mod(i,ceil(fEnc/(fAcc)))==0 && mod(i,ceil(fEnc/(fGps)))==0
        %EKF GPS + ACC
        aa = aa + 1;
        gg = gg + 1;
        
        acc_meas = [omega_meas(aa-1) alpha_meas(aa-1)];
        gps_meas = igps_meas(gg-1,:);
        
        
        [X_hat_ekf, P_ekf_out]=asekf_gps_acc(enc_meas, acc_meas, gps_meas, P_ekf, geo_vars, X_hat_prev_ekf,variances_ekf );
       
        P_ekf = P_ekf_out;
        
    elseif mod(i,ceil(fEnc/fAcc))==0
        %EKF ACC
        aa = aa + 1;
        
        acc_meas = [omega_meas(aa-1) alpha_meas(aa-1)];
        
        [X_hat_ekf, P_ekf_out]=asekf_acc(enc_meas, acc_meas, P_ekf, geo_vars, X_hat_prev_ekf,variances_ekf );

        P_ekf = P_ekf_out;
        
    elseif mod(i,ceil(fEnc/fGps))==0
        %EKF GPS
        gg = gg + 1;
        
        gps_meas = igps_meas(gg-1,:);
        
        [X_hat_ekf, P_ekf_out]=asekf_gps(enc_meas, gps_meas, P_ekf, geo_vars, X_hat_prev_ekf,variances_ekf );

        P_ekf = P_ekf_out;
    else
       %Dead Reckoning 

    y_dot = @(vl, vr, a,dt) (vr/2 + vl/2)*cos(a)*dt;
    a_dot = @(vl, vr,dt) (1/b)*(vr - vl)*dt;
    w_dot = @(vl, vr, a,dt) (vr/2 + vl/2)*sin(a)*dt/r_p;

    da = a_dot(enc_meas(1), enc_meas(2), dtEnc);
    dw = w_dot(enc_meas(1), enc_meas(2), X_hat_prev_ekf(4), dtEnc);
    dy = y_dot(enc_meas(1), enc_meas(2), X_hat_prev_ekf(4), dtEnc);

    x_k =  r_p*sin(X_hat_prev_ekf(5)+X_hat_prev_ekf(6));
    y_k =  X_hat_prev_ekf(2) + dy;
    z_k =  r_p*cos(X_hat_prev_ekf(5)+X_hat_prev_ekf(6));
    a_k =  X_hat_prev_ekf(4) + da;
    w_k =  X_hat_prev_ekf(5) + X_hat_prev_ekf(6);
    w_dot_k = dw;

    X_hat_ekf = [x_k; y_k; z_k; a_k; w_k; w_dot_k];
       
        
    end

     
     
    
    
%% get measurments 


X_hat_ekf_prev = X_hat_ekf;


X_kal_ekf(i,1:6)=X_hat_ekf;
%% Plot in Loop
if plotinloop ==1
    h;
    hold on
    scatter3(gps_meas(1),gps_meas(2),gps_meas(3),5,'g')
    scatter3(X_kal_asekf_gps(i,1),X_kal_asekf_gps(i,2),X_kal_asekf_gps(i,3),5,'r')
    scatter3(x_enc(i), y_enc(i), z_enc(i),5,'b')
    
    
end
end



    figure
    title('Mixed EKF')
    hold on
    scatter3(igps_meas(:,1),igps_meas(:,2),igps_meas(:,3),5,'g')
    plot3(x_enc, y_enc, z_enc,'b','LineWidth',2)
    plot3(X_kal_ekf(:,1),X_kal_ekf(:,2),X_kal_ekf(:,3),'r','LineWidth',2)
    plot3(x_dec_enc, y_dec_enc, z_dec_enc,'k','LineWidth',2)
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    legend('Gps Meas','Encoder','EKF','Ideal')
    grid on
    
      

%% Compare to Ideal
ideal_mat = [x_dec_enc, y_dec_enc, z_dec_enc, a_dec_enc ,w_dec_enc, w_dot_dec_enc];
err_dead_rec = abs(dead_reck-ideal_mat);
err_X_kal_ekf = abs(X_kal_ekf-ideal_mat);


%%

if plotStuff ==1
   
    ss = linspace(0,360*revols,length(X_kal_ekf));
    figure
    subplot(5,1,1)
    hold on
    grid on
    plot(ss,err_dead_rec(:,1),'b')
    plot(ss,err_X_kal_ekf(:,1),'r')

    
    title('X Error')
    legend('dead rec','gps only','acc only', 'gps+acc','Location','eastoutside')
    xlabel('degrees')
    ylabel('mm')
    xlim([0 360*revols])
    
    subplot(5,1,2)
    hold on
    grid on
    plot(ss,err_dead_rec(:,2),'b')
    plot(ss,err_X_kal_ekf(:,2),'r')
    
    title('Y Error')
    legend('dead rec','gps only','acc only', 'gps+acc','Location','eastoutside')
    xlabel('degrees')
    ylabel('mm')
    xlim([0 360*revols])
    
    subplot(5,1,3)
    hold on

    plot(ss,err_dead_rec(:,3),'b')
    plot(ss,err_X_kal_ekf(:,3),'r')
    grid on
    title('Z Error')
    legend('dead rec','gps only','acc only', 'gps+acc','Location','eastoutside')
    xlabel('degrees')
    ylabel('mm')
    xlim([0 360*revols])
    
    subplot(5,1,4)
    hold on
    plot(ss,err_dead_rec(:,4),'b')
    plot(ss,err_X_kal_ekf(:,4),'r')
    grid on
    title('\alpha Error')
    legend('dead rec','gps only','acc only', 'gps+acc','Location','eastoutside')
    xlabel('degrees')
    ylabel('deg')
    xlim([0 360*revols])
    
    subplot(5,1,5)
    hold on
    plot(ss,err_dead_rec(:,5),'b')
    plot(ss,err_X_kal_ekf(:,5),'r')
    grid on
    title('\omega Error')
    legend('dead rec','gps only','acc only', 'gps+acc','Location','eastoutside')
    xlabel('degrees')
    ylabel('deg')
    xlim([0 360*revols])


    
    figure
    title('XYZ Error')
    hold on
    mag_err_enc = sqrt(err_dead_rec(:,1).^2 +err_dead_rec(:,3).^2+err_dead_rec(:,2));
    mag_err_ekf = sqrt(err_X_kal_ekf(:,1).^2 +err_X_kal_ekf(:,3).^2+err_X_kal_ekf(:,2).^2);
   
    
    plot(ss,mag_err_enc,'b')
    plot(ss,mag_err_ekf,'r')

    legend('Dead Rec','Gps + Acc ','GPS','Acc')
    xlabel('deg')
    ylabel('mm')
    grid on
    
    %%
    
    ss = linspace(0,360*revols,length(X_kal_ekf));
    figure
    subplot(3,1,1)
    title('XYZ Error')
    hold on
    mag_err_enc = sqrt(err_dead_rec(:,1).^2 +err_dead_rec(:,3).^2+err_dead_rec(:,2));
    mag_err_ekf = sqrt(err_X_kal_ekf(:,1).^2 +err_X_kal_ekf(:,3).^2+err_X_kal_ekf(:,2).^2);
   
    
    plot(ss,mag_err_enc,'b')
    plot(ss,mag_err_ekf,'r')

    legend('Dead Rec','EKF')
    xlabel('deg')
    ylabel('mm')
    grid on
    
    subplot(3,1,2)  
    hold on
    plot(ss,rad2deg(err_dead_rec(:,4)),'b')
    plot(ss,rad2deg(err_X_kal_ekf(:,4)),'r')
    grid on
    title('\alpha Error')
    legend('Dead Rec','EKF')
    xlabel('degrees')
    ylabel('deg')
    xlim([0 360*revols])
    
    subplot(3,1,3)
    hold on
    plot(ss,rad2deg(err_dead_rec(:,5)),'b')
    plot(ss,rad2deg(err_X_kal_ekf(:,5)),'r')
    grid on
    title('\omega Error')
    legend('Dead Rec','EKF')
    xlabel('degrees')
    ylabel('deg')
    xlim([0 360*revols])
   
    
end



