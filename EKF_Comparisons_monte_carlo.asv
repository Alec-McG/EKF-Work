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
revols = 1;
sims = 500;

% pipe_centre = [0  0 0];
% %pipe_centre2 = [0 pipe_length*cos(pipe_angle) pipe_length*sin(pipe_angle)];
% pipe_centre2 = [0 pipe_length*cos(pipe_angle) pipe_length*sin(pipe_angle)];


f = 100; %f should be devisable by fEnc and fPos otherwise decimate function messes up
dt = 1/f;
vMax = 20; %mm/s

fEnc = 10; %Hz: Needs to be a multiple of fPos
fGps = 10; %Hz
fAcc = 10; %Hz

dtEnc = 1/fEnc; %Encoder Timestep
varEnc = 0.5^2; 

dtGps = 1/fGps;
varGps = 20^2; %mm^2
varPosAngle = atan2(varGps,-varGps);

dtAcc = 1/fAcc;
varAcc = deg2rad(5)^2; %angles in rad


varOmegaAcc = varAcc;
varAlphaAcc = varAcc;
varOmegaGps = varPosAngle; 

w_0 = deg2rad(0.001);
a_0 = deg2rad(90);
a_dot_0 = 0;
w_dot_0 = 0.004;
x_0 = r_p*sin(w_0);
y_0 = 0;
z_0 = r_p*cos(w_0);

p0 = [x_0 y_0 z_0 a_0 w_0 w_dot_0];
geo_vars_ideal = [r_p b];

%% ------ GENERATE IDEAL POS ------ %%

disp('Generateing Control Inputs...')

[x_ideal, y_ideal, z_ideal, a_ideal, w_ideal,w_dot_ideal, control_input] = genIdealPos_ekf(vMax,f, p0, geo_vars_ideal, revols);

x_dec = decimate(x_ideal, ceil(f/fEnc),'fir');
y_dec = decimate(y_ideal, ceil(f/fEnc),'fir');
z_dec = decimate(z_ideal, ceil(f/fEnc),'fir');
a_dec = decimate(a_ideal, ceil(f/fEnc),'fir');
%w_dec = atan2(x_dec,z_dec);
w_dec = decimate(w_ideal, ceil(f/fEnc),'fir');
w_dot_dec = decimate(w_dot_ideal,ceil(f/fEnc),'fir');

wheel_control(:,1) = decimate(control_input(:,1), ceil(f/fEnc),'fir');
wheel_control(:,2) = decimate(control_input(:,2), ceil(f/fEnc),'fir');

wheel_control = wheel_control(1:end-1,:);


% if plotStuff == 1
%     
%    figure
%    subplot(1,2,1)
%    plot3(x_ideal, y_ideal, z_ideal)
%    title('Ideal Position')
%    xlabel('X')
%    ylabel('Y')
%    zlabel('Z')
%    
%    subplot(1,4,3)
%    plot(a_ideal)
%    title('Ideal \alpha')
%    xlabel('')
%    ylabel('\alpha')
%    
%    subplot(1,4,4)
%    plot(w_ideal)
%    title('Ideal \omega')
%    xlabel('')
%    ylabel('\omega')
% 
%     
% end

disp('Generating Encoder Measurments...')



[x_enc, y_enc, z_enc, a_enc, w_enc, w_dot_enc, wheel_control_error] = genEncoder_ekf(varEnc,vMax, wheel_control, p0, dtEnc,geo_vars_ideal);

dead_reck = [x_enc, y_enc, z_enc, a_enc, w_enc, w_dot_enc];

% if plotStuff == 1
%     
%    figure
%    subplot(1,2,1)
%    plot3(x_enc, y_enc, z_enc)
%    title('enc Position')
%    xlabel('X')
%    ylabel('Y')
%    zlabel('Z')
%    axis equal
%    
%    subplot(1,4,3)
%    plot(a_enc)
%    title('enc \alpha')
%    xlabel('')
%    ylabel('\alpha')
%    
%    subplot(1,4,4)
%    plot(w_enc)
%    title('enc \omega')
%    xlabel('')
%    ylabel('\omega')
% 
%     
% end

disp('Generating Accelerometer and GPS Measurments...')

geo(1) = r_p;
geo(2) = b; 
geo(3) = l;
geo(4) = y_0;
geo(6) = pipe_angle;
geo(7) = height;


% igps_meas = [(r_p*sin(w_dec)+ sqrt(varGps)*randn(length(w_dec),1).*ones(length(w_dec),1)) y_0*ones(length(w_dec),1)+ sqrt(varGps)*randn(length(w_dec),1).*ones(length(w_dec),1) r_p*cos(w_dec)+ sqrt(varGps)*randn(length(w_dec),1).*ones(length(w_dec),1)];
% 
% alpha_meas = a_dec + sqrt(varAcc)*randn(length(a_dec),1);
% omega_meas = w_dec + sqrt(varAcc)*randn(length(w_dec),1);
% %[alpha_meas, omega_meas, igps_meas] = genAccGPS_ekf(w_dec, a_dec, y_dec, geo, varAcc, varGps);
% 
% accel_cart = [r_p*sin(omega_meas) ones(1,length(omega_meas))'*y_0 r_p*cos(omega_meas) alpha_meas omega_meas];
% enc_cart = [x_enc y_enc z_enc a_enc w_enc];

% if plotStuff == 1
%     
%    figure
%    subplot(1,2,1)
%    scatter3(igps_meas(:,1), igps_meas(:,2), igps_meas(:,3))
%    title('gps Position')
%    xlabel('X')
%    ylabel('Y')
%    zlabel('Z')
%    axis equal
%    
%    subplot(1,4,3)
%    plot(alpha_meas)
%    title('acc \alpha')
%    xlabel('')
%    ylabel('\alpha')
%    
%    subplot(1,4,4)
%    plot(omega_meas)
%    title('acc \omega')
%    xlabel('')
%    ylabel('\omega')
%    
%    disp('Comparing Measurements...')
%    
%    figure
%    subplot(1,2,1)
%    hold on
%    scatter3(igps_meas(:,1), igps_meas(:,2), igps_meas(:,3),'g')
%    scatter3(x_dec,y_dec, z_dec,'b')
%    scatter3(x_enc,y_enc, z_enc,'k')
%    axis equal
%    
%    subplot(1,4,3)
%    hold on
%    plot(alpha_meas,'m')
%    plot(a_enc,'k')
%    
%    title('Compare \alpha')
%    xlabel('')
%    ylabel('\alpha')
%    
%    subplot(1,4,4)
%    hold on
%    plot(omega_meas,'m')
%    plot(w_enc,'k')
%    title('Compare \omega')
%    xlabel('')
%    ylabel('\omega')
%    
%    figure
%    subplot(1,2,1)
%    hold on
%    scatter3(accel_cart(:,1), accel_cart(:,2), accel_cart(:,3),'m')
%    scatter3(x_dec,y_dec, z_dec,'b')
%    scatter3(x_enc,y_enc, z_enc,'k')
%    title('Acceleration Cart')
%    axis equal
%    
%    subplot(2,2,2)
%    hold on
%    plot(accel_cart(:,5) - w_dec)
%    title('e accel w')

   
%end

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
variances_asekf_gps = [varEnc varGps varAcc];


P_asekf_acc = zeros(6);
%P_asekf_acc = diag([1,1,1,1,1,1]);
X_kal_asekf_acc(1,1:6) = [x_0 y_0 z_0 a_0 w_0 w_dot_0]; % check starting pos
variances_asekf_acc = [varEnc varGps varAcc];


P_asekf_gps_acc = zeros(6);
%P_asekf_acc = diag([1,1,1,1,1,1]);
X_kal_asekf_gps_acc(1,1:6) = [x_0 y_0 z_0 a_0 w_0 w_dot_0]; % check starting pos
variances_asekf_gps_acc = [varEnc varGps varAcc];


ideal_end = [x_dec(end), y_dec(end), z_dec(end)];

% ---- All the EKFS ---- %

h = figure;

for j = 1:sims
    %% Generate Measurments 
    
    [x_enc, y_enc, z_enc, a_enc, w_enc, w_dot_enc, wheel_control_error] = genEncoder_ekf(varEnc,vMax, wheel_control, p0, dtEnc,geo_vars_ideal);

    dead_reck = [x_enc, y_enc, z_enc, a_enc, w_enc, w_dot_enc];
    
    igps_meas = [(r_p*sin(w_dec)+ sqrt(varGps)*randn(length(w_dec),1).*ones(length(w_dec),1)) y_0*ones(length(w_dec),1)+ sqrt(varGps)*randn(length(w_dec),1).*ones(length(w_dec),1) r_p*cos(w_dec)+ sqrt(varGps)*randn(length(w_dec),1).*ones(length(w_dec),1)];

    alpha_meas = a_dec + sqrt(varAcc)*randn(length(a_dec),1);
    omega_meas = w_dec + sqrt(varAcc)*randn(length(w_dec),1);



    for i = 2:length(wheel_control_error)+1  


        %% get measurments 
        enc_meas = wheel_control_error(i-1,:);
        acc_meas = [omega_meas(i-1) alpha_meas(i-1)];
        gps_meas = igps_meas(i-1,:);

        %% do function

        X_hat_prev_asekf_gps = X_kal_asekf_gps(i-1,1:6);
        [X_hat_asekf_gps, P_asekf_gps_out]=asekf_gps(enc_meas, gps_meas, P_asekf_gps, geo_vars, X_hat_prev_asekf_gps,variances_asekf_gps );
        P_asekf_gps = P_asekf_gps_out;

        X_hat_prev_asekf_acc = X_kal_asekf_acc(i-1,1:6);
        [X_hat_asekf_acc, P_asekf_acc_out]=asekf_acc(enc_meas, acc_meas, P_asekf_acc, geo_vars, X_hat_prev_asekf_acc,variances_asekf_acc );
        P_asekf_acc = P_asekf_acc_out;

        X_hat_prev_asekf_gps_acc = X_kal_asekf_gps_acc(i-1,1:6);
        [X_hat_asekf_gps_acc, P_asekf_gps_out_acc]=asekf_gps_acc(enc_meas, acc_meas, gps_meas, P_asekf_gps_acc, geo_vars, X_hat_prev_asekf_gps_acc,variances_asekf_gps_acc );
        P_asekf_gps_acc = P_asekf_gps_out_acc;

        %% set X_hat to X_hat_prev
        X_hat_prev_asekf_gps = X_hat_asekf_gps; 
        X_hat_prev_asekf_acc = X_hat_asekf_acc; 
        X_hat_prev_asekf_gps_acc = X_hat_asekf_gps_acc; 

        %% Save and Convert
        %wya_wa
        X_kal_asekf_gps(i,1:6)=X_hat_asekf_gps;
        X_kal_asekf_acc(i,1:6)=X_hat_asekf_acc;
        X_kal_asekf_gps_acc(i,1:6)=X_hat_asekf_gps_acc;
        %% Plot in Loop

    end

%         figure
%         title('Cartesian Comparison GPS')
%         hold on
%         scatter3(igps_meas(:,1),igps_meas(:,2),igps_meas(:,3),5,'g')
%         plot3(x_enc, y_enc, z_enc,'b','LineWidth',2)
%         plot3(X_kal_asekf_gps(:,1),X_kal_asekf_gps(:,2),X_kal_asekf_gps(:,3),'r','LineWidth',2)
%         plot3(x_dec, y_dec, z_dec,'k','LineWidth',2)
%         xlabel('X [mm]')
%         ylabel('Y [mm]')
%         zlabel('Z [mm]')
%         legend('Gps Meas','Encoder','EKF','Ideal')
%         grid on
% 
%         figure
%         title('Cartesian Comparison Acc')
%         hold on
% 
%         plot3(x_enc, y_enc, z_enc,'b','LineWidth',2)
%         plot3(accel_cart(:,1), accel_cart(:,2),accel_cart(:,3),'m','LineWidth',2)
%         plot3(X_kal_asekf_acc(:,1),X_kal_asekf_acc(:,2),X_kal_asekf_acc(:,3),'r','LineWidth',2)
%         plot3(x_dec, y_dec, z_dec,'k','LineWidth',2)
%         xlabel('X [mm]')
%         ylabel('Y [mm]')
%         zlabel('Z [mm]')
%         legend('Encoder','Acc Meas','EKF','Ideal')
%         grid on
% 
%         figure
%         title('Cartesian Comparison GPS + Acc')
%         hold on
%         scatter3(igps_meas(:,1),igps_meas(:,2),igps_meas(:,3),5,'g')
%         plot3(x_enc, y_enc, z_enc,'b','LineWidth',2)
%         plot3(accel_cart(:,1), accel_cart(:,2),accel_cart(:,3),'m','LineWidth',2)
%         plot3(X_kal_asekf_gps_acc(:,1),X_kal_asekf_gps_acc(:,2),X_kal_asekf_gps_acc(:,3),'r','LineWidth',2)
%         plot3(x_dec, y_dec, z_dec,'k','LineWidth',2)
%         xlabel('X [mm]')
%         ylabel('Y [mm]')
%         zlabel('Z [mm]')
%         grid on
%         legend('Gps','Encoder','Acc Meas','EKF','Ideal')
% 
%         figure
%         title('Cartesian Comparison EKFs')
%         hold on
%         grid on
%         plot3(x_enc, y_enc, z_enc,'b','LineWidth',2)
%         plot3(X_kal_asekf_gps(:,1),X_kal_asekf_gps(:,2),X_kal_asekf_gps(:,3),'g','LineWidth',2)
%         plot3(X_kal_asekf_acc(:,1),X_kal_asekf_acc(:,2),X_kal_asekf_acc(:,3),'m','LineWidth',2)
%         plot3(X_kal_asekf_gps_acc(:,1),X_kal_asekf_gps_acc(:,2),X_kal_asekf_gps_acc(:,3),'r','LineWidth',2)
%         plot3(x_dec, y_dec, z_dec,'k','LineWidth',2)
%         xlabel('X [mm]')
%         ylabel('Y [mm]')
%         zlabel('Z [mm]')
%         legend('Encoder','GPS','Accelerometer','GPS + Acceleroemter','Ideal')

    %% Calcualte and Save stuff for histograms
    
    % Save Last Point
    ekf_gps_ends(j,:)= X_kal_asekf_gps(end,:);
    ekf_acc_ends(j,:)= X_kal_asekf_acc(end,:);
    ekf_gps_acc_ends(j,:)= X_kal_asekf_gps_acc(end,:);
    ekf_enc(j,:)= [x_enc(end), y_enc(end), z_enc(end)];

    % calculate distance error
    end_diff_gps(j,:) = (ideal_end - ekf_gps_ends(j,1:3));
    end_diff_acc(j,:) =(ideal_end - ekf_acc_ends(j,1:3));
    end_diff_gps_acc(j,:) = (ideal_end - ekf_gps_acc_ends(j,1:3));
    end_diff_enc(j,:) = (ideal_end - ekf_enc(j,1:3));

    omega_diff_gps(j) = asin((abs(ideal_end(1)*end_diff_gps(j,1)) + abs(ideal_end(3)*end_diff_gps(j,3)))/r_p^2);
    omega_diff_acc(j) = asin((abs(ideal_end(1)*end_diff_acc(j,1)) + abs(ideal_end(3)*end_diff_acc(j,3)))/r_p^2);
    omega_diff_gps_acc(j) = asin((abs(ideal_end(1)*end_diff_gps_acc(j,1)) + abs(ideal_end(3)*end_diff_gps_acc(j,3)))/r_p^2);
    omega_diff_enc(j) = asin((abs(ideal_end(1)*end_diff_enc(j,1)) + abs(ideal_end(3)*end_diff_enc(j,3)))/r_p^2);
    
    %L is in mm
    L_gps_abs(j) = sqrt(r_p^2 * omega_diff_gps(j)^2 +end_diff_gps(j,2)^2);
    L_acc_abs(j) = sqrt(r_p^2 * omega_diff_acc(j)^2 +end_diff_acc(j,2)^2);
    L_gps_acc_abs(j) = sqrt(r_p^2 * omega_diff_gps_acc(j)^2 +end_diff_gps_acc(j,2)^2);
    L_enc_abs(j) = sqrt(r_p^2 * omega_diff_enc(j)^2 +end_diff_enc(j,2)^2);
    
    L_gps(j) = sign(end_diff_gps(j,2))*sqrt(r_p^2 * omega_diff_gps(j)^2 +end_diff_gps(j,2)^2);
    L_acc(j) = sign(end_diff_acc(j,2))*sqrt(r_p^2 * omega_diff_acc(j)^2 +end_diff_acc(j,2)^2);
    L_gps_acc(j) = sign(end_diff_gps_acc(j,2))*sqrt(r_p^2 * omega_diff_gps_acc(j)^2 +end_diff_gps_acc(j,2)^2);
    L_enc(j) = sign(end_diff_enc(j,2))*sqrt(r_p^2 * omega_diff_enc(j)^2 +end_diff_enc(j,2)^2);
    
    
    % Compare to Ideal
    ideal_mat = [x_dec, y_dec, z_dec, a_dec ,w_dec, w_dot_dec];
    err_dead_rec = abs(dead_reck-ideal_mat);
    err_X_kal_asekf_gps = abs(X_kal_asekf_gps-ideal_mat);
    err_X_kal_asekf_acc = abs(X_kal_asekf_acc-ideal_mat);
    err_X_kal_asekf_gps_acc = abs(X_kal_asekf_gps_acc-ideal_mat);

    %%


    j

end

%% Calcualte end distance from ideal



%plot histograms

[N,edges] = histcounts(L_enc_abs, 'Normalization','pdf');
edges = edges(2:end) - (edges(2)-edges(1))/2;
plot(edges, N);


pd_acc_abs = fitdist(L_acc_abs','Normal');
pd_gps_abs = fitdist(L_gps_abs','Normal');
pd_gps_acc_abs = fitdist(L_gps_acc_abs','Normal');
pd_enc_abs = fitdist(L_enc_abs','Normal');

pd_acc = fitdist(L_acc','Normal');
pd_gps= fitdist(L_gps','Normal');
pd_gps_acc = fitdist(L_gps_acc','Normal');
pd_enc = fitdist(L_enc','Normal');


x_dist_abs = [pd_enc_abs.mu - pd_enc_abs.sigma*3:.01:pd_enc_abs.mu + pd_enc_abs.sigma*3];

x_dist = [pd_enc.mu - pd_enc.sigma*3:.01:pd_enc.mu + pd_enc.sigma*3];

figure
hold on
plot(x_dist_abs, normpdf(x_dist_abs,pd_acc_abs.mu,pd_acc_abs.sigma), 'linewidth',2)
plot(x_dist_abs, normpdf(x_dist_abs,pd_gps_abs.mu,pd_gps_abs.sigma), 'linewidth',2)
plot(x_dist_abs, normpdf(x_dist_abs,pd_gps_acc_abs.mu,pd_gps_acc_abs.sigma), 'linewidth',2)
plot(x_dist_abs, normpdf(x_dist_abs,pd_enc_abs.mu,pd_enc_abs.sigma), 'linewidth',2)

xlabel('Error [mm]');
ylabel('Probability Density');
legend('Acc','Gps','Gps and Acc','Enc')
grid on
title('ABS')

figure
hold on
plot(x_dist, normpdf(x_dist,pd_acc.mu,pd_acc.sigma), 'linewidth',2)
plot(x_dist, normpdf(x_dist,pd_gps.mu,pd_gps.sigma), 'linewidth',2)
plot(x_dist, normpdf(x_dist,pd_gps_acc.mu,pd_gps_acc.sigma), 'linewidth',2)
plot(x_dist, normpdf(x_dist,pd_enc.mu,pd_enc.sigma), 'linewidth',2)

xlabel('Error [mm]');
ylabel('Probability Density');
legend('Acc','Gps','Gps and Acc','Enc')
grid on
title('Not ABS')



L_acc_abs
figure
histogram(L_gps_abs)

figure
histogram(L_acc_abs)

figure
histogram(L_gps_acc_abs)

figure
histogram(L_enc_abs)


figure
title('Cartesian Comparison ')
hold on

plot3(x_enc, y_enc, z_enc,'b','LineWidth',2)
plot3(X_kal_asekf_acc(:,1),X_kal_asekf_acc(:,2),X_kal_asekf_acc(:,3),'r','LineWidth',2)
plot3(x_dec, y_dec, z_dec,'k','LineWidth',2)
xlabel('X [mm]')
ylabel('Y [mm]')
zlabel('Z [mm]')
legend('Dead Reckoning','EKF','Ideal')
grid on
axis square

