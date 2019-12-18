function [alpha, omega, gps] = genAccGPS_ekf(w_vals, a_vals, y_vals, geo, varAcc, varGps)


gps = zeros(length(w_vals),3);
omega = zeros(length(w_vals),1);
alpha = zeros(length(w_vals),1);
    
r_p = geo(1);
Y_p = geo(4);

for i = 1:length(w_vals) 
    
    geo(4) = y_vals(i);

  
    a_sim = a_vals(i);
    w_sim = w_vals(i);
    
    % Right and Left side of pipe - Assuming known
    
    if (deg2rad(1) < w_sim && w_sim < deg2rad(179) ) %Right Side
        side = 0;
        
    elseif (deg2rad(-179)< w_sim && w_sim < deg2rad(-1) || deg2rad(181) < w_sim && w_sim < deg2rad(359) ) %Left Side
        side = 1;
        
    else % Top or Bottom 
        side = 0;        
        
    end
    
    geo(5) = side;
    height = geo(7);
    
    %% plot
    
    

    [wheel_sim, acc_sim] = Forward_Model_ekf(a_sim, w_sim, geo, 0);

    %pcent_prime = ((wheel_sim(1,:) + wheel_sim(2,:))/2);

    % Set point of igps receiver from robot plane centre

    %[c, n] = compute_plane_coefficients(wheel_sim');
    
    p_igps_ideal = [r_p*sin(w_sim) Y_p r_p*cos(w_sim)];
    %p_igps_ideal = p_igps_ideal + (-n)*height;
    
    %p_igps_ideal = pcent_prime+(-n)*height;


    gps(i,1) = p_igps_ideal(1) + sqrt(varGps)*randn();
    gps(i,2) = p_igps_ideal(2) + sqrt(varGps)*randn();
    gps(i,3) = p_igps_ideal(3) + sqrt(varGps)*randn();


    omega(i) = w_sim + sqrt(varAcc)*randn();
    alpha(i) = a_sim + sqrt(varAcc)*randn();
    


    
end






end