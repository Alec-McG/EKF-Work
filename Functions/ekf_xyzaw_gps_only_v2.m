function [X_hat, P_out]=ekf_xyzaw_gps_only_v2(enc, gps, acc, P_in, geo_vars, X_hat_prev, variances)

    P = P_in;
    
    omega = acc(1);
    alpha = acc(2);

    b = geo_vars(1);
    r_p = geo_vars(2);
    dtEnc = geo_vars(3);
    
    
    
x_dot = @(vl, vr, a, w,dt) (vr/2 + vl/2)*sin(a)*cos(w)*dt;
y_dot = @(vl, vr, a,dt) (vr/2 + vl/2)*cos(a)*dt;
z_dot = @(vl, vr, a, w,dt) -(vr/2 + vl/2)*sin(a)*sin(w)*dt;
a_dot = @(vl, vr,dt) (1/b)*(vr - vl)*dt;
w_dot = @(vl, vr, a,dt) (vr/2 + vl/2)*sin(a)*dt/r_p;

    varEnc = variances(1);
    varGps = variances(2);

    Q = diag([varEnc varEnc]);
    R = diag([varGps varGps varGps]);
    V = diag([1,1,1]);
    

    H = [1  0   0   0   0
         0  1   0   0   0
         0  0   1   0   0];
 
    vel_r = enc(1);
    vel_l = enc(2);
    v = (vel_r+vel_l)/2;
    
    x_k_1 = X_hat_prev(1);
    y_k_1 = X_hat_prev(2);
    z_k_1 = X_hat_prev(3);
    a_k_1 = X_hat_prev(4);
    w_k_1 = X_hat_prev(5);
    
    
    
    dx = x_dot(vel_r, vel_l, a_k_1, w_k_1,dtEnc);
    dy = y_dot(vel_r, vel_l, a_k_1,dtEnc);
    dz = z_dot(vel_r, vel_l, a_k_1, w_k_1,dtEnc);
    da = a_dot(vel_r, vel_l, dtEnc);
    dw = w_dot(vel_r, vel_l, a_k_1,dtEnc);
    
    x_k =  x_k_1 + dx;
    y_k =  y_k_1 + dy;
    z_k =  z_k_1 + dz;
    a_k =  a_k_1 + da;
    w_k =  w_k_1 + dw;   

    
    X_hat_minus = [x_k; y_k; z_k; a_k; w_k];
    
    A = [1      0       0   v*cos(a_k_1)*cos(w_k_1)*dtEnc      -v*dtEnc*sin(a_k_1)*sin(w_k_1)
         0      1       0        -v*sin(a_k_1)*dtEnc                         0
         0      0       1   -v*cos(a_k_1)*sin(w_k_1)*dtEnc     -v*sin(a_k_1)*cos(w_k_1)*dtEnc
         0      0       0               1                                   0
         0      0       0   v/(r_p)*cos(a_k_1)*dtEnc                       1];
     
     
    W = [(dtEnc/2)*sin(a_k_1)*cos(w_k_1)         (dtEnc/2)*sin(a_k_1)*cos(w_k_1)  
         (dtEnc/2)*cos(a_k_1)                    (dtEnc/2)*cos(a_k_1)
         -(dtEnc/2)*cos(w_k_1)*sin(a_k_1)        -(dtEnc/2)*cos(w_k_1)*sin(a_k_1)
                 dtEnc/b                                      -dtEnc/b
          dtEnc/(2*r_p)*sin(a_k_1)                    dtEnc/(2*r_p)*sin(a_k_1) ];
            
    % Measuremnts stuff
    
    x_gps = gps(1);
    y_gps = gps(2);
    z_gps = gps(3);

    z = [x_gps; y_gps; z_gps];
    
    h = H*X_hat_minus;
    
    % Correction
    
    P = A*P*A' + W*Q*W'; 

    K = P*H'*(H*P*H' + V*R*V')^-1; 

    X_hat = X_hat_minus  + K*(z - h);

    P_out = (eye(5)-K*H)*P;
    


end