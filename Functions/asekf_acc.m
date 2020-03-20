function [X_hat, P_out]=asekf_acc(enc, acc, P_in, geo_vars, X_hat_prev, variances)

  
    P = P_in;
    
    omega = acc(1);
    alpha = acc(2);

    b = geo_vars(1);
    r_p = geo_vars(2);
    dtEnc = geo_vars(3);
    
    varEnc = variances(1);
    varGps = variances(2);
    varAcc = variances(3);

    Q = diag([varEnc varEnc]);
    R = diag([varAcc varAcc]);
    V = diag([1,1]); %number of measurments 
    

    H = [0  0   0   1   0   0   
         0  0   0   0   1   0  ];
 
    vel_r = enc(1);
    vel_l = enc(2);
    v = (vel_r+vel_l)/2;
    
    x_k_1 = X_hat_prev(1);
    y_k_1 = X_hat_prev(2);
    z_k_1 = X_hat_prev(3);
    a_k_1 = X_hat_prev(4);
    w_k_1 = X_hat_prev(5);
    w_dot_k_1 = X_hat_prev(6);
    
    y_dot = @(vl, vr, a,dt) (vr/2 + vl/2)*cos(a)*dt;
    a_dot = @(vl, vr,dt) (1/b)*(vr - vl)*dt;
    w_dot = @(vl, vr, a,dt) (vr/2 + vl/2)*sin(a)*dt/r_p;

    da = a_dot(vel_r, vel_l, dtEnc);
    dw = w_dot(vel_r, vel_l, a_k_1,dtEnc);
    dy = y_dot(vel_r, vel_l, a_k_1,dtEnc);

    a_dot_k = da;
    
    x_k =  r_p*sin(w_k_1+w_dot_k_1);
    y_k =  y_k_1 + dy;
    z_k =  r_p*cos(w_k_1+w_dot_k_1);
    a_k =  a_k_1 + da;
    w_k =  w_k_1 + w_dot_k_1;
    w_dot_k = dw;

    
    X_hat_minus = [x_k; y_k; z_k; a_k; w_k; w_dot_k];
    
    A = [0      0       0           0                   r_p*cos(w_k)*cos(w_dot_k)-r_p*sin(w_k)*sin(w_dot_k)              r_p*cos(w_k)*cos(w_dot_k)-r_p*sin(w_k)*sin(w_dot_k)
         0      1       0     -v*sin(a_k)*dtEnc                                 0                                                   0       
         0      0       0           0                          -r_p*sin(w_k)*cos(w_dot_k)-r_p*cos(w_k)*sin(w_dot_k)        -r_p*sin(w_k)*cos(w_dot_k)-r_p*cos(w_k)*sin(w_dot_k)
         0      0       0           1                                           0                                                   0       
         0      0       0           0                                           1                                                   1       
         0      0       0  (v/r_p)*cos(a_k)*dtEnc                               0                                                   0    ];
     
     
    W = [0                                      0
         (dtEnc/2)*sin(a_k)             (dtEnc/2)*sin(a_k)
         0                                      0
         dtEnc/b                            -dtEnc/b
         0                                      0
         (dtEnc/(2*r_p))*sin(a_k)    (dtEnc/(2*r_p))*sin(a_k)];
            
    % Measuremnts stuff
    

    z = [alpha; omega];
    
    h = H*X_hat_minus;
    
    % Correction
    
    P = A*P*A' + W*Q*W'; 

    K = P*H'*(H*P*H' + V*R*V')^-1; 

    X_hat = X_hat_minus  + K*(z - h);

    P_out = (eye(6)-K*H)*P;
    


end