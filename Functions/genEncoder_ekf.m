function [x_enc, y_enc, z_enc, a_enc, w_enc, w_dot_enc, wheel_control_error] = genEncoder_ekf(varEnc,vMax, wheel_control, p0, dtEnc,geo_vars)


r_p = geo_vars(1);
b = geo_vars(2);

% x_dot = @(vl, vr, a, w,dt) (vr/2 + vl/2)*sin(a)*cos(w)*dt;
% y_dot = @(vl, vr, a,dt) (vr/2 + vl/2)*cos(a)*dt;
% z_dot = @(vl, vr, a, w,dt) -(vr/2 + vl/2)*sin(a)*sin(w)*dt;
% a_dot = @(vl, vr,dt) (1/b)*(vr - vl)*dt;
% w_dot = @(vl, vr, a,dt) (vr/2 + vl/2)*sin(a)*dt/r_p;
    
y_dot = @(vl, vr, a,dt) (vr/2 + vl/2)*cos(a)*dt;
a_dot = @(vl, vr,dt) (1/b)*(vr - vl)*dt;
w_dot = @(vl, vr, a,dt) (vr/2 + vl/2)*sin(a)*dt/r_p;

x_enc = zeros(1,length(wheel_control))';
y_enc = zeros(1,length(wheel_control))';
z_enc = zeros(1,length(wheel_control))';
a_enc = zeros(1,length(wheel_control))';
w_enc = zeros(1,length(wheel_control))';
w_dot_enc = zeros(1,length(wheel_control))';
wheel_control_error = zeros(2,length(wheel_control(:,1)))';

x_enc(1) = p0(1);
y_enc(1) = p0(2);
z_enc(1) = p0(3);
a_enc(1) = p0(4);
w_enc(1) = p0(5);


    
    wheel_control_error = [vMax+sqrt(varEnc)*randn(length(wheel_control_error),1)  vMax+sqrt(varEnc)*randn(length(wheel_control_error),1)];
    


for i = 1:length(wheel_control_error)
 

      
     vr = wheel_control_error(i,1);
     vl = wheel_control_error(i,2);
     
    da = a_dot(vr, vl, dtEnc);
    dw = w_dot(vr, vl, a_enc(i),dtEnc);
    dy = y_dot(vr, vl, a_enc(i),dtEnc);
    
    
    x_enc(i+1) =  r_p*sin(w_enc(i)+w_dot_enc(i));
    y_enc(i+1) =  y_enc(i) + dy;
    z_enc(i+1) =  r_p*cos(w_enc(i)+w_dot_enc(i));
    a_enc(i+1) =  a_enc(i) + da;
    w_enc(i+1) =  w_enc(i) + w_dot_enc(i);
    w_dot_enc(i+1) = dw;
    
end






end