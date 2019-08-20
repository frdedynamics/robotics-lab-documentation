%% Comparison of four different types of filters to find orientation
% In this tutorial, we will learn how to calculate the orientation of our
% mobile phone by using 4 different filters.

%% Read log data
% First, we read out log file and store the variables in Matlab workspace
read_log_script
% On the workspace window, you will see gyro_ts and acc_ts. Those are the
% timeseries (a time-related version of arrays). Those two variables will
% be our main data sets.

%% Extract out numerical entries of data into an array
Gx = gyro_ts.Data(:,1);
Gy = gyro_ts.Data(:,2);
Gz = gyro_ts.Data(:,3);

Ax = acc_ts.Data(:,1);
Ay = acc_ts.Data(:,2);
Az = acc_ts.Data(:,3);

% Our mobile app logs the data in each 10ms.
dt = 0.01;

% Our data lenght is the shortest data set either accelerometer or
% gyroscope.
number_of_data = min(length(gyro_ts.Data),length(acc_ts.Data));

%% 1) Integration of Angular Velocity
phi_hat_gyr   = zeros(1, length(gyro_ts));
theta_hat_gyr = zeros(1, length(gyro_ts));

for i = 2:number_of_data
   p = Gx(i);
   q = Gy(i);
   r = Gz(i);
   
   phi_prev   = phi_hat_gyr(i - 1);
   theta_prev = theta_hat_gyr(i - 1);
    
   phi_hat_gyr(i)   = ...;
   theta_hat_gyr(i) = ...;
end


%% 2) Inclination Sensing
phi_hat_acc   = ...;
theta_hat_acc = ...;
%% 3) Complimentary Filter

% alpha value is related the cut-off frequency of the filters.
alpha = ...;

phi_hat_complimentary   = zeros(1, length(gyro_ts));
theta_hat_complimentary = zeros(1, length(gyro_ts));

for i=2:number_of_data
    p = Gx(i);
    q = Gy(i);
    r = Gz(i);
   
    phi_prev   = phi_hat_complimentary(i - 1);
    theta_prev = theta_hat_complimentary(i - 1);
    
       
    % Complementary filter equations
    % Hint: Use the gyro and acc values which you found in Section 1-2.
    phi_hat_complimentary(i)   = ...;
    theta_hat_complimentary(i) = ...;
end

%% 4) Kalman Filter
A = [1 -dt 0 0; 0 1 0 0; 0 0 1 -dt; 0 0 0 1];
B = [dt 0 0 0; 0 0 dt 0]';
C = [1 0 0 0; 0 0 1 0];
P = eye(4);
Q = eye(4) * 0.01;
R = eye(2) * 10;
state_estimate = [0 0 0 0]';

phi_hat_kalman    = zeros(1, number_of_data);
bias_phi_kalman   = zeros(1, number_of_data);
theta_hat_kalman  = zeros(1, number_of_data);
bias_theta_kalman = zeros(1, number_of_data);

for i=2:number_of_data
    
    p = Gx(i);
    q = Gy(i);
    r = Gz(i);
   
    phi_prev   = phi_hat_kalman(i - 1) + bias_phi_kalman(i-1);
    theta_prev = theta_hat_kalman(i - 1) + bias_theta_kalman(i-1);
    
    % Rotated gyroscope values: - hint: the same as you found in Section 1.
    phi_dot   = ...;
    theta_dot = ...;
          
    % Predict
    state_estimate = A * ...;
    P = A * ...;
    
    % Update (Correct)
    measurement = [phi_hat_acc(i) theta_hat_acc(i)]';
    y_tilde = measurement - C * state_estimate;
    S = R + C * P * C';
    K = ...;
    state_estimate = state_estimate + K * y_tilde;
    P = ...;
    
    phi_hat_kalman(i)    = state_estimate(1);
    bias_phi_kalman(i)   = state_estimate(2);
    theta_hat_kalman(i)  = state_estimate(3);
    bias_theta_kalman(i) = state_estimate(4);
    
end

%% Convert all estimates to degrees
phi_hat_acc = phi_hat_acc * 180.0 / pi; theta_hat_acc = theta_hat_acc * 180.0 / pi;
phi_hat_gyr = phi_hat_gyr * 180.0 / pi; theta_hat_gyr = theta_hat_gyr * 180.0 / pi;
phi_hat_complimentary = phi_hat_complimentary * 180.0 / pi; theta_hat_complimentary = theta_hat_complimentary * 180.0 / pi;
phi_hat_kalman = phi_hat_kalman * 180.0 / pi; theta_hat_kalman = theta_hat_kalman * 180.0 / pi;

%% Plots
t = acc_ts.Time;
plot(t, phi_hat_complimentary);
hold on;
plot(t, phi_hat_acc);
plot(t, phi_hat_gyr);
plot(t, phi_hat_kalman);
legend('Complimentary', 'Accelerometer', 'Gyro', 'Kalman');
xlabel('Time (s)');
ylabel('Angle (Degrees)');
title('Roll');
xlim([0 t(end)])

figure(2);
plot(t, theta_hat_complimentary);
hold on;
plot(t, theta_hat_acc);
plot(t, theta_hat_gyr);
plot(t, theta_hat_kalman);
legend('Complementary', 'Accelerometer', 'Gyro', 'Kalman');
xlabel('Time (s)');
ylabel('Angle (Degrees)');
title('Pitch');
xlim([0 t(end)])