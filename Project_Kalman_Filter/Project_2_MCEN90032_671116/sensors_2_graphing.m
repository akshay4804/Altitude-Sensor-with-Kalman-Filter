close all
clear all
clc
% Reading all the data from the Excel sheet which had been collected and
% allocating parameters
sensors_2 = readtable('sensors_2_v3.xlsx','Sheet','Actual');
time_step = table2array(sensors_2(:,1));
accelerometer_acc = table2array(sensors_2(:,3));
accelerometer_vel = table2array(sensors_2(:,4));
accelerometer_height = table2array(sensors_2(:,5));

pressure_height = table2array(sensors_2(:,7));
pressure_vel = table2array(sensors_2(:,8));
pressure_acc = table2array(sensors_2(:,9));

%% Part 1

plot(time_step, accelerometer_acc,'b');
title('Accelerometer Acceleration Vs. Time Step')
xlabel('Time Step (seconds)') 
ylabel('Normalised acceleration (m/s^2)') 

figure
plot(time_step, accelerometer_vel,'b');
title('Accelerometer velocity Vs. Time Step ')
xlabel('Time Step (seconds)') 
ylabel('Accelerometer velocity (integrated acceleration)') 

figure
plot(time_step, accelerometer_height,'b');
title('Accelerometer height Vs. Time Step ')
xlabel('Time Step (seconds)') 
ylabel('Accelerometer height (integrated twice for acceleration') 

figure
plot(time_step, pressure_height,'r');
title('Pressure sensor height Vs. Time Step')
xlabel('Time Step (seconds)') 
ylabel('Pressure sensor velocity (differentiated height)') 

figure
plot(time_step, pressure_vel,'r');
title('Pressure sensor velocity Vs. Time Step')
xlabel('Time Step (seconds)') 
ylabel('Pressure sensor acceleration (double differentiated height)')  

figure
plot(time_step, pressure_acc,'r');
title('Pressure sensor acceleration Vs. Time Step')
xlabel('Time Step (seconds)') 
ylabel('Pressure sensor height') 
%% Part 2


% Establishing an initial x estimate for the Kalman filter
x_k = [0;0];

% Establishing an initial error covariance. This has been set out to be 1
% since the system is non-ideal and has a lot of noise. 
P = [1 1; 1 1];     

T = 0.2;        % Sampling time for the arduino
A = [1 T; 0 1]; % The calculated A matrix
B = [(T^2)/2;T];% The calculated B matrix
C = [1 0];      % The covariance matrix selected

% The variance of the acceleration of data
variance_acc_actual = 5.44678E-05;
% The variance of pressure sensor data
variance_p = 0.000327537;

% sensors_1 = readtable('sensors_2_v3.xlsx','Sheet','Idling');
% time_step_idling = table2array(sensors_2(:,1));
% accelerometer_acc = table2array(sensors_2(:,3));

% The noise matrices for both the accelerometer and pressure sensor 
Q = 0.005*B*B'*variance_acc_actual; 
R = variance_p*variance_p; 

figure
h = animatedline;
v = animatedline;

axis([0, 35,-1, 3])
hold on
% The following loop is the actual Kalman filter being implemented, with
% their 2 major steps: The Time Update and Measurement Update. 

for i = 1:length(accelerometer_acc)-1
% The Time Update Step (prediction process) 
    % Project the state ahead of time
    x_bar = A*x_k+B*accelerometer_acc(i);
    % Project the error covariance ahead of time
    p_avg = A*P*A'+Q;% Include noise
% The Measurement Update 
    % The Kalman Gain
    K = p_avg*C'*inv((C*p_avg*C'+R));  
    % The estimate in the x direction
    x_k = x_bar + K*(pressure_height(i) - C*x_bar);
    % The error covariance
    P = (eye(2) - K*C)*p_avg;
    % Saving the terms to specific arrays
    speed(i) = x_k(2);
    height(i) = x_k(1);
    addpoints(h,time_step(i),x_k(1));
    drawnow
    
        
  
end
speed(i+1) = 0;
height(i+1) = 1.5;
hold on
% plotting for Kalman purposes
plot(time_step, accelerometer_acc);
plot(time_step, pressure_height);
xlabel('Time (seconds)')
title('Kalman Height, Pressure sensor height and accelerometer acceleration');
legend('Kalman Height','Accelerometer acceleration', 'Pressure height')


% Plotting for height comparisons
figure
plot(time_step, height');
hold on
plot(time_step, accelerometer_height);
plot(time_step, pressure_height)
title('Comparison between heights for Kalman, Accelerometer and Pressure');
xlabel('Time (seconds)')
ylabel('Height (m)');
legend('Kalman Height','Accelerometer Height', 'Pressure Height')


% Plotting for velocity comparisons
figure
plot(time_step, speed');
hold on
plot(time_step, accelerometer_vel);
plot(time_step, pressure_vel)
title('Comparison between velocities for Kalman, Accelerometer and Pressure');
xlabel('Time (seconds)')
ylabel('Velocity (m/s)');
legend('Kalman Velocity','Accelerometer Vel', 'Pressure Vel')




