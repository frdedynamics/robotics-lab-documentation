%% Read Log Script
% Read sensor data from a text file and saves them into a .mat file in

%% Fresh workspace at first
clear all
close all

%% Import log file
% Specify name of input file that you saved using your smart phone.
input_filename = 'sensorLog_2pitch.txt';
output_filename = 'data.mat';
fid=fopen(input_filename);

% The next lines are in the format
% data_type time data_x data_y data_z
% where:
% data_type = { ATTITUDE, ACCEL, GYRO, MAGNETO, GPS }
% time = floating-point time [s]
% data_x, data_y, data_z = floating-point values of vector of above
% Note units depend on type

data = textscan(fid,'%f %s %f %f %f');  % Resulting format: cells
fclose(fid);

%% Extract out numerical entries of data into an array (used below)
data_matrix = cell2mat( data(:,[1,3,4,5]) );

%% Now scan through A{1} (a cell) looking for each of type. 
% We will implement Kalman filter only using accelerometer and gyroscope
% data. Therefore we check only two types.
test = strcmp(data{2},'ACC');  
test1 = find(test==true);  
accel_data = data_matrix(test1,:); 
accel_data_length = length(accel_data);


test = strcmp(data{2},'GYR');  
test1 = find(test==true); 
gyro_data = data_matrix(test1,:); 
gyro_data_length = length(gyro_data);

% accelerometer starts logging earlier than gyroscope. With the following
% line we crop the first elements of the accelerometer where the gyroscope
% has not started yet. By doing that we make sure the length of accel_data
% and gyro_data are equal.
accel_data = accel_data(length(accel_data)+1-min(length(accel_data),length(gyro_data)):end,:);

% Our mobile app logs the data in each 10ms. We set this value here and
% create our accelerometer and gyroscope timeseries
sample_time = 0.01; 
sim_time = (sample_time*(length(accel_data)-1)); 
t_data = 0:sample_time:sim_time; 
acc_ts = timeseries(accel_data(:,2:4), t_data, 'name', 'Accelerometer'); 
gyro_ts = timeseries(gyro_data(:,2:4), t_data, 'name', 'Gyroscope'); 

%% Save all to disk as .mat file to be able to use in simulink
save('Accelerometer','acc_ts','-v7.3')
save('Gyroscope','gyro_ts','-v7.3')

%% flush the memory after we are done.
clear data;
clear fid
clear output_filename
clear test
clear test1
disp([' Data from ' input_filename  'saved']);


