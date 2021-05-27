% This Program is written to solve 1D SLAM Problem using Linear
% Least Square Approach. The Problem is designed in such a way that
% the robot travels in the hall way until it reaches the end. It is 
% certain about it's position when it starts (i.e. at position 0 when
% it starts)

%%
clc, clear 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters of the Environment and Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Environment
length_hallway = 10;
landmarks = [2; 5; 8]; %Position of Landmarks
num_landmarks = length(landmarks);

% Robot
sensor_range = 0.5;
stdev_odometry = 0.1; % m/s
stdev_range = 0.01; % m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot's "Ground Truth" Trajectory when it travels the hallway 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

delta_t = 0.1; % time is discretized with a time-step of 0.1 seconds
v_robot = 0.1; % robot travels at a constant velocity of 0.1 m/s

t_terminal = length_hallway/v_robot; % time to reach the end of hall

t_vector = 0:delta_t:t_terminal; % time vector for robot's trajectory
x_vector = 0:v_robot*delta_t:length_hallway; % robot's true position in time
v_vector = ones(1,length(t_vector)).*v_robot; % robot's true velocity in time

num_states = length(x_vector); % number of discrete-time states in trajectory

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%------------    1D SLAM Calculation     -----------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Actual_x = zeros(size(x_vector)); % Position Estimate with Sensor Measurement
AE = zeros(1000,1001); % Absolute Error Vector

for N = 1:1000
    Actual_x(1,:) = 0; % Starting Position of Robot
    noise = stdev_odometry*randn(1,numel(x_vector)); % Zero Mean Gaussian 
                                                    % White Noise of Odometry Sensor 
    for i = 2:length(x_vector)
        Actual_x(i) = Actual_x(i-1) + (delta_t*v_robot) + noise(i);
    end    
    AE(N,:) = abs(x_vector - Actual_x);
end

MAE = sum(AE,1)/N;
Mean_A_Error = [];

for i = 1: length(x_vector)
     if mod(i,10) == 0
         Mean_A_Error = cat(2,Mean_A_Error,MAE(i));
     end
end

figure, xlabel('time   (s)'), ylabel('distance   (m)')
title('Mean Absolute Error for 1000 Trials without Landmarks')
plot(Mean_A_Error,'LineWidth', 2), xlim([0 100])




















