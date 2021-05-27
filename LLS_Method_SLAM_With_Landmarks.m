
clc, clear 
%reply = input('Please run the code section wise; press any number to continue: ');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Some parameters you can use to set up your solution of SimLab1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
length_hallway = 10;
sensor_range = 0.5;

landmarks = [2; 5; 8];
num_landmarks = length(landmarks);

stdev_odometry = 0.1; % m/s
stdev_range = 0.01; % m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot's "Ground Truth" Trajectory when it travels the hallway 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

delta_t = 0.1; % time is discretized with a time-step of 0.1 seconds
v_robot = 0.1; % robot travels at a constant speed of 0.1 m/s

t_terminal = length_hallway/v_robot; % time that we reach the end of hall

t_vector = 0:delta_t:t_terminal; % time vector for robot's trajectory
x_vector = 0:v_robot*delta_t:length_hallway; % robot's true position in time
v_vector = ones(1,length(t_vector)).*v_robot;  % robot's true velocity in time

num_states = length(x_vector); % number of discrete-time states in trajectory

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gausian error in Odometry
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mean = 0;   % mean
t = t_vector;

%figure, plot(Calculated_x)% your signal
%%
% Deliverable 1
clc
Actual_x = zeros(size(x_vector));
AE = zeros(1000,1001);
%AE = zeros(1,1000);
for N = 1:1000
    Actual_x(1,:) = 0; 
    noise = mean + stdev_odometry*randn(1,numel(x_vector));
    for i = 2:size(x_vector,2)
        Actual_x(i) = Actual_x(i-1) + (delta_t*v_robot) + noise(i);
    end
    
    AE(N,:) = abs(x_vector - Actual_x);
end

MAE = sum(AE,1)/N;
sprintf("deliverable 1 - Mean Absolute Error for %d Trials in meters is : ", N)
disp(MAE);
figure, plot(MAE,'LineWidth',2), xlim([0 1000])
xlabel('trials'), ylabel('distance   (m)')
title('Mean Absolute Error for 1000 Trials without Landmarks')

%%
% Deliverable 3
clc

X = 0;
dt = 0.1;
u = 0.1; %speed/control = 0.1m/s
w = 0;%0.01*randn(1,101); Q
v = 0.1*randn(1,1001); %R
v(:,1) =0;
%Kalman Filter variables

x_measured = [u; X];      %state vector
A = [1 0; 0.1 1];    %state transition matrix
P_Measured = [1 0; 0 0.0001];    %std_dev*std_dev = 0.1*0.1 
Q = [1*10^-8, 0; 0, 0];      %process noise covariance matrix 
R = 0.01;       %Sensor noise covariance matrix
H = 0:0.01:10;
H_k = H + v;

% Storing calculated values in these vectors for plotting
XX = zeros(1,1001);
tt = zeros(1,1001);
xx_predicted = zeros(2,1001);
xx = zeros(2,1001);
PP = zeros(2,2002);
yy = zeros(2,1001);
for t=1:1
   %simulating the System
    X = X + u*dt + w; % New Current State
    Z_K(1:2,:) = [0.1; H_k(t)]; % Measurements 
   % Z_K = H_k(t);
   %Prediction Step
    x_predicted = A*x_measured;         %predicting the new state (mean)
    P_Predicted = A * P_Measured * A' + Q;    %predicting the new uncertainity (covariance)
    
   %Update Step
    E = H_k(1,1:t)*P_Predicted*H_k(1,1:t)';     %Covariance of ^ expectation
    z = Z_K -  H_k(t)*x_predicted;      %innovation: diff between the expectation and real sensor measurement
    Z = R + E;      %Covariance of ^ - sum of uncertainities of expectation and real measurement
    K = P_Predicted*H_k(t)' * Z^-1;
    
    x_measured = x_predicted + K*z; %final corrected state
    P_Measured = P_Predicted - K * H_k(t)* P_Predicted;       %uncertainity in corrected state
    
   %Saving the outputs
   
    xx_predicted(:,t) = x_predicted;
    xx(:,t) = x_measured;
    PP(:,t*2+1:t*2+2) = P_Measured;
    XX(t) = X;
    tt(t) = t;
    yy(:,t*2+1) = Z_K;
end

figure
plot(tt,XX,tt,xx)
title(' 1D Kalman Filter')
legend('Ground Truth','Measured State')
xlabel('Time')
ylabel('Position')



