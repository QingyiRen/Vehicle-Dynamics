% B.Shyrokau
% Template for homework assignment #2
% RO47017 Vehicle Dynamics & Control
% Use and distribution of this material outside the RO47017 course 
% only with the permission of the course coordinator
clc; clear; clear mex;

%% Non-tunable parameters
par.g = 9.81;
par.Vinit   = 50 /3.6;              % initialization velocity, Don't TUNE
% Vehicle/Body (Camry)
par.mass     = 1380;                % vehicle mass, kg      
par.Izz      = 2634.5;              % body inertia around z-axis, kgm^2
par.L        = 2.79;                % wheelbase, m
par.l_f      = 1.384;               % distance from front axle to CoG, m
par.l_r      = par.L - par.l_f;     % distance from rear axle to CoG, m
% Steering
par.i_steer  = 15.4;                % steering ratio, -
% Additional
par.m_f      = par.mass * par.l_r / par.L;      % front sprung mass, kg
par.m_r      = par.mass * par.l_f / par.L;      % rear sprung mass, kg
%par.mu       = 0.5;                   % friction coefficient, -
%par.mu       = 1;                   % friction coefficient, -
par.mu       = 0.1;   
%% Tunable parameters
% Reference Generator
par.Calpha_front = 120000;          % front axle cornering stiffness
par.Calpha_rear  = 190000;          % rear axle cornering stiffness
par.Kus = par.m_f/par.Calpha_front - par.m_r/par.Calpha_rear; % understeer gradient
% second order TF identified from Sine Swept Test
par.wn      = 11;                   % yaw rate frequency
par.kseta   = 0.7;                  % yaw rate damping
par.tau     = 0.09;                 % yaw rate time constant

%% Add/ Change after this line
% Maneuver settings
%V_ref = 100 /3.6;                % pre-maneuver speed, km/h
%V_ref = 60 /3.6;                % pre-maneuver speed, km/h
V_ref = 60/3.6;                % pre-maneuver speed, km/h
% max_error=max(error_yaw_rate);

%% LQR CONTROLLER DESIGN

B=1/par.Izz;
% Q=530000000;
% R=0.01;
Q=10000000000;
R=0.05;
u_in=1:28;
K_lp=zeros(size(u_in));
for i=1:length(u_in)
    A=-(par.l_f^2*par.Calpha_front+par.l_r^2*par.Calpha_rear)/par.Izz/i;
    [K,S,P] = lqr(A,B,Q,R);
    K_lp(i)=K;
end

%%

% load speed60.mat
% 
% u1 = train_input_reference.Data;
% u2 = train_input_error.Data;
% u3 = train_input_velocity.Data;
% u4 = par.mu.*ones(length(u2),1);
% y60 = (train_output.Data)';
% 
% u60 = [u1 u2 u3 u4]';
% 
% %%
% 
% %load speed100.mat
% 
% u1 = train_input_reference.Data;
% u2 = train_input_error.Data;
% u3 = train_input_velocity.Data;
% u4 = par.mu.*ones(length(u2),1);
% y100 = (train_output.Data)';
% 
% u100 = [u1 u2 u3 u4]';
% 
% 
% %%
% 
% u = [u60 u100];
% y = [y60 y100];
% 
% net = feedforwardnet([3,5,1]);
% net = train(net,u,y);
% 
% %%
% 
% y_pred = net(u);
% perf = perform(net,y,y_pred);
% plot(1:size(y,2),[y;y_pred]);
% 
% %%
% 
% gensim(net)
