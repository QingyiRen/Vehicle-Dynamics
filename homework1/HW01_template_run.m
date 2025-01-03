% B.Shyrokau
% b.shyrokau@tudelft.nl
% Intelligent Vehicles & Cognitive Robotics
% Department of Cognitive Robotics
% Faculty of Mechanical, Maritime and Materials Engineering
% Delft University of Technology, The Netherlands

%% Cleaning
% clc;clear;close all;
%%%

%% wheel slip controller

% Vehicle parameters
par.mass = 450;          % quarter car mass, kg
par.Iw = 1.2;            % inertia of the wheel, kg*m2
par.Reff = 0.305;        % wheel effective radius, m
par.g = 9.81;           
par.Pres2Moment = 11.25; % convertion from brake pressure to brake torque
par.max_pressure = 160;  % max. brake pressure, bar

% Maneuver settings
par.V0 = 120/3.6;        % Initial speed, m/s
par.Vmin = 10 / 3.6;     % Minimal speed to stop simulation, m/s
par.friction = 0.6;      % Friction coefficient [-]

%Reference
par.kref=0.12;


