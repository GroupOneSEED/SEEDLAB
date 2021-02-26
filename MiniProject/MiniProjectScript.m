%%
% Name: Brett Shearer
% Course: EENG350
%
%% Title: Simulation of the motor (open loop)
%
%% Description:
% Mini Project, Section 4.6
%
% This script defines motor parameters, runs an open loop simulation 
% MiniProject.slx to determine the transfer function representing the
% motor.
% The simulation instructions and parameters were taken from Section 4.6
% of the MiniProjectHandout document. The transfer function was derived 
% from techniques shown in EENG307 Lecture 15, with the angular velocity
% estimates taken from an Arduino sketched also developed as part of
% Sectionn 4.6 in the MiniProject Handout Document.  
% 
%% Parameters of the system/simulation
% The motor block diagram variables are shown below.  The parameters 
% model an input voltage (0-255 representing 0-7.5 Volts from the battery pack.
% The 'Theta per volt' model in the simulink block diagram is the Vin value,
% a K-gain, an integrator, and a theta output. The corresponding transfer
% function values to mimic the motor are shown below as K and sig. 
% below as K and sig. 

clear all

Vin = 255; % input voltage variable
angVelPerVolt = 12/255; % angular velocity in radians (12 radians/second)


% Transfer function values tuned using techniques in Lecture 15 EENG307
K = 12/255;
sig = 22;

%% Simulink Block Diagram
% This portion of the script runs the simulink model for block diagram that
% compares step response of the motor with a tuned transfer function also 
% representing the motor. The simulation outputs are captured and stored as 
% an 'out' variable. The first figure is the velocity comparison and the 
% second figure is the final position comparison.

open_system('MiniProject');
out = sim('MiniProject');

%% Plot of the Results
% Plots show both velocity and position outputs from the block diagram.
% The first figure plots the Velocity output from the simulink model, and the
% second figure plots the position output. Both graphs are very
% close to the inputs. This indicates the transfer function is sufficiently
% close/accurate to represent the open loop transfer function of the motor.

%plots the Velocity output
figure(1)
plot(out.Velocity);
title('Velocity');

% plots the Position output from the simulink model
figure(2)
plot(out.Position);
title('Position');

legend('motor', 'transferFunc');

%% Interpretation of Results
% The transfer function generated from techniques listed and shown in
% EENG307 Lecture 15 generates nearly identical matches to the step response
% of the DC motor system's behavior. The open loop transfer function is a
% sufficiently accurate representation of the open-loop step response of
% the DC motor system. 