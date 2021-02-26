
%%
% Name: Brett Shearer
% Course: EENG350
%
%% Title: Controller Design and Tune (closed loop)
%
%% Description:
% Mini Project, Section 4.7
%
% This script uses the transfer function derived from Section 4.6 to
% represent the behavior of the Motor system and then simulates a PI
% controller. The simulation instructions and parameters were taken from Section 4.7
% of the MiniProjectHandout document. The controller P and I values were
% developed by simulating the block diagram and using the 'tune' operation
% to reduce overshoot to less than 12% and rise time to < 1 second, per the
% instructions in the MiniProject handout document.
%
% 
%% Parameters of the system/simulation
% The motor block diagram variables are shown below.  The parameters 
% model an input voltage (0-255 representing 0-7.5 Volts from the battery pack.
% The 'Theta per volt' model in the simulink block diagram is the Vin value,
% a K-gain, an integrator, and a theta output. The corresponding transfer
% function values to mimic the motor are shown below as K and sig. 
% below as K and sig. 

% Transfer function values for the motor tuned using techniques in Lecture 15 EENG307
% and Section 4.6 of the MiniProjectHandout documentation

K = 12/255; % Gain value 
sig = 22; 

%% Simulink Block Diagram
% This portion of the script runs the simulink model for the closed loop
% step response with the PI Controller. The simulation's position output is 
% captured and stored as an 'out' variable. 

open_system('MiniProject2');
out = sim('MiniProject2');

%% Plot of the Results
% The output plot shows the position outputs from the closed-loop block diagram.
% The tuned PI controller values (P = 241 and I = 40, approximately)
% resulted in about 5% overshoot and a slightly less than 1 second rise
% time. These parameters/results could change with different P and I values
% depending on the intended purpose or restrictions for the system.

plot(out.position);
title('Position');

%% Interpretation of Results
% The PI controller simulated results show a step response within the
% MiniProjectHandout document parameters. We intentionally increased overshoot
% to 5%-6% as tradeoff with a slightly longer step response because smaller overshoots
% showed less accurate results in experimentation.
