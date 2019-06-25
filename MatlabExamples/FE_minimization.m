%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Free-energy minimisation example
%
% Simple example of free-energy minimisation for a 2-DOF robot arm in s
% static case. This ie the equivalent of the state estimation step in
% classical control theory.
% 
% Author: Corrado Pezzato, TU Delft
% Last modified: 25.06.2019
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

%% Set-up variables for simulation
t = 0.3;        % [s] Simulation time
h = 0.001;      % [s] Integration step

% Planar 2DOF robot parameters
a1 = 1;         % [m] Length link 1 
a2 = 1;         % [m] Length link 1 

%% Set-up variables for perception
% Real state of the robot, fixed joint position, no control actions
q = [0.3 0.8]'; % [rad]

% Prior belief about the states of the robot arm
  mu_d = q;              % [rad] 
% mu_d = [0.4, 0.6]';    % [rad] 
                  
% Precision matrix for the prior belief
P_mu = eye(2);     
% Precision matrix for the proprioceptive sensory data
P_y = eye(2);

% Learning rate
k_mu = 20;        

%% Initialization
% Initialize the vector of beliefs abot the states
mu = zeros(2,t/h);

% Initialize vector for collecting the free-energy values
F = zeros(1,t/h-1);

% Random initial guess about the states of the robot, initial conditions
mu(:,1) = [1 1.2]';

%% Free-energy minimization using gradient descent

for i=1:t/h-1
    
    % Simulate noisy sensory input from encoders
    % Noise
    z = random('norm', 0, 0.001, length(q), 1);
    % Sensory input
    y_q = q + z;  
    
    % Free-energy computation
    F(i) =  1/2*((y_q-mu(:,i))'*P_y*(y_q-mu(:,i)) + (mu(:,i)-mu_d)'*P_mu*(mu(:,i)-mu_d));
 
    % Define free-energy gradient
    gradF = -((y_q-mu(:,i))'*P_y - (mu(:,i)-mu_d)'*P_mu);
    
    % State update using gradient descent on the free-energy
    mu(:,i+1) = mu(:,i)-k_mu*h*gradF';  
end 

%% Graphics

% Plot params
time_vec = 0:h:t-h;   
fontSize = 20;

% Indixes for the markers, downsample 
ind = 20;

set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaulttextinterpreter','latex');

figure('DefaultAxesFontSize',fontSize,'Position', [10 10 640*2 480])
    subplot(1,2,1)
    plot(time_vec,mu(1,:),'-k','linewidth',2)
    hold on
    plot(time_vec,mu(2,:),':k','linewidth',2)
    plot(time_vec, q(1)*ones(size(time_vec)),'-xk','MarkerIndices',1:ind:length(mu),'MarkerSize',9,'linewidth',2)
    plot(time_vec, q(2)*ones(size(time_vec)),'-*k','MarkerIndices',1:ind:length(mu),'MarkerSize',9,'linewidth',2)
    grid on
    grid minor
    legend('$\mu_1$','$\mu_2$','$q_1$','$q_2$')
    title('\textbf{State estimation}')
    xlabel ('Time $[s]$');
    ylabel ('Internal states $\mu\ [rad]$');

    subplot(1,2,2)
    semilogy(time_vec(1:end-1),F,'-k','linewidth',2)
    grid on
    grid minor
    legend('$\mathcal{F}$')
    title('\textbf{Free-energy}')
    xlabel ('Time $[s]$');
    ylabel ('Free-energy $[-]$');
