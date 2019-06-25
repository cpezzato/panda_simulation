%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Active inference for robot control
%
% Simple example of active inference control for a 2DOF robot arm. The free
% -energy is minimised in a dynamoc case through beliefs and actions update
% The script makes use of generalised motions since the states are
% dynamically changing
% 
% Author: Corrado Pezzato, TU Delft
% Last modified: 25.06.2019
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc
 
%% Set-up variables for simulation
t = 6;              % [s] Simulation time
h = 0.001;          % [s] Integration step
actionsTime = 1;    % [s]
% Planar 2DOF robot parameters
a1 = 1;             % [m] Length link 1 
a2 = 1;             % [m] Length link 2 

%% Set-up variables for perception

% Initialize generative process (so the sensors' output)
q = zeros(2,t/h);   % [rad]
dq = zeros(2,t/h);  % [rad/s]
ddq = zeros(2,t/h); % [rad/s^2]

% Initial state of the robot
q(:,1) = [-pi/2, 0]';     

% Prior belief about the states of the robot arm, desired position
mu_d = [-0.2 0.3]';  

%% Tuning parameters

% Confidence in the prior belief about the states
P_mu0 = eye(2);
% Confidence in the prior belief about the derivative of the states
P_mu1 = eye(2);
% Confidence in the proprioceptive sensory data (position)
P_y0 = eye(2);
% Confidence in the proprioceptive sensory data (velocity)
P_y1 = eye(2);
 
% Learning rate for beliefs update
k_mu = 20;
% Learning rate for actions
k_a = 500;

%% Initialize vectors

% Initialize actions vector
u = zeros(2,t/h);
% Initial control action
u(:,1) = [0 0]';

% Initialize the vector of beliefs abot the states and its derivatives
mu = zeros(2,t/h);     
mu_p = zeros(2,t/h);
mu_pp = zeros(2,t/h);   
 
% Initial guess about the states of the robot, initial belief
mu(:,1) = q(:,1)+[+0.6 +0.2]';   % mu    position + random constant to appreciate the convergence
mu_p(:,1) = [0 0]';              % mu'   velocity
mu_pp(:,1) = [0 0]';             % mu''  acceleration

% Initialize vector for collecting the free-energy values
F = zeros(1,t/h-1);
 
% Initialize vectors for sensory input (these will be noisy)
y_q = zeros(2,t/h);   
y_dq = zeros(2,t/h);

%% Active Inference loop

for i=1:t/h-1
    
    %% Simulate noisy sensory input from encoders and tachometers
    z = random('norm', 0, 0.001, size(q,1), 1);  
    z_prime = random('norm', 0, 0.001, size(q,1), 1);
    y_q(:,i) = q(:,i) + z;
    y_dq(:,i) = dq(:,i) + z_prime; 
   
    %% Compute free-energy in generalised coordinates
    F(i) = 0.5*(y_q(:,i)-mu(:,i))'*P_y0*(y_q(:,i)-mu(:,i))+...              % Proprioceptive position for joint 1 and 2
         + 0.5*(y_dq(:,i)-mu_p(:,i))'*P_y1*(y_dq(:,i)-mu_p(:,i))+...        % Proprioceptive velocity for joint 1 and 2 
         + 0.5*(mu_p(:,i)+mu(:,i)-mu_d)'*P_mu0*(mu_p(:,i)+mu(:,i)-mu_d)+... % Model prediction errors mu_p      
         + 0.5*(mu_pp(:,i)+mu_p(:,i))'*P_mu1*(mu_pp(:,i)+mu_p(:,i));        % Model prediction errors mu_pp
  
    %% Belifs update
    % Support variables for beliefs update 
    mu_dot = mu_p(:,i) - k_mu*(-P_y0*(y_q(:,i)-mu(:,i)) + P_mu0*(mu_p(:,i)+mu(:,i)-mu_d));
    mu_dot_p = mu_pp(:,i) - k_mu*(-P_y1*(y_dq(:,i)-mu_p(:,i)) ...                              
                          +P_mu0*(mu_p(:,i)+mu(:,i)-mu_d) ...                                
                          +P_mu1*(mu_pp(:,i)+mu_p(:,i)));
    mu_dot_pp = - k_mu*(P_mu1)*(mu_pp(:,i)+mu_p(:,i));
      
    % State estimation
    mu(:,i+1) = mu(:,i) + h*mu_dot;             % Belief about the position
    mu_p(:,i+1) = mu_p(:,i) + h*mu_dot_p;       % Belief about motion of mu
    mu_pp(:,i+1) = mu_pp(:,i) + h*mu_dot_pp;    % Belief about motion of mu'

    %% Control actions
    if i > actionsTime/h
        % Active inference
        u(:,i+1) = u(:,i)-h*k_a*(P_y1*(y_dq(:,i)-mu_p(:,i)) + P_y0*(y_q(:,i)-mu(:,i)));
    else
        u(:,i+1) = [0,0]';  % Set control torques to zero during the initial part
    end
    
    %% Update sensory input according to the actions taken
    ddq(:,i) = RealrobotDynamics(q(1,i),q(2,i),dq(1,i),dq(2,i),u(1,i),u(2,i));
    dq(:,i+1) = dq(:,i)+h*ddq(:,i);
    q(:,i+1) = q(:,i)+h*dq(:,i);
    
end 
 
%% Graphics

% Plot parameters
time_vec = 0:h:t-h;   
fontSize = 20;
lineWidth = 4;

% Indixes for the markers, downsample 
ind = 400;

set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaulttextinterpreter','latex');

figure('DefaultAxesFontSize',fontSize,'Position', [10 10 640*2 480*2])
    subplot(2,2,1)
    plot(time_vec,mu(1,:),'-k','linewidth',2)
    ylim([q(1,1)-0.06 mu_d(1)+0.12])
    hold on
    plot(time_vec, q(1,:),':k','linewidth',2)
    plot([actionsTime actionsTime], ylim,'-.k','linewidth',2); 
    plot(time_vec, mu_d(1)*ones(size(time_vec)),'--k','MarkerIndices',1:ind:length(mu),'MarkerSize',9,'linewidth',2)
    grid on
    grid minor
    legend('$\mu_1$','$q_1$','$t_a$','$\mu_{d_1}$','Location','best')
    title('\textbf{State estimation for $q_1$}')
    xlabel ('Time $[s]$');
    ylabel ('Internal states $\mu\ [rad]$');

    subplot(2,2,2)
    plot(time_vec, mu(2,:),'-k','linewidth',2)
    ylim([q(2,1)-0.06 mu_d(2)+0.06])
    hold on
    plot(time_vec, q(2,:),':k','linewidth',2)
    plot([actionsTime actionsTime], ylim,'-.k','linewidth',2); 
    plot(time_vec, mu_d(2)*ones(size(time_vec)),'--k','MarkerIndices',1:ind:length(mu),'MarkerSize',9,'linewidth',2)    
    grid on
    grid minor
    legend('$\mu_2$','$q_2$','$t_a$','$\mu_{d_2}$','Location','best')
    title('\textbf{State estimation for $q_2$}')
    xlabel ('Time $[s]$');
    ylabel ('Internal states $\mu\ [rad]$');
    
    subplot(2,2,3)
    plot(time_vec,u(1,:),'-k','linewidth',2)
    ylim([min(min(u)) max(max(u))+10])
    hold on
    plot(time_vec,u(2,:),':k','linewidth',2)
    plot([actionsTime actionsTime], ylim,'-.k','linewidth',2); 
    grid on
    grid minor
    legend('$u_1$','$u_2$','$t_a$','Location','best')
    title('\textbf{Control actions}')
    xlabel ('Time $[s]$');
    ylabel ('Torques $ u\ [rad]$');
    
    subplot(2,2,4)
    plot(time_vec(1:end-1),F,'-k','linewidth',2)
    hold on
    ylim([min(F) max(F)+0.06])
    plot([actionsTime actionsTime], ylim,'-.k','linewidth',1.5); 
    grid on
    grid minor
    legend('$\mathcal{F}$','$t_a$','Location','best')
    title('\textbf{Free-energy}')
    xlabel ('Time $[s]$');
    ylabel ('Free-energy $[-]$');

