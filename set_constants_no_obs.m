function [tau, Gamma, Iterations_num, MaxTrials, ConHor_len, PreHor_len, center_barrier, Obstacle,mu,Q,R,run,scale,Umax,Umin] = set_constants_no_obs() 
tau = 0.05; Gamma = 0.95; % discount factor 
Iterations_num = 180; 
MaxTrials = 30; 
ConHor_len = 1; % Control Horizon length 
PreHor_len = 20; % Predictive Horizon length 
center_barrier = 2.5; 
Obstacle = [30;1]; % no collision
mu=0.02;
Q=1*eye(8);
R=0.5*eye(2);
run=1;
scale=50;
Umax{1}=[5;5];Umin{1}=[-5;-5]; % enlarge the control constraint to for collision avoidance
Umax{2}=[5;5];Umin{2}=[-5;-5];
end