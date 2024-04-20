%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Autor: Anhelina Kyrylova  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% MPC controller using the GPC settings
Kp=1.5;
T1=1.5;
G = Kp * tf(1, [T1 1])

% Discretize the plant with sample time
Ts = 0.3;
Gd = c2d(G, Ts)

% Create a GPC settings structure.
GPCoptions = gpc2mpc;

% Hu
GPCoptions.NU = 2; 
% Hp
GPCoptions.N2 = 1;
GPCoptions.N2 = 3; 
% R
GPCoptions.Lam = 0.5; 
GPCoptions.T = [1 -0.8];

% Convert GPC to an MPC controller.
mpc = gpc2mpc(Gd, GPCoptions);

% Simulate for 50 steps with unmeasured disturbance between 
% steps 26 and 28, and reference signal of 0. 
SimOptions = mpcsimopt(mpc)
SimOptions.UnmeasuredDisturbance = [zeros(25,1);-0.1*ones(3,1); 0];
sim(mpc, 50, 0, SimOptions);