%
%  Code for example V.2 in CDC 2015 paper of connected UAV/robot
%
%    x_1 (t+1) = [1 1; *  x_1(t) + [0 0; *  x_2(t) + [0;   * u_1(t)
%                 k 1 ]           -k 0]               1]
%   

clear all;

% spring constant
k = 0.1; 

% number of zonotope generators
num_ineq = 5;

% max state
maxX = 1;

% max input 
maxU = 0.3;

% LMI tolerance
tolerance = 1e-12;

% matrix of allowed feedbacks (eye = only own state)
feedback = eye(2);

% maximize volume (0 = no, 1 = yes)
obj = 1;

% options for solver
solversettings = sdpsettings('solver', 'mosek', 'verbose', 2);

%%%%%%%%%%%%%%%%%%%%%%%
%%%% Set up system %%%%
%%%%%%%%%%%%%%%%%%%%%%%

sepdyn = SepDyn([2 2], [1 1], [0 0]);

A_ii = [1 1; k 1];
A_ij = [0 0; -k 0];

sepdyn.setAij(1,1,A_ii);
sepdyn.setAij(2,2,A_ii);
sepdyn.setAij(1,2,A_ij);
sepdyn.setAij(2,1,A_ij);

B_i = zeros(2,1);
B_i(2,1) = 1;
sepdyn.setBi(1,B_i);
sepdyn.setBi(2,B_i);

% input, state constraints, disturbance
for i=1:2
	sepdyn.setUi(i,[1; -1], maxU * ones(2,1));
	sepdyn.setSi(i,[eye(2); -eye(2)], maxX * ones(4,1));
end

% generators
for i=1:2
	sepdyn.setZ_2d(i, num_ineq);
end

%%%%%%%%%%%%%%%%%%%%%%%%
% Set up and solve LMI %
%%%%%%%%%%%%%%%%%%%%%%%%

tstart = tic;
[Hx, K] = solvelmi(sepdyn, feedback, obj, 1, tolerance, solversettings);
toc(tstart)

%%%%%%%%%%%%%%%%%%%%%%%%
%%% Check solution %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%

check_sol(sepdyn, Hx, K);

%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Plot solution %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%

plot_sol(sepdyn, Hx, K);