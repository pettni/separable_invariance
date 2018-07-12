%
%  Code for example V.3 in CDC 2015 paper of connected inverted pendulums
%
%   

clear all;

% parameters
num = 5;
k = 3; %spring constant
c = 3; %damper constant
tau = 0.1;

% number of zonotope generators
num_ineq = 6;

% max state
maxX = 1;

% max input 
maxU = 10;

% LMI tolerance
tolerance = 1e-6;

% number of iterations
maxiter = 6;

% matrix of allowed feedbacks (tridiag)
feedback = eye(num) + diag(ones(num-1, 1), 1) + diag(ones(num-1, 1), -1);

% maximize volume (0 = no, 1 = yes)
obj = 1;

% options for solver
solversettings = sdpsettings('solver', 'mosek', 'verbose', 2);

%%%%%%%%%%%%%%%%%%%%%%%
%%%% Set up system %%%%
%%%%%%%%%%%%%%%%%%%%%%%

sepdyn = SepDyn(2*ones(1,num), ones(1, num), zeros(1, num));

A_ii_edge = tau*[0 1; -k -c] + eye(2);
A_ii_mid= tau*[0 1; -2*k -2*c] + eye(2);
A_ij = tau*[0 0; k c];
B_i = tau*[0; 1];

for i=1:num
	sepdyn.setAij(i,i,A_ii_mid);
	if (i-1 >= 1)
		sepdyn.setAij(i,i-1,A_ij);
	end
	if (i+1 <= num)
		sepdyn.setAij(i,i+1,A_ij);
	end
	sepdyn.setBi(i,B_i);
end
sepdyn.setAij(1,1,A_ii_edge);
sepdyn.setAij(num,num,A_ii_edge);

% input, state constraints, disturbance
for i=1:num
	sepdyn.setUi(i,[1; -1], maxU * ones(2,1));
	sepdyn.setSi(i,[eye(2); -eye(2)], maxX * ones(4,1));
end

% generators
for i=1:num
	sepdyn.setZ_2d(i, num_ineq);
end

%%%%%%%%%%%%%%%%%%%%%%%%
% Set up and solve LMI %
%%%%%%%%%%%%%%%%%%%%%%%%

tstart = tic;
[Hx, K] = solvelmi(sepdyn, feedback, obj, maxiter, tolerance, solversettings);
toc(tstart)

%%%%%%%%%%%%%%%%%%%%%%%%
%%% Check solution %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%

check_sol(sepdyn, Hx, K);

%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Plot solution %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%

plot_sol(sepdyn, Hx, K);