clear all

%
%  Code for example V.1 in CDC 2015 paper of interconnected rotational systems
%
%    x_i (t+1) = alpha * R(theta) x_i(t) + beta * \sum_{j != i} x_j(t) + u_i(t)
%   

% Parameters
theta = pi/4;
alpha = 0.8;
beta = 0.1;

% number of subsystems
num = 3;

% number of zonotope generators
num_ineq = 8;

% max state
maxX = 1;

% max input 
maxU = 0.65;

% max external disturbance
maxD = .4;

% LMI tolerance
tolerance = 1e-8;

% matrix of allowed feedbacks (eye = only own state)
feedback = eye(num);

% maximize volume (0 = no, 1 = yes)
obj = 1;

% options for solver
solversettings = sdpsettings('solver', 'mosek', 'verbose', 0);

%%%%%%%%%%%%%%%%%%%%%%%
%%%% Set up system %%%%
%%%%%%%%%%%%%%%%%%%%%%%

sepdyn = SepDyn(2*ones(1,num), 2*ones(1,num), 2*ones(1,num));

A_ii = alpha*[cos(theta) sin(theta); -sin(theta) cos(theta)];
A_ij = beta*[1 0; 0 1];
B_i = eye(2);
E_i = eye(2);

for i=1:num
	sepdyn.setAij(i,i,[A_ii]);
	sepdyn.setBi(i,B_i);
	sepdyn.setEi(i,E_i);
	for j=1:num
		if (abs(i-j) == 1)
			sepdyn.setAij(i,j,A_ij);
		end
	end
end

% input, state constraints, disturbance
for i=1:num
	sepdyn.setUi(i,[eye(2); -eye(2)], maxU * ones(4,1));
	sepdyn.setSi(i,[eye(2); -eye(2)], maxX * ones(4,1));
	sepdyn.setHd(i, (1/maxD)*eye(2));
end

% generators
for i=1:num
	sepdyn.setZ_auto(i, num_ineq);
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