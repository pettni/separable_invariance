function [Hx,K] = solve_lmi(sd, feedback, objective, maxiter, tol, sdp_opts)
    % solve_lmi: verify correctness of separable invariant set solution
    % ======================================================
    %
    % SYNTAX
    % ------
    %   [Hx, K] = check_sol(sd, feedback, objective, maxiter, tol, sdp_opts)
    %
    % DESCRIPTION
    % -----------
    %  Set up and solve LMIs that search for separable controlled invariant sets.
    %  In particular, for each subsystem i = 1, ..., d, look for sets X_i and feedback controllers K_ij s.t.
    %
    %    (i)    (A_ij + B_i K_ij) X_j + E_i D_i \subset X_i, for all i   (robust controlled invariance)
    %    (ii)   X_i \subset S_i, for all i   							 (state constraints)
    %    (iii)  K_ij X_j \subset U_i, for all i 						 (control constraints)
    %
    %  The solution is represented by a block matrix K representing feedback, and 
    %  a block diagonal matrix Hx, s.t. each block Hx_ii represents the set X_i as
    %  X_i = { x : -1 <= Hx_ii x <= 1  }.
    %
    % INPUT
    % -----
    %   sd    		Description of separable dynamics together with state, input, and disturbance constraints
    %           		Class: SepDyn
    %   feedback 	0/1 (d x d)-matrix representing local feedback availability. Default: eye(d)
    % 
    %   objective   If true, try to maximize set volume (might cause numerical problems). Default: 0.
    %
    %	maxiter     Total number of iterations to update the solution. Default: 1 (i.e. no iterations).
    %
    %   tol    		Tolerance used in LMI inequalities
    %
    %   sdp_opts	Settings used in call to Yalmip optimization package. See `help sdpsettings'

	if (nargin < 2 | isempty(feedback) )
		feedback = eye(sd.d); % own state feedback is default
	end

	if (nargin < 3 | isempty(objective) )
		objective = 0; % no obj fcn is default
	end

	if (nargin < 4 | isempty(maxiter) )
		maxiter = 1; % no iterations is default
	end

	if (nargin < 5 | isempty(tol) )
		tol = 1e-8;
	end

	if (nargin < 6)
		sdp_opts = [];
	end

	disp('setting up LMIs..')

	A = sd.A;
	B = sd.B;
	E = sd.E;

	Hd = sd.Hd;

	Z = sd.Z;
	
	Hs = sd.Hs;
	hs = sd.hs;
	Hu = sd.Hu;
	hu = sd.hu;

	Nx = size(Z, 1);
	Nu = size(Hu, 1);
	Ns = size(Hs, 1);
	Nd = size(Hd, 1);

	n = size(A, 1);
	m = size(B, 2);

	for iter = 1:maxiter

		if iter > 1
			disp(['starting iteration ', num2str(iter)])
			% save old variables
			Hx_inv_0 = double(Hx_inv);
			BlkLambda_0 = double(BlkLambda);
			Phi_inv_j_0 = {};
			for j=1:Nx
				Phi_inv_j_0{j} = double(Phi_inv_j{j});
			end
		end

		% Invariant set matrix Hx_inv
		Hx_inv_i = cell(1, sd.d);
		for i=1:sd.d
			Hx_inv_i{i} = sdpvar(sd.n_i(i), sd.n_i(i), 'full');
		end
		Hx_inv = blkdiag(Hx_inv_i{:});

		% feedback matrix Khat
		Khat = sdpvar(m, n, 'full');
		x_ind = 1;
		for i=1:length(sd.n_i)
			u_ind = 1;
			for j=1:length(sd.m_i)
				if (feedback(i,j))
					Khat(u_ind:u_ind+sd.m_i(i)-1, x_ind:x_ind+sd.n_i(i)-1) = sdpvar(sd.m_i(i), sd.n_i(i), 'full');
				else
					Khat(u_ind:u_ind+sd.m_i(i)-1, x_ind:x_ind+sd.n_i(i)-1) = zeros(sd.m_i(i), sd.n_i(i));
				end
				u_ind = u_ind + sd.m_i(i);
			end
			x_ind = x_ind + sd.n_i(i);
		end

		constraints = [];

		BlkLambda_cell = cell(1, sd.d);
		for i=1:sd.d
			BlkLambda_cell{i} = eye(sd.n_i(i)) * sdpvar(1,1);
		end
		BlkLambda = blkdiag(BlkLambda_cell{:});

		Phi_inv_j = {};

		% Invariance constraints
		for j=1:Nx

			ej = zeros(Nx,1);
			ej(j,1) = 1;

			% distinguish cases with / w/o disturbance
			if (sd.p > 0)
				stddim = 2*n;
				Dd_j = sdpvar(Nd, Nd, 'diagonal');
				constraints = [constraints, Dd_j >= tol*eye(Nd)];

				Hx_inv_block = blkdiag(Hx_inv, Hx_inv);
				BlkLambda_block = blkdiag(BlkLambda, BlkLambda);
				Zej_block = [Z'*ej; Z'*ej];
			else
				stddim = n;
				
				Hx_inv_block = Hx_inv;
				BlkLambda_block = BlkLambda;
				Zej_block = Z'*ej;
			end

			Dx_j = sdpvar(Nx, Nx, 'diagonal');
			Phi_inv_j{j} = sdpvar(stddim, stddim, 'symmetric');
			constraints = [constraints, Dx_j >= tol*eye(Nx)];

			if (iter == 1)
				% solve initial problem

				% Set up auxiliary SDP vars
				Gamma_j = sdpvar(stddim, stddim, 'symmetric');
				Xi_j = sdpvar(stddim, stddim, 'symmetric');
				Psi_j = sdpvar(stddim, stddim, 'full');
				Omega_1_j = sdpvar(stddim, stddim, 'full');
				Omega_2_j = sdpvar(stddim, stddim, 'full');


				% eq (13a)
				constraints = [constraints, [Gamma_j Psi_j; Psi_j' Xi_j] >= tol*eye(2 * stddim)];

				% eq (13b)
				syst_ind = sep_to_ind(sd.z_i, j); % index of subsystem pertaining to current inequality

				c11 = Xi_j - Phi_inv_j{j};
				c12 = Omega_1_j - Hx_inv_block;
				c13 = Omega_2_j - Hx_inv_block;
				c14 = Psi_j'*Zej_block;
				c22 = 2*BlkLambda_block - Gamma_j;
				c23 = BlkLambda_block + Omega_1_j' - Psi_j;
				c24 = zeros(stddim, 1);
				c33 = Omega_2_j + Omega_2_j' - Xi_j;
				c34 = zeros(stddim, 1);
				c44 = BlkLambda(sum(sd.n_i(1:syst_ind)), sum(sd.n_i(1:syst_ind))) - ones(1,Nx) * Dx_j * ones(Nx,1);
				if (sd.p > 0)
					c44 = c44 - ones(1,Nd) * Dd_j * ones(Nd,1);
				end

				constraints = [constraints, [c11 c12 c13 c14; 
											 c12' c22 c23 c24;
											 c13' c23' c33 c34;
											 c14' c24' c34' c44] >= tol*eye(3*stddim+1)];
			else
				% improve solution
				c22 = 1 - ones(1,Nx) * Dx_j * ones(Nx,1);
				if (sd.p > 0)
					c11 = -blkdiag(Hx_inv_0, Hx_inv_0)'*inv(Phi_inv_j_0{j})'*Phi_inv_j{j}*inv(Phi_inv_j_0{j})*blkdiag(Hx_inv_0, Hx_inv_0) ...
						+ Hx_inv_block'*inv(Phi_inv_j_0{j})*blkdiag(Hx_inv_0, Hx_inv_0) ...
						+ blkdiag(Hx_inv_0, Hx_inv_0)'*inv(Phi_inv_j_0{j})'*Hx_inv_block;
					c22 = c22 - ones(1,Nd) * Dd_j * ones(Nd,1);
				else
					c11 = -Hx_inv_0'*inv(Phi_inv_j_0{j})'*Phi_inv_j{j}*inv(Phi_inv_j_0{j})*Hx_inv_0 ...
						+ Hx_inv'*inv(Phi_inv_j_0{j})*Hx_inv_0 ...
						+ Hx_inv_0'*inv(Phi_inv_j_0{j})'*Hx_inv;
				end
				c12 = Zej_block;
				constraints = [constraints, [c11 c12; c12' c22] >= tol * eye(stddim+1)];
			end

			% eq (19)
			if (sd.p > 0)
				c11 = blkdiag(Z'*Dx_j*Z, Dd_j);
				c12 = blkdiag(-0.5*( Hx_inv'*A' + Khat'*B' ), -0.5*inv(Hd)'*E');
				c22 = Phi_inv_j{j};
			else
				c11 = Z'*Dx_j*Z;
				c12 = -0.5*( Hx_inv'*A' + Khat'*B' );
				c22 = Phi_inv_j{j};
			end
			constraints = [constraints, [c11 c12; c12' c22] >= tol*eye(2*stddim) ];
		end

		% State constraints
		for j=1:Ns
			ej = zeros(Ns, 1);
			ej(j,1) = 1;
			
			Ds_j = sdpvar(Nx, Nx, 'diagonal');

			% pos def Ds_j
			constraints = [constraints, Ds_j >= tol*eye(Nx)];

			% eq (20)
			c11 = Z'*Ds_j*Z;
			c12 = -0.5*Hx_inv'*Hs'*ej;
			c22 = ej'*hs - ones(1,Nx)*Ds_j*ones(Nx,1);
			constraints = [constraints, [c11 c12; c12' c22] >= tol*eye(n+1)];
		end

		% control constraints
		for j=1:Nu
			ej = zeros(Nu, 1);
			ej(j,1) = 1;
			
			Du_j = sdpvar(Nx, Nx, 'diagonal');

			% pos def Du_j
			constraints = [constraints, Du_j >= tol*eye(Nx)];

			% eq (21)
			c11 = Z'*Du_j*Z;
			c12 = -0.5 * Khat' * Hu' * ej;
			c22 = ej'*hu - ones(1,Nx)*Du_j*ones(Nx,1);

			constraints = [constraints, [c11 c12; c12' c22] >= tol*eye(n + 1)];
		end

		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		obj_fcn = [];

		if objective
			% cost function
			lambda_hat = sdpvar(1,1);
			w_half = sdpvar(n, n, 'symmetric');
			constraints = [constraints, lambda_hat >= tol];
			c11 = lambda_hat * eye(n);
			c12 = lambda_hat * eye(n);
			c13 = zeros(n);
			c22 = Hx_inv + Hx_inv';
			c23 = w_half;
			c33 = lambda_hat * eye(n);
			constraints = [constraints, [c11 c12 c13; c12' c22 c23; c13' c23' c33] >= tol*eye(3*n)];
			obj_fcn = -logdet(w_half);
		end

		disp('Calling LMI solver')
		sol = optimize(constraints, obj_fcn, sdp_opts);
		disp('Optimization completed')

	end

	Hx = inv(double(Hx_inv));
	K = double(Khat) * Hx;
end