function check_sol(sd, Hx, K)
    % check_sol: verify correctness of separable invariant set solution
    % ======================================================
    %
    % SYNTAX
    % ------
    %   check_sol(sd, Hx)
    %
    % DESCRIPTION
    % -----------
    %  Prints a message about solution correctness
    %
    % INPUT
    % -----
    %   sd    Separable dynamics description
    %           Class: SepDyn
    %   Hx 	  Block-diagonal matrix representing invariant sets

	A_BK_cell = mat2cell(sd.A() + sd.B()*K, sd.n_i, sd.n_i);

	K_cell = mat2cell(K, sd.m_i, sd.n_i);

	P = extract_sets(sd, Hx);

	res = [];
	margin = [];

	for i=1:length(sd.n_i)

		% check invariance
		if (sd.p_i(i) > 0)
			Di = Polyhedron('A', [sd.H_d_i{i}; -sd.H_d_i{i}], 'b', ones(2*sd.p_i(i), 1));
			worstcase = max(P{i}.A * sd.E_i{i} * Di.V', [], 2);
		else
			worstcase = zeros(size(P{i}.A, 1), 1);
		end

		for j = 1:length(sd.n_i)
			worstcase = worstcase + max(P{i}.A * A_BK_cell{i,j} * P{j}.V', [], 2);
		end
		if ~all(worstcase - P{i}.b <= 0 )
			res = [res 1];
			margin = [margin max(worstcase - P{i}.b)];
		end

		% check control constraints
		worstcase = zeros(size(sd.H_u_i{i}, 1), 1);
		for j = 1:length(sd.n_i)
			worstcase = worstcase + max(sd.H_u_i{i} * K_cell{i,j} * P{j}.V', [], 2);
		end
		if ~all(worstcase - sd.h_u_i{i} <= 0 )
			res = [res 2];
			margin = [margin max(worstcase - sd.h_u_i{i}) ];
		end 

		% check state constraints
		if ~all( max(sd.H_s_i{i} * P{i}.V', [], 2) - sd.h_s_i{i} <= 0 )
			res = [res 3];
			margin = [margin max(max(sd.H_s_i{i} * P{i}.V', [], 2) - sd.h_s_i{i} )];
		end
	end

	if size(res)
		disp(['check_sol: solution has error messages ', num2str(res), ' with margins ', num2str(margin)]);
	else
		disp('check_sol: solution is correct');
	end

end