function [res] = plot_sol(sd, Hx, K)
    % plot_sol: plot separable invariant set solution
    % ======================================================
    %
    % SYNTAX
    % ------
    %   plot_sol(sd, Hx)
    %
    % DESCRIPTION
    % -----------
    %  Plots invariant sets together with possible successor states
    %
    % INPUT
    % -----
    %   sd    Separable dynamics description
    %           Class: SepDyn
    %   Hx 	  Block-diagonal matrix representing invariant sets

	Hx_cell = mat2cell(Hx, sd.n_i, sd.n_i);
	
	A_BK_cell = mat2cell(sd.A() + sd.B()*K, sd.n_i, sd.n_i);

	P = extract_sets(sd, Hx);

	for i=1:length(sd.n_i)
		figure(i); clf
		plot(P{i}, 'alpha', 0.5);
		hold on
		if (sd.p_i(i) > 0)
			Di = Polyhedron('A', [sd.H_d_i{i}; -sd.H_d_i{i}], 'b', ones(2*sd.p_i(i), 1));
			other_poly = sd.E_i{i} * Di;
		else
			other_poly = Polyhedron;
		end
		for j=1:length(sd.n_i)
			other_poly = other_poly + A_BK_cell{i,j}*P{j};
		end
		plot(other_poly, 'color', 'blue', 'alpha', 0.5)
	end
end