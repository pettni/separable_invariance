function [P] = extract_sets(sd, Hx)
    % function_name: short description
    % ======================================================
    %
    % SYNTAX
    % ------
    %   P = extract_sets(sd, Hx)
    %
    % DESCRIPTION
    % -----------
    %  extract list P{} of decomposed sets for given Hx
    %
    % INPUT
    % -----
    %   sd    Separable dynamics description
    %           Class: SepDyn
    %   Hx    Block-diagonal matrix representing invariant sets
    %
    % OUTPUT
    % -----
    %   P     Cell of Polyhedron objects representing decomposed sets

	Hx_cell = mat2cell(Hx, sd.n_i, sd.n_i);
	P = cell(1,length(sd.n_i));
	for i=1:length(sd.n_i)
		P{i} = Polyhedron('A', [sd.Z_i{i} * Hx_cell{i,i}; - sd.Z_i{i} * Hx_cell{i,i}], 'b', ones(2*size(sd.Z_i{i}, 1), 1));
	end
end