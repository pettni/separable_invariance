%% inoput_constr: function description
function [P] = input_constr(x0,X_final,A,B,E,U_set,D_set)

	n = size(A,2);
	m = size(B,2);
	p = size(E,2);

	mat = [U_set.A];
	vec = [U_set.b];

	for i = 1:size(D_set.V,1)
		di = D_set.V(i,:)';
		mat = [mat; [X_final.A*B]];
		vec = [vec; [X_final.b-X_final.A*E*di-X_final.A*A*x0]];
	end

	P = Polyhedron('A', mat, 'b', vec);
	P.minHRep();
