%% pre: function description
function [x_pre] = pre(A,B,E,X_set,U_set,D_set)

	n = size(A,2);
	m = size(B,2);
	p = size(E,2);

	mat = [];
	vec = [];

	for i = 1:size(D_set.V,1)
		di = D_set.V(i,:)';
		mat = [mat; [X_set.A*A X_set.A*B; zeros(size(U_set.A,1), n) U_set.A]];
		vec = [vec; [X_set.b-X_set.A*E*di; U_set.b]];
	end

	poly = Polyhedron('A', mat, 'b', vec);
	poly.minHRep();

	x_pre = projection(poly, 1:n);