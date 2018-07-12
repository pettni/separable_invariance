%
%  Messy file used to solve synthesis problem inside invariant sets
%

P = extract_sets(sepdyn, Hx);

%%% system 1
X1_set = P{1};
D1_set = P{2};
U1_set = Polyhedron('A', sepdyn.H_u_i{1}, 'b', sepdyn.h_u_i{1});

A1 = sepdyn.A_ij{1,1};
E1 = sepdyn.A_ij{1,2};
B1 = sepdyn.B_i{1};

G1_1 = Polyhedron('V', [-0.3 0.2]);
G1_1 = G1_1 + 0.1*Polyhedron('A', [eye(2); -eye(2)], 'b', ones(4,1));
G1_1_list = {G1_1};
for i=1:5
	G1_1_list{end+1} = intersect(X1_set, pre(A1,B1,E1,G1_1_list{end},U1_set,D1_set));
end

G1_2 = Polyhedron('V', [0.3 -0.2]);
G1_2 = G1_2 + 0.1*Polyhedron('A', [eye(2); -eye(2)], 'b', ones(4,1));
G1_2_list = {G1_2};
for i=1:5
	G1_2_list{end+1} = intersect(X1_set, pre(A1,B1,E1,G1_2_list{end},U1_set,D1_set));
end

%%% system 2

X2_set = P{2};
D2_set = P{1};
U2_set = Polyhedron('A', sepdyn.H_u_i{2}, 'b', sepdyn.h_u_i{2});

A2 = sepdyn.A_ij{2,2};
E2 = sepdyn.A_ij{2,1};
B2 = sepdyn.B_i{2};

G2_1 = Polyhedron('V', [-0.3 0.2]);
G2_1 = G2_1 + 0.1*Polyhedron('A', [eye(2); -eye(2)], 'b', ones(4,1));
G2_1_list = {G2_1};
for i=1:5
	G2_1_list{end+1} = intersect(X2_set, pre(A2,B2,E2,G2_1_list{end},U2_set,D2_set));
end
G2_2 = Polyhedron('V', [0.3 -0.2]);
G2_2 = G2_2 + 0.1*Polyhedron('A', [eye(2); -eye(2)], 'b', ones(4,1));
G2_2_list = {G2_2};
for i=1:5
	G2_2_list{end+1} = intersect(X2_set, pre(A2,B2,E2,G2_2_list{end},U2_set,D2_set));
end

% simulation
tmax = 100;

x1 = -0.1+0.2*rand(2,1);
x2 = -0.1+0.2*rand(2,1);

x1vec = zeros(2, tmax);
x2vec = zeros(2, tmax);

x1_obj = 1;
x2_obj = 2;

for t=1:tmax

	x1vec(:,t) = x1;
	x2vec(:,t) = x2;
	% find goal set for x1
	if x1_obj == 1
		for i=1:length(G1_1_list)
			if G1_1_list{i}.contains(x1)
				break;
			end
		end
		if i == 1
			x1_obj = 2;
			goal_set1 = G1_2_list{end};
		else
			goal_set1 = G1_1_list{i-1};
		end
	else
		for i=1:length(G1_2_list)
			if G1_2_list{i}.contains(x1)
				break;
			end
		end
		if i == 1
			x1_obj = 1;
			goal_set1 = G1_1_list{end};
		else
			goal_set1 = G1_2_list{i-1};
		end
	end


	if x2_obj == 1
		for i=1:length(G2_1_list)
			if G2_1_list{i}.contains(x2)
				break;
			end
		end
		if i == 1
			x2_obj = 2;
			goal_set2 = G2_2_list{end};
		else
			goal_set2 = G2_1_list{i-1};
		end
	else
		for i=1:length(G2_2_list)
			if G2_2_list{i}.contains(x2)
				break;
			end
		end
		if i == 1
			x2_obj = 1;
			goal_set2 = G2_1_list{end};
		else
			goal_set2 = G2_2_list{i-1};
		end
	end

	% find controls that steer to goal set
	U1_constr = input_constr(x1,goal_set1,A1,B1,E1,U1_set,D1_set);
	minu = min(U1_constr.V);
	maxu = max(U1_constr.V);
	u1 = minu + rand(1)*(maxu - minu);

	U2_constr = input_constr(x2,goal_set2,A2,B2,E2,U2_set,D2_set);
	minu = min(U2_constr.V);
	maxu = max(U2_constr.V);
	u2 = minu + rand(1)*(maxu - minu);

	x1_new = A1 * x1 + B1 * u1 + E1 * x2;
	x2_new = A2 * x2 + B2 * u2 + E2 * x1;

	x1 = x1_new;
	x2 = x2_new;

end

%%% Produce the plots that are in the paper
if 1
	figure(1)
	clf; hold on
	fixpoly(plot(X1_set, 'color', 'white'), 1);
		fixpoly(plot(G1_1, 'color', 'green', 'alpha', 0.3), 0);
		fixpoly(plot(G1_2, 'color', 'blue', 'alpha', 0.3), 0);
	plot(x1vec(1,:), x1vec(2,:), 'color', 'red')

	matlab2tikz('traj1.tikz','interpretTickLabelsAsTex',true, ...
			     'width','\figurewidth', 'height', '\figureheight', ...
			     'parseStrings',false, 'showInfo', false, ...
			    'extraAxisOptions', ...
			    'xmajorgrids=false, ymajorgrids=false, axis x line=bottom, axis y line=left, every axis x label/.style={at={(current axis.south east)},anchor=west},  every axis y label/.style={at={(current axis.north west)},anchor=south}')


	figure(2)
	clf; hold on
	fixpoly(plot(X2_set, 'color', 'white'), 1);
		fixpoly(plot(G2_1, 'color', 'green', 'alpha', 0.3), 0);
		fixpoly(plot(G2_2, 'color', 'blue', 'alpha', 0.3), 0);

	plot(x2vec(1,:), x2vec(2,:), 'color', 'red')

	matlab2tikz('traj2.tikz','interpretTickLabelsAsTex',true, ...
			     'width','\figurewidth', 'height', '\figureheight', ...
			     'parseStrings',false, 'showInfo', false, ...
			    'extraAxisOptions', ...
			    'xmajorgrids=false, ymajorgrids=false, axis x line=bottom, axis y line=left, every axis x label/.style={at={(current axis.south east)},anchor=west},  every axis y label/.style={at={(current axis.north west)},anchor=south}')


	figure(3)
	clf; hold on
	fixpoly(plot(X1_set, 'color', 'white'), 1);

	for i=1:6
		fixpoly(plot(G1_1_list{i}, 'color', 'green', 'alpha', 0.3), 0);
	end

	fixpoly(plot(G1_1, 'color', 'green', 'alpha', 1), 1);
	fixpoly(plot(G1_2, 'color', 'blue', 'alpha', 1), 1);

	matlab2tikz('reach1.tikz','interpretTickLabelsAsTex',true, ...
			     'width','\figurewidth', 'height', '\figureheight', ...
			     'parseStrings',false, 'showInfo', false, ...
			    'extraAxisOptions', ...
			    'xmajorgrids=false, ymajorgrids=false, axis x line=bottom, axis y line=left, every axis x label/.style={at={(current axis.south east)},anchor=west},  every axis y label/.style={at={(current axis.north west)},anchor=south}')


	figure(4)
	clf; hold on

	fixpoly(plot(X1_set, 'color', 'white'), 1);
	for i=1:6
		fixpoly(plot(G1_2_list{i}, 'color', 'blue', 'alpha', 0.3), 0);
	end

	fixpoly(plot(G1_1, 'color', 'green', 'alpha', 1, 'linewidht', 1), 1);
	fixpoly(plot(G1_2, 'color', 'blue', 'alpha', 1, 'linewidth', 1), 1);

	matlab2tikz('reach2.tikz','interpretTickLabelsAsTex',true, ...
			     'width','\figurewidth', 'height', '\figureheight', ...
			     'parseStrings',false, 'showInfo', false, ...
			    'extraAxisOptions', ...
			    'xmajorgrids=false, ymajorgrids=false, axis x line=bottom, axis y line=left, every axis x label/.style={at={(current axis.south east)},anchor=west},  every axis y label/.style={at={(current axis.north west)},anchor=south}')
end