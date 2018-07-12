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

G1_1 = intersect(X1_set, Polyhedron('H', [1 0 -0.2; -1 0 0.35]));

G1_1_list = {G1_1};
for i=1:6
	G1_1_list{end+1} = intersect(X1_set, pre(A1,B1,E1,G1_1_list{end},U1_set,D1_set));
end

G1_2 = intersect(X1_set, Polyhedron('H', [1 0 0.35; -1 0 -0.2]));

G1_2_list = {G1_2};
for i=1:6
	G1_2_list{end+1} = intersect(X1_set, pre(A1,B1,E1,G1_2_list{end},U1_set,D1_set));
end

%%% system 2

X2_set = P{2};
D2_set = P{1};
U2_set = Polyhedron('A', sepdyn.H_u_i{2}, 'b', sepdyn.h_u_i{2});

A2 = sepdyn.A_ij{2,2};
E2 = sepdyn.A_ij{2,1};
B2 = sepdyn.B_i{2};

G2_1 = intersect(X2_set, Polyhedron('H', [1 0 -0.05; -1 0 0.18]));
G2_1_list = {G2_1};
for i=1:5
	G2_1_list{end+1} = intersect(X2_set, pre(A2,B2,E2,G2_1_list{end},U2_set,D2_set));
end

G2_2 = intersect(X2_set, Polyhedron('H', [1 0 0.33; -1 0 -0.18]));
G2_2_list = {G2_2};
for i=1:5
	G2_2_list{end+1} = intersect(X2_set, pre(A2,B2,E2,G2_2_list{end},U2_set,D2_set));
end

% simulation
tmax = 1000;

x1 = -0.1+0.2*rand(2,1);
x2 = -0.1+0.2*rand(2,1);

x1vec = zeros(2, tmax);
x2vec = zeros(2, tmax);

x1_obj = 1;
x2_obj = 2;


NN = 10; % split one time segment into NN parts
A1_small = sepdyn.A_ij{1,1}^(1/NN);
A1_small_sum = eye(2);
for i=1:NN-1
	A1_small_sum = A1_small*A1_small_sum + eye(2);
end 
B1_small = A1_small_sum \ sepdyn.B_i{1};
E1_small = A1_small_sum \ sepdyn.A_ij{1,2};

A2_small = sepdyn.A_ij{2,2}^(1/NN);
A2_small_sum = eye(2);
for i=1:NN-1
	A2_small_sum = A2_small*A2_small_sum + eye(2);
end 
B2_small = A2_small_sum \ sepdyn.B_i{2};
E2_small = A2_small_sum \ sepdyn.A_ij{2,1};

new = 10;

for t=1:tmax

	x1vec(:,t) = x1;
	x2vec(:,t) = x2;

	if new >= 10

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

		new = 0;
	else
		new = new + 1;
	end

	x1_new = A1_small * x1 + B1_small * u1 + E1_small * x2;
	x2_new = A2_small * x2 + B2_small * u2 + E2_small * x1;

	x1 = x1_new;
	x2 = x2_new;

end

%%% Produce the plots that are in the paper
if 1
	plot_stuff_dt_pres
end