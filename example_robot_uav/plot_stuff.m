	figure(1)
	clf; hold on
	fixpoly(plot(X1_set, 'color', 'white'), 1);
		fixpoly(plot(G1_1, 'color', 'green', 'alpha', 0.3), 0);
		fixpoly(plot(G1_2, 'color', 'blue', 'alpha', 0.3), 0);
	plot(x1vec(1,:), x1vec(2,:), 'color', 'red')

	% matlab2tikz('traj1.tikz','interpretTickLabelsAsTex',true, ...
	% 		     'width','\figurewidth', 'height', '\figureheight', ...
	% 		     'parseStrings',false, 'showInfo', false, ...
	% 		    'extraAxisOptions', ...
	% 		    'xmajorgrids=false, ymajorgrids=false, axis x line=bottom, axis y line=left, every axis x label/.style={at={(current axis.south east)},anchor=west},  every axis y label/.style={at={(current axis.north west)},anchor=south}')

	figure(2)
	clf; hold on
	fixpoly(plot(X2_set, 'color', 'white'), 1);
	fixpoly(plot(G2_1, 'color', 'green', 'alpha', 0.3), 0);
	fixpoly(plot(G2_2, 'color', 'blue', 'alpha', 0.3), 0);

	plot(x2vec(1,:), x2vec(2,:), 'color', 'red')

	% matlab2tikz('traj2.tikz','interpretTickLabelsAsTex',true, ...
	% 		     'width','\figurewidth', 'height', '\figureheight', ...
	% 		     'parseStrings',false, 'showInfo', false, ...
	% 		    'extraAxisOptions', ...
	% 		    'xmajorgrids=false, ymajorgrids=false, axis x line=bottom, axis y line=left, every axis x label/.style={at={(current axis.south east)},anchor=west},  every axis y label/.style={at={(current axis.north west)},anchor=south}')


	figure(3)
	clf; hold on
	fixpoly(plot(X1_set, 'color', 'white'), 1);

	for i=1:length(G1_1_list)
		fixpoly(plot(G1_1_list{i}, 'color', 'green', 'alpha', 0.1), 0);
	end

	fixpoly(plot(G1_1, 'color', 'green', 'alpha', 1), 1);
	fixpoly(plot(G1_2, 'color', 'blue', 'alpha', 1), 1);

	% % matlab2tikz('reach1.tikz','interpretTickLabelsAsTex',true, ...
	% % 		     'width','\figurewidth', 'height', '\figureheight', ...
	% % 		     'parseStrings',false, 'showInfo', false, ...
	% % 		    'extraAxisOptions', ...
	% % 		    'xmajorgrids=false, ymajorgrids=false, axis x line=bottom, axis y line=left, every axis x label/.style={at={(current axis.south east)},anchor=west},  every axis y label/.style={at={(current axis.north west)},anchor=south}')


	figure(4)
	clf; hold on

	fixpoly(plot(X1_set, 'color', 'white'), 1);
	for i=1:length(G1_2_list)
		fixpoly(plot(G1_2_list{i}, 'color', 'blue', 'alpha', 0.1), 0);
	end

	fixpoly(plot(G1_1, 'color', 'green', 'alpha', 1, 'linewidth', 1), 1);
	fixpoly(plot(G1_2, 'color', 'blue', 'alpha', 1, 'linewidth', 1), 1);

	% matlab2tikz('reach2.tikz','interpretTickLabelsAsTex',true, ...
	% 		     'width','\figurewidth', 'height', '\figureheight', ...
	% 		     'parseStrings',false, 'showInfo', false, ...
	% 		    'extraAxisOptions', ...
	% 		    'xmajorgrids=false, ymajorgrids=false, axis x line=bottom, axis y line=left, every axis x label/.style={at={(current axis.south east)},anchor=west},  every axis y label/.style={at={(current axis.north west)},anchor=south}')


	figure(5)
	clf; hold on
	plot((1:tmax)/NN, x1vec(1,:), 'color', 'blue', 'linewidth', 1);
	plot((1:tmax)/NN, x2vec(1,:), 'color', 'red', 'linewidth', 1);
	legend('Robot', 'UAV')
	pp = patch([0 tmax tmax 0]/10, [0.2 0.2 0.4 0.4], [0 0.6 0]);
	set(pp, 'facealpha', 0.1);
	set(pp, 'linestyle', 'none');
	pp = patch([0 tmax tmax 0]/10, [-0.2 -0.2 -0.4 -0.4], [0 0.6 0]);
	set(pp, 'facealpha', 0.1);
	set(pp, 'linestyle', 'none');
	ylim([-.5 .5])
	xlabel('$t$')
	ylabel('$x_1, x_2$')

	matlab2tikz('robot_uav_time_plot.tikz','interpretTickLabelsAsTex',true, ...
			     'width','\figurewidth', 'height', '\figureheight', ...
			     'parseStrings',false, 'showInfo', false, ...
			    'extraAxisOptions', ...
			    'xmajorgrids=false, ymajorgrids=false, axis x line=bottom, axis y line=left, every axis x label/.style={at={(current axis.south east)},anchor=west},  every axis y label/.style={at={(current axis.north west)},anchor=south}')

