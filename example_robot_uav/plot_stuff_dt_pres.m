	figure(1)
	clf; hold on
	fixpoly(plot(X1_set, 'color', 'white'), 1);
	fixpoly(plot(G1_1, 'color', 'green', 'alpha', 0.3), 0);
	fixpoly(plot(G1_2, 'color', 'blue', 'alpha', 0.3), 0);
	xlabel('$x^1$')
	ylabel('$v_x^1$')
	matlab2tikz('goal1.tikz','interpretTickLabelsAsTex',true, ...
			     'width','\figurewidth', 'height', '\figureheight', ...
			     'parseStrings',false, 'showInfo', false, ...
			    'extraAxisOptions', ...
			    'xmajorgrids=false, ymajorgrids=false, axis x line=bottom, axis y line=left, every axis x label/.style={at={(current axis.south east)},anchor=west},  every axis y label/.style={at={(current axis.north west)},anchor=south}')

	figure(2)
	clf; hold on
	fixpoly(plot(X2_set, 'color', 'white'), 1);
	fixpoly(plot(G2_1, 'color', 'green', 'alpha', 0.3), 0);
	fixpoly(plot(G2_2, 'color', 'blue', 'alpha', 0.3), 0);
	xlabel('$x^2$')
	ylabel('$v_x^2$')
	matlab2tikz('goal2.tikz','interpretTickLabelsAsTex',true, ...
			     'width','\figurewidth', 'height', '\figureheight', ...
			     'parseStrings',false, 'showInfo', false, ...
			    'extraAxisOptions', ...
			    'xmajorgrids=false, ymajorgrids=false, axis x line=bottom, axis y line=left, every axis x label/.style={at={(current axis.south east)},anchor=west},  every axis y label/.style={at={(current axis.north west)},anchor=south}')


	figure(3)
	clf; hold on
	fixpoly(plot(X1_set, 'color', 'white'), 1);

	for i=1:length(G1_1_list)
		fixpoly(plot(G1_1_list{i}, 'color', 'green', 'alpha', 0.1), 0);
	end

	fixpoly(plot(G1_1, 'color', 'green', 'alpha', 1), 1);
	fixpoly(plot(G1_2, 'color', 'blue', 'alpha', 1), 1);
	xlabel('$x^1$')
	ylabel('$v_x^1$')
	matlab2tikz('reach11.tikz','interpretTickLabelsAsTex',true, ...
			     'width','\figurewidth', 'height', '\figureheight', ...
			     'parseStrings',false, 'showInfo', false, ...
			    'extraAxisOptions', ...
			    'xmajorgrids=false, ymajorgrids=false, axis x line=bottom, axis y line=left, every axis x label/.style={at={(current axis.south east)},anchor=west},  every axis y label/.style={at={(current axis.north west)},anchor=south}')


	figure(4)
	clf; hold on

	fixpoly(plot(X1_set, 'color', 'white'), 1);
	for i=1:length(G1_2_list)
		fixpoly(plot(G1_2_list{i}, 'color', 'blue', 'alpha', 0.1), 0);
	end
	xlabel('$x^1$')
	ylabel('$v_x^1$')
	fixpoly(plot(G1_1, 'color', 'green', 'alpha', 1, 'linewidth', 1), 1);
	fixpoly(plot(G1_2, 'color', 'blue', 'alpha', 1, 'linewidth', 1), 1);

	matlab2tikz('reach12.tikz','interpretTickLabelsAsTex',true, ...
			     'width','\figurewidth', 'height', '\figureheight', ...
			     'parseStrings',false, 'showInfo', false, ...
			    'extraAxisOptions', ...
			    'xmajorgrids=false, ymajorgrids=false, axis x line=bottom, axis y line=left, every axis x label/.style={at={(current axis.south east)},anchor=west},  every axis y label/.style={at={(current axis.north west)},anchor=south}')

	figure(5)
	clf; hold on
	fixpoly(plot(X1_set, 'color', 'white'), 1);

	for i=1:length(G2_1_list)
		fixpoly(plot(G2_1_list{i}, 'color', 'green', 'alpha', 0.1), 0);
	end
	xlabel('$x^2$')
	ylabel('$v_x^2$')
	fixpoly(plot(G2_1, 'color', 'green', 'alpha', 1), 1);
	fixpoly(plot(G2_2, 'color', 'blue', 'alpha', 1), 1);

	matlab2tikz('reach21.tikz','interpretTickLabelsAsTex',true, ...
			     'width','\figurewidth', 'height', '\figureheight', ...
			     'parseStrings',false, 'showInfo', false, ...
			    'extraAxisOptions', ...
			    'xmajorgrids=false, ymajorgrids=false, axis x line=bottom, axis y line=left, every axis x label/.style={at={(current axis.south east)},anchor=west},  every axis y label/.style={at={(current axis.north west)},anchor=south}')


	figure(6)
	clf; hold on

	fixpoly(plot(X1_set, 'color', 'white'), 1);
	for i=1:length(G2_2_list)
		fixpoly(plot(G2_2_list{i}, 'color', 'blue', 'alpha', 0.1), 0);
	end
	xlabel('$x^2$')
	ylabel('$v_x^2$')
	fixpoly(plot(G2_1, 'color', 'green', 'alpha', 1, 'linewidth', 1), 1);
	fixpoly(plot(G2_2, 'color', 'blue', 'alpha', 1, 'linewidth', 1), 1);

	matlab2tikz('reach22.tikz','interpretTickLabelsAsTex',true, ...
			     'width','\figurewidth', 'height', '\figureheight', ...
			     'parseStrings',false, 'showInfo', false, ...
			    'extraAxisOptions', ...
			    'xmajorgrids=false, ymajorgrids=false, axis x line=bottom, axis y line=left, every axis x label/.style={at={(current axis.south east)},anchor=west},  every axis y label/.style={at={(current axis.north west)},anchor=south}')


	figure(7)
	clf; hold on
	plot((1:tmax)/NN, x1vec(1,:), 'color', 'blue', 'linewidth', 1);
	plot((1:tmax)/NN, x2vec(1,:), 'color', 'red', 'linewidth', 1);
	legend('Robot', 'UAV')
	pp = patch([0 tmax tmax 0]/10, [0.2 0.2 0.35 0.35], [0 0 0.6]);
	set(pp, 'facealpha', 0.1);
	set(pp, 'linestyle', 'none');
	pp = patch([0 tmax tmax 0]/10, [-0.2 -0.2 -0.35 -0.35], [0 0 0.6]);
	set(pp, 'facealpha', 0.1);
	set(pp, 'linestyle', 'none');
	pp = patch([0 tmax tmax 0]/10, [0.18 0.18 0.33 0.33], [0.6 0 0]);
	set(pp, 'facealpha', 0.1);
	set(pp, 'linestyle', 'none');
	pp = patch([0 tmax tmax 0]/10, [-0.05 -0.05 -0.18 -0.18], [0.6 0 0]);
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

