%% sep_to_ind: given an array v of ints and a number c, return the first index i s.t. c <= sum(v(1:i+1))
%% if no such i exists, return 0
function [i] = sep_to_ind(v, c)
	sum = 0;
	for j=1:length(v)
		if sum + v(j) >= c
			i = j;
			return;
		end
		sum = sum + v(j);
	end
	i = 0;
