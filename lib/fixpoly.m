%% fixpoly: given a plot handle, remove 3d data and adjust linewidth
function [handle] = fixpoly(handle, linewidth)
	v3d = get(handle, 'Vertices');
	if size(v3d, 2) > 2
		v2d = v3d(:,1:2);
		set(handle, 'Vertices', v2d)
		if (linewidth)
			set(handle, 'linewidth', linewidth)
		else
			set(handle, 'linestyle', 'none')
		end
	end