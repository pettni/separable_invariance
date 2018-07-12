classdef SepDyn < handle
	% SepDyn: Class to store a separable invariance problem specification
    % ======================================================
    %
    % SYNTAX
    % ------
    %   sd = SepDyn(n_i, m_i, p_i)
    %
    % DESCRIPTION
    % -----------
    %  Stores information about dynamics, state separation, constraints.
    %  All dynamics are initialized to 0.
    %
    % INPUT
    % -----
    %   n_i   Vector representing subsystem state dimensions. 
    %         I.e. n_i = [2 2] indicates two two-dimensional subsystems.
    %   m_i   Vector representing subsystem input dimensions. 
    % 		  I.e. m_i = [1 2] indicates that subsystem 1 has 1 input and subsystem 2 has 2.
    %   p_i   Vector representing subsystem disturbance dimensions.
    %
    % METHODS
    % -------
    %  ds.setAij(i,j,A)  set block (i,j) of system dynamics to A
    %  
    %  ds.setBi(i,B)  set the i'th input matrix to B
    %
    %  ds.setEi(i,e)  set the i'th disturbance matrix to E
    %
    %  ds.setUi(i,Hu, hu)  define input constraints for subsystem i as { u : Hu u <= hu}
    %
    %  ds.setSi(i,Hs, hs)  define state constraints for subsystem i as { x : Hs x <= hs}
    %
    %  ds.setHd(i,Hd)  define disturbance constraints for subsystem i as { d : -1 <= Hd x <= 1}
    %
    %  ds.setZ_auto(i, num)  randomly sample num unit length generators for subsystem i
    %
    %  ds.setZ_2d(i, num)  pick num evenly spaced unit length generators for a 2-dimensional subsystem i
    %
    %  The following methods return composed system description quantities:
    %
    %  ds.A, ds.B, ds.E, ds.Z, ds.Hd, ds.Hu, ds.hu, ds.Hs, ds.hs, ds.Hd, ds.n, ds.m, ds.p
    %

	properties (SetAccess = protected)
		% number of subsystems
		d;

		% Decomposed description
		A_ij;
		B_i;
		E_i;

		% dimensions
		n_i;
		m_i;
		p_i;
		z_i;

		% constraints
		H_u_i; % control
		h_u_i
		H_s_i; % state
		h_s_i;
		H_d_i; % disturbance

		% generators
		Z_i;
	end

	methods 
		function ds = SepDyn(n_i, m_i, p_i)
			if (length(n_i) ~= length(m_i) || length(n_i) ~= length(p_i))
				error('SepDyn: input must have same dimensions')
			end
			ds.n_i = n_i;
			ds.m_i = m_i;
			ds.p_i = p_i;
			ds.z_i = n_i;

			ds.d = length(n_i);

			ds.A_ij = cell(ds.d, ds.d);
			ds.B_i = cell(1,ds.d);
			ds.E_i = cell(1,ds.d);

			ds.H_u_i = cell(1,ds.d);
			ds.h_u_i = cell(1,ds.d);
			ds.H_s_i = cell(1,ds.d);
			ds.h_s_i = cell(1,ds.d);

			for i=1:ds.d
				for j=1:ds.d
					ds.A_ij{i,j} = zeros([n_i(i) n_i(j)]);
				end
				ds.B_i{i} = zeros([n_i(i) m_i(j)]);
				ds.E_i{i} = zeros([n_i(i) p_i(j)]);

				ds.H_u_i{i} = zeros(0, m_i(i));
				ds.h_u_i{i} = zeros(0, 0);
				ds.H_s_i{i} = zeros(0, n_i(i));
				ds.h_s_i{i} = zeros(0, 0);

				ds.H_d_i{i} = eye(p_i(i));

				ds.Z_i{i} = eye(n_i(i));
			end
		end

		function setAij(ds,i,j,A) 
			if (~all(size(A) == [ds.n_i(i) ds.n_i(i)]))
				error(['wrong dimensions in setAij, found ', num2str(size(A)), ' expected ', num2str([ds.n_i(i) ds.n_i(i)])]);
			end
			ds.A_ij{i,j} = A;
		end

		function setBi(ds,i,B)
			if (~all(size(B) == [ds.n_i(i) ds.m_i(i)]))
				error(['wrong dimensions in setBi, found ', num2str(size(B)), ' expected ', num2str([ds.n_i(i) ds.m_i(i)])]);
			end
			ds.B_i{i} = B;
		end

		function setEi(ds,i,E)
			if (~all(size(E) == [ds.n_i(i) ds.p_i(i)]))
				error(['wrong dimensions in setEi, found ', num2str(size(E)), ' expected ', num2str([ds.n_i(i) ds.p_i(i)])]);
			end
			ds.E_i{i} = E;
		end

		function setUi(ds,i,Hu, hu)
			if (size(Hu, 2) ~= ds.m_i(i) || size(Hu, 1) ~= size(hu,1))
				error(['wrong dimensions in setUi, found ', num2str(size(Hu, 2)), ' expected ', num2str(ds.m_i(i)) ]);
			end
			ds.H_u_i{i} = Hu;
			ds.h_u_i{i} = hu;
		end

		function setSi(ds, i, Hs, hs)
			if (size(Hs, 2) ~= ds.n_i(i) || size(Hs, 1) ~= size(hs,1) || size(hs, 2) ~= 1)
				error(['wrong dimensions in setSi, found ', num2str(size(Hs, 2)), ' expected ', num2str(ds.n_i(i)) ]);
			end
			ds.H_s_i{i} = Hs;
			ds.h_s_i{i} = hs;
		end

		function setHd(ds, i, Hd)
			if ~all(size(Hd, 2) == [ds.p_i(i) ds.p_i(i)] )
				error(['wrong dimensions in setSi, found ', num2str(size(Hd)), ' expected ', num2str([ ds.p_i(i) ds.p_i(i)]) ]);
			end
			ds.H_d_i{i} = Hd;
		end

		function setZ_auto(ds, i, num)
			points = randn(num, ds.n_i(i));
			n = sqrt(sum(points.*points, 2));
			ds.Z_i{i} = points ./ repmat(n, 1, ds.n_i(i));
			ds.z_i(i) = num;
		end

		function setZ_2d(ds, i, num)
			if (ds.n_i(i) ~= 2)
				error('can only use setZ_2d for 2D subsystem')
			end
			ds.Z_i{i} = zeros(num, 2);
			for j=1:num
				theta = (j-1)*pi/num;
				ds.Z_i{i}(j,:) = [cos(theta), sin(theta)];
			end
			ds.z_i(i) = num;
		end

		function ret = A(ds)
			ret = cell2mat(ds.A_ij);
		end

		function ret = B(ds)
			ret = blkdiag(ds.B_i{:});
		end

		function ret = E(ds)
			ret = blkdiag(ds.E_i{:});
		end
		
		function ret = Z(ds)
			ret = blkdiag(ds.Z_i{:});
		end

		function ret = Hu(ds)
			ret = blkdiag(ds.H_u_i{:});
		end

		function ret = hu(ds)
			ret = cell2mat(ds.h_u_i');
		end

		function ret = Hs(ds)
			ret = blkdiag(ds.H_s_i{:});
		end

		function ret = hs(ds)
			ret = cell2mat(ds.h_s_i');
		end

		function ret = Hd(ds)
			ret = blkdiag(ds.H_d_i{:}); 
		end

		function ret = n(ds)
			ret = sum(ds.n_i);
		end

		function ret = m(ds)
			ret = sum(ds.m_i);
		end

		function ret = p(ds)
			ret = sum(ds.p_i);
		end

	end

end