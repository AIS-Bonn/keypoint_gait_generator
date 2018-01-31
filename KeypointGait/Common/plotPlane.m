% plotPlane.m - Philipp Allgeuer - 13/09/16
% Plot an arbitrary plane based on point and normal vector.
%
% function [x, y, z] = plotPlane(normal, point, uval, vval)
%
function [x, y, z] = plotPlane(normal, point, uval, vval)

	% Input variables
	if nargin < 4
		vval = uval;
	end

	% Normalise the normal vector
	normalMag = norm(normal);
	if normalMag <= 0
		warning('Encountered normal of zero magnitude!');
		normal = [0 0 1];
	else
		normal = normal / normalMag;
	end
	normal = normal(:);

	% Calculate orthogonal tangent vectors that span the required plane
	[~, I] = max(abs(normal));
	index = 1:3;
	index(I) = [];
	orthvec = eye(3) - normal*normal';
	orthvec = orthvec(:,index);
	[~, I] = max([norm(orthvec(:,1)) norm(orthvec(:,2))]);
	u = orthvec(:,I) / norm(orthvec(:,I));
	v = cross(normal, u);

	% Calculate the plane coordinates
	[U, V] = meshgrid(uval, vval);
	x = point(1) + U*u(1) + V*v(1);
	y = point(2) + U*u(2) + V*v(2);
	z = point(3) + U*u(3) + V*v(3);

	% Plot the plane if no output arguments
	if nargout < 1
		surf(x, y, z);
		clear x y z;
	end

end
% EOF