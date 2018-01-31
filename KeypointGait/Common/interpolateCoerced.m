% Coerced interpolation
% function [y] = interpolateCoerced(X, Y, x)
function [y] = interpolateCoerced(X, Y, x)
	if X(1) == X(2)
		y = repmat(0.5*(Y(1) + Y(2)), size(x));
	else
		u = (x - X(1))/(X(2) - X(1));
		u(u < 0.0) = 0.0;
		u(u > 1.0) = 1.0;
		y = Y(1) + u*(Y(2) - Y(1));
	end
end
% EOF