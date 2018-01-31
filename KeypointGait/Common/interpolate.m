% Interpolation
% function [y] = interpolate(X, Y, x)
function [y] = interpolate(X, Y, x)
	if X(1) == X(2)
		y = repmat(0.5*(Y(1) + Y(2)), size(x));
	else
		y = Y(1) + (Y(2) - Y(1))*((x - X(1))/(X(2) - X(1)));
	end
end
% EOF