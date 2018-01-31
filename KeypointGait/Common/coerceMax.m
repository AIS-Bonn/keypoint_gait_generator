% Hard-coerce a value to the range [-Inf,Mp]
%
% function [y, coerced] = coerceMax(x, Mp)
%
% x can be a vector or matrix of values.
%
function [y, coerced] = coerceMax(x, Mp)
	y = nan(size(x));
	for i = 1:size(x,1)
		for j = 1:size(x,2)
			if x(i,j) >= Mp
				y(i,j) = Mp;
			else
				y(i,j) = x(i,j);
			end
		end
	end
	coerced = (y ~= x);
end
% EOF