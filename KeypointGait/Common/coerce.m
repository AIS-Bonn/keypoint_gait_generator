% Hard-coerce a value to the range [Mn,Mp]
%
% function [y, coerced] = coerce(x, Mn, Mp)
%
% x can be a vector or matrix of values.
%
function [y, coerced] = coerce(x, Mn, Mp)
	y = nan(size(x));
	for i = 1:size(x,1)
		for j = 1:size(x,2)
			if Mn > Mp
				y(i,j) = 0.5*(Mn + Mp);
			elseif x(i,j) >= Mp
				y(i,j) = Mp;
			elseif x(i,j) <= Mn
				y(i,j) = Mn;
			else
				y(i,j) = x(i,j);
			end
		end
	end
	coerced = (Mn > Mp | y ~= x);
end
% EOF