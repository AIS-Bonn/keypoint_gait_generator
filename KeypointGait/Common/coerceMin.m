% Hard-coerce a value to the range [Mn,Inf]
%
% function [y, coerced] = coerceMin(x, Mn)
%
% x can be a vector or matrix of values.
%
function [y, coerced] = coerceMin(x, Mn)
	y = nan(size(x));
	for i = 1:size(x,1)
		for j = 1:size(x,2)
			if x(i,j) <= Mn
				y(i,j) = Mn;
			else
				y(i,j) = x(i,j);
			end
		end
	end
	coerced = (y ~= x);
end
% EOF