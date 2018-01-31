% Hard-coerce a value to the range [-M,M]
%
% function [y, coerced] = coerceAbs(x, M)
%
% x can be a vector or matrix of values.
%
function [y, coerced] = coerceAbs(x, M)
	y = nan(size(x));
	for i = 1:size(x,1)
		for j = 1:size(x,2)
			if M < 0
				y(i,j) = 0;
			elseif x(i,j) >= M
				y(i,j) = M;
			elseif x(i,j) <= -M
				y(i,j) = -M;
			else
				y(i,j) = x(i,j);
			end
		end
	end
	coerced = (M < 0 | y ~= x);
end
% EOF