% Soft-coerce a value to the range [-Inf,Mp], while softening up the coercion within range m of the bound
%
% function [y, coerced] = coerceSoftMax(x, Mp, m)
%
% x can be a vector or matrix of values.
%
function [y, coerced] = coerceSoftMax(x, Mp, m)

	% Error checking on the buffer range
	if m <= 0
		[y, coerced] = coerceMax(x, Mp);
		return
	end

	% Calculate the effective bound
	boundP = Mp - m;

	% Soft-coerce the value x as required
	y = nan(size(x));
	coerced = false(size(x));
	for i = 1:size(x,1)
		for j = 1:size(x,2)
			if x(i,j) > boundP
				y(i,j) = Mp - m*exp(-(x(i,j) - boundP) / m);
				coerced(i,j) = true;
			else
				y(i,j) = x(i,j);
				coerced(i,j) = false;
			end
		end
	end
end
% EOF