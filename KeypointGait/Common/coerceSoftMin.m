% Soft-coerce a value to the range [Mn,Inf], while softening up the coercion within range m of the bound
%
% function [y, coerced] = coerceSoftMin(x, Mn, m)
%
% x can be a vector or matrix of values.
%
function [y, coerced] = coerceSoftMin(x, Mn, m)

	% Error checking on the buffer range
	if m <= 0
		[y, coerced] = coerceMin(x, Mn);
		return
	end

	% Calculate the effective bound
	boundN = Mn + m;

	% Soft-coerce the value x as required
	y = nan(size(x));
	coerced = false(size(x));
	for i = 1:size(x,1)
		for j = 1:size(x,2)
			if x(i,j) < boundN
				y(i,j) = Mn + m*exp((x(i,j) - boundN) / m);
				coerced(i,j) = true;
			else
				y(i,j) = x(i,j);
				coerced(i,j) = false;
			end
		end
	end
end
% EOF