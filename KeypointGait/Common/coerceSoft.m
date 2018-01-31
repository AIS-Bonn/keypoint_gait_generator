% Soft-coerce a value to the range [Mn,Mp], while softening up the coercion within range m of each of the two bounds
%
% function [y, coerced] = coerceSoft(x, Mn, Mp, m)
%
% x can be a vector or matrix of values.
%
function [y, coerced] = coerceSoft(x, Mn, Mp, m)

	% Error checking on the buffer range
	maxm = 0.5*(Mp - Mn);
	if m <= 0 || maxm <= 0
		[y, coerced] = coerce(x, Mn, Mp);
		return
	end

	% Calculate the required centre slope
	slope = 1;
	if m > maxm
		slope = exp(maxm/m - 1);
		m = maxm;
	end

	% Calculate the effective bounds
	boundN = Mn + m;
	boundP = Mp - m;

	% Soft-coerce the value x as required
	y = nan(size(x));
	coerced = false(size(x));
	for i = 1:size(x,1)
		for j = 1:size(x,2)
			if x(i,j) > boundP
				y(i,j) = Mp - m*exp(-(x(i,j) - boundP) * slope / m);
				coerced(i,j) = true;
			elseif x(i,j) < boundN
				y(i,j) = Mn + m*exp((x(i,j) - boundN) * slope / m);
				coerced(i,j) = true;
			else
				y(i,j) = x(i,j);
				coerced(i,j) = false;
			end
		end
	end
end
% EOF