% Soft-coerce a value to the range [-M,M], while softening up the coercion within range m of each of the two bounds
%
% function [y, coerced] = coerceSoftAbs(x, M, m)
%
% x can be a vector or matrix of values.
%
function [y, coerced] = coerceSoftAbs(x, M, m)

	% Error checking on the buffer range
	maxm = M;
	if m <= 0 || maxm <= 0
		[y, coerced] = coerceAbs(x, M);
		return
	end

	% Calculate the required centre slope
	slope = 1;
	if m > maxm
		slope = exp(maxm/m - 1);
		m = maxm;
	end

	% Calculate the effective absolute bound
	bound = M - m;

	% Soft-coerce the value x as required
	y = nan(size(x));
	coerced = false(size(x));
	for i = 1:size(x,1)
		for j = 1:size(x,2)
			if x(i,j) > bound
				y(i,j) = M - m*exp(-(x(i,j) - bound) * slope / m);
				coerced(i,j) = true;
			elseif x(i,j) < -bound
				y(i,j) = -M + m*exp((x(i,j) + bound) * slope / m);
				coerced(i,j) = true;
			else
				y(i,j) = x(i,j);
				coerced(i,j) = false;
			end
		end
	end
end
% EOF