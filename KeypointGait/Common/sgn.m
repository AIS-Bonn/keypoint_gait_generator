% Sign function with output 1 or -1 only (sgn(0) = 1)
%
% function y = sgn(x)
function y = sgn(x)
	A = (x < 0);
	y = ones(size(x));
	y(A) = -1;
end
% EOF