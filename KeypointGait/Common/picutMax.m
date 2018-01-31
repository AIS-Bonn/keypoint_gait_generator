% Wrap angle in radians to (M-2*pi,M]
function [out] = picutMax(in, M)
	out = in + 2*pi*floor((M - in)/(2*pi));
end