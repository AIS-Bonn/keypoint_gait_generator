% Wrap angle in radians to (M,M+2*pi]
function [out] = picutMin(in, M)
	out = in + 2*pi*floor((M - in)/(2*pi) + 1);
end