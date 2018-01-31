% Wrap angle in radians to (-pi,pi]
function [out] = picut(in)
	out = in + 2*pi*floor((pi - in)/(2*pi));
end