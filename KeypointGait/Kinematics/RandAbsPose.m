% Generate a random abstract pose.
%
% function [AP] = RandAbsPose(nice)
%
function [AP] = RandAbsPose(nice)

	% Generate the required random abstract pose
	randUnit = 2*rand(1,5) - 1;
	if nargin < 1
		nice = true;
	end
	if nice
		AP = AbstractPose(pi/2*randUnit(1:3), pi/2*randUnit(4:5), rand);
	else
		AP = AbstractPose(6*pi*randUnit(1:3), 6*pi*randUnit(4:5), rand);
	end

end
% EOF