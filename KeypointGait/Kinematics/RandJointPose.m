% Generate a random joint pose.
%
% function [JP] = RandJointPose(nice)
%
function [JP] = RandJointPose(nice)

	% Generate the required random joint pose
	randUnit = 2*rand(1,5) - 1;
	if nargin < 1
		nice = true;
	end
	if nice
		JP = JointPose(pi/2*randUnit(1:3) + [0 -pi/4 0], pi*rand, pi/2*randUnit(4:5));
	else
		JP = JointPose(6*pi*randUnit(1:3), pi*rand + 2*pi*randi([-3 2]), 6*pi*randUnit(4:5));
	end

end
% EOF