% RandInvPose.m - Philipp Allgeuer - 31/10/16
% Generate a random inverse pose.
%
% function [IP, AP] = RandInvPose(nice, limbSign, RM)
%
function [IP, AP] = RandInvPose(nice, limbSign, RM)
	if nargin < 1
		nice = true;
	end
	if nargin < 2
		limbSign = +1;
	end
	if nargin < 3
		RM = RobotModel;
	end
	AP = RandAbsPose(nice);
	IP = InvFromAbs(AP, limbSign, RM);
end
% EOF