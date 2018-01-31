% Conversion from inverse space to abstract space
%
% function [AP, JP, exact] = AbsFromInv(IP, limbSign, RM)
%
function [AP, JP, exact] = AbsFromInv(IP, limbSign, RM)

	% Input arguments
	if nargin < 3
		RM = RobotModel;
	end

	% Calculate the leg inverse kinematics
	[JP, AP, exact] = LegInvKin(IP, limbSign, RM);

end
% EOF