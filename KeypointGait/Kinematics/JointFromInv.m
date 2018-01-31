% Conversion from inverse space to joint space
%
% function [JP, AP, exact] = JointFromInv(IP, limbSign, RM)
%
function [JP, AP, exact] = JointFromInv(IP, limbSign, RM)

	% Input arguments
	if nargin < 3
		RM = RobotModel;
	end

	% Calculate the leg inverse kinematics
	[JP, AP, exact] = LegInvKin(IP, limbSign, RM);

end
% EOF