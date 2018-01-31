% RandArmInvPose.m - Philipp Allgeuer - 06/02/17
% Generate a random inverse arm pose.
%
% function [AIP, AAP] = RandArmInvPose(RM)
%
function [AIP, AAP] = RandArmInvPose(RM)
	if nargin < 1
		RM = RobotModel;
	end
	AAP = RandArmAbsPose;
	AIP = ArmInvFromAbs(AAP, RM);
end
% EOF