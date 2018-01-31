% Conversion from abstract arm space to inverse arm space
%
% function [AIP, elbowPos] = ArmInvFromAbs(AAP, RM)
%
function [AIP, elbowPos] = ArmInvFromAbs(AAP, RM)

	% Input arguments
	if nargin < 2
		RM = RobotModel;
	end

	% Convert via joint space
	AJP = ArmJointFromAbs(AAP);
	[AIP, elbowPos] = ArmInvFromJoint(AJP, RM);

end
% EOF