% Conversion from inverse arm space to abstract arm space
%
% function [AAP] = ArmAbsFromInv(AIP, RM)
%
function [AAP] = ArmAbsFromInv(AIP, RM)

	% Input arguments
	if nargin < 2
		RM = RobotModel;
	end

	% Convert via arm joint space
	AJP = ArmJointFromInv(AIP, RM);
	AAP = ArmAbsFromJoint(AJP);

end
% EOF