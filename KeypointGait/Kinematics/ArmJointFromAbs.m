% Conversion from arm abstract space to joint space
%
% function [AJP] = ArmJointFromAbs(AAP)
%
function [AJP] = ArmJointFromAbs(AAP)

	% Convert between the spaces as required
	alpha = acos(coerce(1 - AAP.retraction, 0.0, 1.0));
	shoulderAngle = [AAP.angleX AAP.angleY+alpha];
	elbowAngle = -2*alpha;
	AJP = ArmJointPose(shoulderAngle, elbowAngle);

end
% EOF