% Conversion from arm joint space to abstract arm space
%
% function [AAP] = ArmAbsFromJoint(AJP)
%
function [AAP] = ArmAbsFromJoint(AJP)

	% Convert between the spaces as required
	alpha = 0.5*picut(AJP.elbowPitch); % Note: If we don't picut here, then an unwrapped input would produce an incorrect output, not just an unwrapped one...
	armAngle = [AJP.shoulderRoll picut(AJP.shoulderPitch+alpha)];
	armRetraction = coerce(1.0 - cos(alpha), 0.0, 1.0);
	AAP = ArmAbstractPose(armAngle, armRetraction);

end
% EOF