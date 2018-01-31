% Conversion from joint space to abstract space
%
% function [AP] = AbsFromJoint(JP)
%
% It is assumed that all angles in JP are in (-pi,pi].
% Then, all angles in AP are in (-pi,pi].
% The leg retraction is always in [0,1].
%
function [AP] = AbsFromJoint(JP)

	% Convert between the spaces as required
	alpha = 0.5*picut(JP.kneePitch); % Note: If we don't picut here, then an unwrapped input would produce an incorrect output, not just an unwrapped one...
	legAngle = [JP.hipRoll picut(JP.hipPitch+alpha) JP.hipYaw];
	footAngle = [picut(JP.ankleRoll+legAngle(1)) picut(JP.anklePitch+legAngle(2)+alpha)];
	legRetraction = coerce(1 - cos(alpha), 0.0, 1.0);
	AP = AbstractPose(legAngle, footAngle, legRetraction);

end
% EOF