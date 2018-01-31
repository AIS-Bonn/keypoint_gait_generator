% PicutJoint.m - Philipp Allgeuer - 22/03/17
% Picut the relevant fields of a joint pose.
%
% function [JP] = PicutJoint(JP)
%
function [JP] = PicutJoint(JP)

	% Wrap all angles to (-pi,pi]
	JP.hipYaw = picut(JP.hipYaw);
	JP.hipRoll = picut(JP.hipRoll);
	JP.hipPitch = picut(JP.hipPitch);
	JP.kneePitch = picut(JP.kneePitch);
	JP.anklePitch = picut(JP.anklePitch);
	JP.ankleRoll = picut(JP.ankleRoll);

end
% EOF