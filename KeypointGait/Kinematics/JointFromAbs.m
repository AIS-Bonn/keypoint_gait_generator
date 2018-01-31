% Conversion from abstract space to joint space
%
% function [JP] = JointFromAbs(AP)
%
% It is assumed that all angles in AP are in (-pi,pi].
% Then, all angles in JP are in (-pi,pi].
% The knee angle is always in [0,pi].
%
function [JP] = JointFromAbs(AP)

	% Convert between the spaces as required
	alpha = acos(coerce(1 - AP.retraction, 0.0, 1.0)); % In [0,pi/2]
	hipAngle = [AP.angleX picut(AP.angleY-alpha) AP.angleZ];
	kneeAngle = 2*alpha; % In [0,pi]
	ankleAngle = [picut(AP.footAngleX-AP.angleX) picut(AP.footAngleY-AP.angleY-alpha)];
	JP = JointPose(hipAngle, kneeAngle, ankleAngle);

end
% EOF