% Conversion from an inverse space velocity to an abstract space velocity
%
% function [APVel, JIA, JFA, JAI, JAF, FFP, IP, hipPRPos] = AbsFromInvVel(InvVel, AP, limbSign, RM)
%
% InvVel can be either an IPVel (struct) or FootVel (vector).
%
% Singularities and problematic regions of high sensitivity:
% - If fully retracted leg (fully extended is ok)
% - If the ankle point is at the same z-height as the hip
% - If the leg axis is almost parallel to the foot x-axis
%
% It is suggested to use the stable Jacobians JIA and JFA instead of the more unstable inverted ones JAI and JAF.
% i.e. APVel = JIA \ StructToMat(IPVel, 0)
%      APVel = JFA \ FootVel
%
function [APVel, JIA, JFA, JAI, JAF, FFP, IP, hipPRPos] = AbsFromInvVel(InvVel, AP, limbSign, RM) % Note: In final implementation of this function some aspects can still be optimised computationally...

	% Input arguments
	if nargin < 4
		RM = RobotModel;
	end

	% Avoid code duplication and just retrieve the required Jacobians from the reverse velocity space conversion function
	[~, ~, JIA, JFA, FFP, IP, hipPRPos] = InvFromAbsVel(AbstractPose, AP, limbSign, RM);

	% Calculate the required abstract space velocity
	if isstruct(InvVel)
		APVelVec = JIA \ StructToMat(InvVel, 0); % InvVel is an IPVel
	else
		APVelVec = JFA \ InvVel(:); % InvVel is a FootVel
	end
	APVel = AbstractPose(APVelVec(1:3), APVelVec(4:5), APVelVec(6));

	% Compute the inverse Jacobians explicitly (warns and gives Inf if singular)
	JAI = inv(JIA);
	JAF = inv(JFA);

end
% EOF