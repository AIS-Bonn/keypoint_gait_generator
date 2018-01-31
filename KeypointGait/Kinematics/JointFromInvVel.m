% Conversion from an inverse space velocity to a joint space velocity
%
% function [JPVel, JIJ, JFJ, JJI, JJF, FFP, IP, hipPRPos, kneePos] = JointFromInvVel(InvVel, JP, limbSign, RM)
%
% InvVel can be either an IPVel (struct) or FootVel (vector).
% 
% Singularities and problematic regions of high sensitivity:
% - If fully extended or retracted leg
% - If the ankle point is at the same z-height as the hip
% - If the leg axis is almost parallel to the foot x-axis
%
% It is suggested to use the stable Jacobians JIJ and JFJ instead of the more unstable inverted ones JJI and JJF.
% i.e. JPVel = JIJ \ StructToMat(IPVel, 0)
%      JPVel = JFJ \ FootVel
%
function [JPVel, JIJ, JFJ, JJI, JJF, FFP, IP, hipPRPos, kneePos] = JointFromInvVel(InvVel, JP, limbSign, RM) % Note: In final implementation of this function some aspects can still be optimised computationally...

	% Input arguments
	if nargin < 4
		RM = RobotModel;
	end

	% Avoid code duplication and just retrieve the required Jacobians from the reverse velocity space conversion function
	[~, ~, JIJ, JFJ, FFP, IP, hipPRPos, kneePos] = InvFromJointVel(JointPose, JP, limbSign, RM);

	% Calculate the required joint space velocity
	if isstruct(InvVel)
		JPVelVec = JIJ \ StructToMat(InvVel, 0); % InvVel is an IPVel
	else
		JPVelVec = JFJ \ InvVel(:); % InvVel is a FootVel
	end
	JPVel = JointPose(JPVelVec([2 3 1]), JPVelVec(4), JPVelVec([6 5]));

	% Compute the inverse Jacobians explicitly (warns and gives Inf if singular)
	JJI = inv(JIJ);
	JJF = inv(JFJ);

end
% EOF