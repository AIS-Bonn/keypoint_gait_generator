% Conversion from arm joint space to inverse arm space
%
% function [AIP, elbowPos] = ArmInvFromJoint(AJP, RM)
%
function [AIP, elbowPos] = ArmInvFromJoint(AJP, RM)

	% Input arguments
	if nargin < 2
		RM = RobotModel;
	end

	% Calculate the joint rotations
	shoulderRot = ComposeQuat(QuatFromAxis('y', AJP.shoulderPitch), QuatFromAxis('x', AJP.shoulderRoll));
	elbowRot = QuatFromAxis('y', AJP.elbowPitch);

	% Calculate the hand rotation
	handRot = ComposeQuat(shoulderRot, elbowRot);

	% Calculate the hand position
	L = RM.armLinkLength;
	limbVec = [0; 0; L];
	elbowPos = 2*limbVec - QuatRotVec(shoulderRot, limbVec);
	handPos = elbowPos - QuatRotVec(handRot, limbVec);

	% Construct the output inverse pose
	AIP = ArmInversePose(handPos);
	elbowPos = elbowPos(:)';

end
% EOF