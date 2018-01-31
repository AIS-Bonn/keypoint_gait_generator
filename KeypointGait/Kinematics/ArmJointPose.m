% Construct an arm joint pose representation
%
% function [AJP] = ArmJointPose(shoulderAngle, elbowAngle)
%
function [AJP] = ArmJointPose(shoulderAngle, elbowAngle)

	% Return zero pose if no input arguments are given
	if nargin == 0
		shoulderAngle = [0 0];
		elbowAngle = 0;
	end

	% Error checking
	if numel(shoulderAngle) ~= 2
		error('ArmJointPose: Invalid shoulder angle!');
	end
	if numel(elbowAngle) ~= 1
		error('ArmJointPose: Invalid elbow angle!');
	end

	% Construct the required representation
	AJP = struct();
	AJP.shoulderPitch = shoulderAngle(2);
	AJP.shoulderRoll = shoulderAngle(1);
	AJP.elbowPitch = elbowAngle(1);

end
% EOF