% Generate a random arm joint pose.
%
% function [AJP] = RandArmJointPose()
%
function [AJP] = RandArmJointPose()

	% Generate the required random joint pose
	randUnit = 2*rand(1,2) - 1;
	AJP = ArmJointPose((pi/2)*randUnit(1:2) + [0 pi/4], -pi*rand);

end
% EOF