% Generate a random arm abstract pose.
%
% function [AAP] = RandArmAbsPose()
%
function [AAP] = RandArmAbsPose()

	% Generate the required random abstract pose
	randUnit = 2*rand(1,2) - 1;
	AAP = ArmAbstractPose((pi/2)*randUnit(1:2), rand);

end
% EOF