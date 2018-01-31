% Construct an abstract arm pose representation
%
% function [AAP] = ArmAbstractPose(armAngle, armRetraction)
%
function [AAP] = ArmAbstractPose(armAngle, armRetraction)

	% Return zero pose if no input arguments are given
	if nargin == 0
		armAngle = [0 0];
		armRetraction = 0;
	end

	% Error checking
	if numel(armAngle) ~= 2
		error('ArmAbstractPose: Invalid arm angle!');
	end
	if numel(armRetraction) ~= 1
		error('ArmAbstractPose: Invalid arm retraction!');
	end

	% Construct the required representation
	AAP = struct();
	AAP.angleX = armAngle(1);
	AAP.angleY = armAngle(2);
	AAP.retraction = armRetraction;

end
% EOF