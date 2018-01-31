% Construct a joint pose representation
%
% function [JP] = JointPose(hipAngle, kneeAngle, ankleAngle)
%
function [JP] = JointPose(hipAngle, kneeAngle, ankleAngle)

	% Return zero pose if no input arguments are given
	if nargin == 0
		hipAngle = [0 0 0];
		kneeAngle = 0;
		ankleAngle = [0 0];
	end

	% Error checking
	if numel(hipAngle) ~= 3
		error('JointPose: Invalid hip angle!');
	end
	if numel(kneeAngle) ~= 1
		error('JointPose: Invalid knee angle!');
	end
	if numel(ankleAngle) ~= 2
		error('JointPose: Invalid ankleAngle!');
	end

	% Construct the required representation
	JP = struct();
	JP.hipYaw = hipAngle(3);
	JP.hipRoll = hipAngle(1);
	JP.hipPitch = hipAngle(2);
	JP.kneePitch = kneeAngle(1);
	JP.anklePitch = ankleAngle(2);
	JP.ankleRoll = ankleAngle(1);

end
% EOF