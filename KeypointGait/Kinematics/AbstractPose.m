% Construct an abstract pose representation
%
% function [AP] = AbstractPose(legAngle, footAngle, legRetraction)
%
function [AP] = AbstractPose(legAngle, footAngle, legRetraction)

	% Return zero pose if no input arguments are given
	if nargin == 0
		legAngle = [0 0 0];
		footAngle = [0 0];
		legRetraction = 0;
	end

	% Error checking
	if numel(legAngle) ~= 3
		error('AbstractPose: Invalid leg angle!');
	end
	if numel(footAngle) ~= 2
		error('AbstractPose: Invalid foot angle!');
	end
	if numel(legRetraction) ~= 1
		error('AbstractPose: Invalid leg retraction!');
	end

	% Construct the required representation
	AP = struct();
	AP.angleX = legAngle(1);
	AP.angleY = legAngle(2);
	AP.angleZ = legAngle(3);
	AP.footAngleX = footAngle(1);
	AP.footAngleY = footAngle(2);
	AP.retraction = legRetraction;

end
% EOF