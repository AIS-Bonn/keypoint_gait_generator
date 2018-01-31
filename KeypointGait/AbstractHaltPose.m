% AbstractHaltPose.m - Philipp Allgeuer - 01/09/16
% Returns the abstract halt pose
%
% function [AP] = AbstractHaltPose(limbSign, config)
%
function [AP] = AbstractHaltPose(limbSign, config)

	% Input variables
	if nargin < 1
		limbSign = 1;
	end
	limbSign = sgn(limbSign);
	if nargin < 2
		config = ConfigVars;
	end

	% Define the abstract pose parameters
	legAngle = [limbSign*config.haltLegAngleX config.haltLegAngleY limbSign*config.haltLegAngleZ];
	footAngle = [limbSign*config.haltFootAngleX config.haltFootAngleY];
	legRetraction = config.haltLegRetraction;

	% Construct the abstract pose
	AP = AbstractPose(legAngle, footAngle, legRetraction);

end
% EOF