% FootFloorPoint.m - Philipp Allgeuer - 06/09/16
% Get the foot floor point from an inverse pose representation.
%
% function [FFP] = FootFloorPoint(IP, limbSign, RM)
%
function [FFP] = FootFloorPoint(IP, limbSign, RM)
	if nargin < 2
		limbSign = 1;
	end
	limbSign = sgn(limbSign);
	if nargin < 3
		RM = RobotModel;
	end
	FFP = IP.anklePos + (RotmatFromQuat(IP.footRot)*[RM.footOffsetX; limbSign*RM.footOffsetY; -RM.footOffsetZ])';
end
% EOF