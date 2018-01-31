% CalcHipPose.m - Philipp Allgeuer - 29/03/17
% Calculate the hipPRPos of a joint or abstract pose.
%
% function [hipPRPos] = CalcHipPose(P, limbSign, RM)
%
% P can be a JP or AP.
%
function [hipPRPos] = CalcHipPose(P, limbSign, RM)

	% Input arguments
	if nargin < 3
		RM = RobotModel;
	end

	% Robot dimensions
	L = RM.legLinkLength;
	Ldbl = 2*L;
	hx = RM.hipOffsetX;
	hy = limbSign*RM.hipOffsetY;

	% Get the hip yaw
	if isfield(P, 'retraction')
		hipYaw = P.angleZ;
	else
		hipYaw = P.hipYaw;
	end

	% Calculate the hip PR point
	Cz = cos(hipYaw) - 1;
	sz = sin(hipYaw);
	hipPRPos = [hx*Cz-hy*sz hx*sz+hy*Cz Ldbl];

end
% EOF