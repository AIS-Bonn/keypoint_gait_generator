% InvFromFFPRot.m - Philipp Allgeuer - 31/10/16
% Get the inverse pose from a foot floor point and rotation.
%
% function [IP] = InvFromFFPRot(FFP, Rot, limbSign, RM)
%
function [IP] = InvFromFFPRot(FFP, Rot, limbSign, RM)
	if numel(Rot) == 4
		Q = Rot;
		R = RotmatFromQuat(Q);
	else
		R = Rot;
		Q = QuatFromRotmat(R);
	end
	if nargin < 3
		limbSign = 1;
	end
	limbSign = sgn(limbSign);
	if nargin < 4
		RM = RobotModel;
	end
	IP = InversePose(FFP(:) - R*[RM.footOffsetX; limbSign*RM.footOffsetY; -RM.footOffsetZ], Q);
end
% EOF