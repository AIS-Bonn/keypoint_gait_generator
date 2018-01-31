% Conversion from a foot inverse space velocity to an ankle inverse space velocity
%
% function [IPVel, JIF] = InvFromFootVel(FootVel, footRot, limbSign, RM)
%
% footRot is the quaternion orientation of the foot, as per IP.footRot.
%
function [IPVel, JIF] = InvFromFootVel(FootVel, footRot, limbSign, RM)

	% Input arguments
	if nargin < 4
		RM = RobotModel;
	end

	% Calculate the required foot velocity
	footVec = [RM.footOffsetX; limbSign*RM.footOffsetY; -RM.footOffsetZ];
	rotFootVec = QuatRotVec(footRot, footVec);
	IPVel = InversePoseVel(FootVel(1:3) - cross(FootVel(4:6), rotFootVec), FootVel(4:6));

	% Calculate the Jacobian if required
	if nargout >= 2
		JIF = [eye(3) [0 -rotFootVec(3) rotFootVec(2); rotFootVec(3) 0 -rotFootVec(1); -rotFootVec(2) rotFootVec(1) 0]; zeros(3) eye(3)];
	end

end
% EOF