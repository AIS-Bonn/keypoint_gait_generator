% Conversion from an ankle inverse space velocity to a foot inverse space velocity
%
% function [FootVel, JFI] = FootFromInvVel(IPVel, footRot, limbSign, RM)
%
% footRot is the quaternion orientation of the foot, as per IP.footRot.
%
function [FootVel, JFI] = FootFromInvVel(IPVel, footRot, limbSign, RM)

	% Input arguments
	if nargin < 4
		RM = RobotModel;
	end

	% Calculate the required foot velocity
	footVec = [RM.footOffsetX; limbSign*RM.footOffsetY; -RM.footOffsetZ];
	rotFootVec = QuatRotVec(footRot, footVec);
	FootVel = [IPVel.ankleVel(:) + cross(IPVel.footAngVel(:), rotFootVec); IPVel.footAngVel(:)];

	% Calculate the Jacobian if required
	if nargout >= 2
		JFI = [eye(3) [0 rotFootVec(3) -rotFootVec(2); -rotFootVec(3) 0 rotFootVec(1); rotFootVec(2) -rotFootVec(1) 0]; zeros(3) eye(3)];
	end

end
% EOF