% Conversion from a joint space velocity to an abstract space velocity
%
% function [APVel, JAJ] = AbsFromJointVel(JPVel, P)
%
% P can be either the JP or AP to convert the velocity at.
% The JAJ Jacobian transforms velocities of [HY HR HP KP AP AR]' to [AX AY AZ FAX FAY RET]'.
%
function [APVel, JAJ] = AbsFromJointVel(JPVel, P)

	% Calculate the pose representation-dependent terms
	if isfield(P, 'retraction')
		A = 0.5*sqrt(P.retraction*(2 - P.retraction));
	else
		A = 0.5*sin(0.5*picut(P.kneePitch));
	end

	% Construct the output abstract space velocity
	legAngle = [JPVel.hipRoll JPVel.hipPitch+0.5*JPVel.kneePitch JPVel.hipYaw];
	footAngle = [JPVel.hipRoll+JPVel.ankleRoll JPVel.hipPitch+JPVel.kneePitch+JPVel.anklePitch];
	legRetraction = A * JPVel.kneePitch;
	APVel = AbstractPose(legAngle, footAngle, legRetraction);

	% Compute the Jacobian matrix explicitly if required
	if nargout >= 2
		JAJ = [0 1 0 0 0 0; 0 0 1 0.5 0 0; 1 0 0 0 0 0; 0 1 0 0 0 1; 0 0 1 1 1 0; 0 0 0 A 0 0];
	end

end
% EOF