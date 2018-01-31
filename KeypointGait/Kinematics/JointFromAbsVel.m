% Conversion from an abstract space velocity to a joint space velocity
%
% function [JPVel, JJA] = JointFromAbsVel(APVel, P)
%
% P can be either the JP or AP to convert the velocity at.
% The JJA Jacobian transforms velocities of [AX AY AZ FAX FAY RET]' to [HY HR HP KP AP AR]'.
%
% Avoid the singularity at P.retraction = 0 or you may get Inf or NaN.
%
function [JPVel, JJA] = JointFromAbsVel(APVel, P)

	% Calculate the pose representation-dependent terms
	if isfield(P, 'retraction')
		A = 1 / sqrt(P.retraction*(2 - P.retraction));
	else
		A = 1 / sin(0.5*picut(P.kneePitch));
	end
	AVel = A * APVel.retraction;

	% Construct the output joint space velocity
	hipAngle = [APVel.angleX APVel.angleY-AVel APVel.angleZ];
	kneeAngle = 2*AVel;
	ankleAngle = [APVel.footAngleX-APVel.angleX APVel.footAngleY-APVel.angleY-AVel];
	JPVel = JointPose(hipAngle, kneeAngle, ankleAngle);

	% Compute the Jacobian matrix explicitly if required
	if nargout >= 2
		JJA = [0 0 1 0 0 0; 1 0 0 0 0 0; 0 1 0 0 0 -A; 0 0 0 0 0 2*A; 0 -1 0 0 1 -A; -1 0 0 1 0 0];
	end

end
% EOF