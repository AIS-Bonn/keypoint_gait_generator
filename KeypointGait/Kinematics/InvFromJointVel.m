% Conversion from a joint space velocity to an inverse space velocity
%
% function [IPVel, FootVel, JIJ, JFJ, FFP, IP, hipPRPos, kneePos] = InvFromJointVel(JPVel, JP, limbSign, RM)
%
% The JIJ Jacobian transforms velocities of [HY HR HP KP AP AR]' to [ankleVel footAngVel]'.
% The JFJ Jacobian transforms velocities of [HY HR HP KP AP AR]' to FootVel = [FFPVel footAngVel]'.
%
% IPVel is a struct as returned by InversePoseVel
% FootVel is a column vector
% JIJ and JFJ are square 6x6 Jacobian matrices
%
function [IPVel, FootVel, JIJ, JFJ, FFP, IP, hipPRPos, kneePos] = InvFromJointVel(JPVel, JP, limbSign, RM) % Note: In final implementation of this function some aspects can still be optimised computationally...

	% Input arguments
	if nargin < 4
		RM = RobotModel;
	end

	% Calculate the joint rotations
	hipYRot = RotmatFromAxis('z', JP.hipYaw);
	hipRRot = RotmatFromAxis('x', JP.hipRoll);
	hipPRot = RotmatFromAxis('y', JP.hipPitch);
	kneePRot = RotmatFromAxis('y', JP.kneePitch);
	anklePRot = RotmatFromAxis('y', JP.anklePitch);
	ankleRRot = RotmatFromAxis('x', JP.ankleRoll);

	% Calculate the required coordinate frame orientations
	RY = hipYRot;
	RJ = RY*hipRRot;
	RK = RJ*hipPRot;
	RS = RK*kneePRot;
	RA = RS*anklePRot;
	RF = RA*ankleRRot;

	% Calculate the joint axis vectors in body-fixed coordinates
	axishy = [0; 0; 1];
	axishr = RJ(:,1);
	axishp = RJ(:,2);
	axiskp = axishp;
	axisap = axiskp;
	axisar = RF(:,1);

	% Calculate the joint angular velocity vectors
	omegahy = axishy * JPVel.hipYaw;
	omegahr = axishr * JPVel.hipRoll;
	omegahp = axishp * JPVel.hipPitch;
	omegakp = axiskp * JPVel.kneePitch;
	omegaap = axisap * JPVel.anklePitch;
	omegaar = axisar * JPVel.ankleRoll;

	% Calculate the foot angular velocity
	footAngVel = omegahy + omegahr + omegahp + omegakp + omegaap + omegaar;

	% Hip and limb offset vectors
	hipVec = [RM.hipOffsetX; limbSign*RM.hipOffsetY; 0];
	limbVec = [0; 0; -RM.legLinkLength];
	footVec = [RM.footOffsetX; limbSign*RM.footOffsetY; -RM.footOffsetZ];

	% Calculate the leg positions
	PH = -2*limbVec - hipVec;
	PY = PH + RY*hipVec;
	PK = PY + RK*limbVec;
	PA = PK + RS*limbVec;
	PF = PA + RF*footVec;

	% Calculate the relative leg displacements
	PHA = PA - PH;
	PYA = PA - PY;
	PKA = PA - PK;
	PHF = PF - PH;
	PYF = PF - PY;
	PKF = PF - PK;
	PAF = PF - PA;

	% Calculate the ankle and foot floor point velocities
	ankleVel = cross(omegahy, PHA) + cross(omegahr + omegahp, PYA) + cross(omegakp, PKA);
	FFPVel = cross(omegahy, PHF) + cross(omegahr + omegahp, PYF) + cross(omegakp, PKF) + cross(omegaap + omegaar, PAF);

	% Construct the output inverse pose velocities
	IPVel = InversePoseVel(ankleVel, footAngVel);
	FootVel = [FFPVel(:); footAngVel(:)];

	% Compute additional outputs if required
	if nargout >= 3
		JIJ = [cross(axishy,PHA) cross(axishr,PYA) cross(axishp,PYA) cross(axiskp,PKA) zeros(3,2); axishy axishr axishp axiskp axisap axisar];
	end
	if nargout >= 4
		JFJ = [cross(axishy,PHF) cross(axishr,PYF) cross(axishp,PYF) cross(axiskp,PKF) cross(axisap,PAF) cross(axisar,PAF); axishy axishr axishp axiskp axisap axisar];
	end
	if nargout >= 5
		FFP = PF(:)';
	end
	if nargout >= 6
		IP = InversePose(PA, QuatFromRotmat(RF));
		hipPRPos = PY(:)';
		kneePos = PK(:)';
	end

end
% EOF