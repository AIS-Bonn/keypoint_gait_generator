% Conversion from an abstract space velocity to an inverse space velocity
%
% function [IPVel, FootVel, JIA, JFA, FFP, IP, hipPRPos] = InvFromAbsVel(APVel, AP, limbSign, RM)
%
% The JIA Jacobian transforms velocities of [AX AY AZ FAX FAY RET]' to [ankleVel footAngVel]'.
% The JFJ Jacobian transforms velocities of [AX AY AZ FAX FAY RET]' to FootVel = [FFPVel footAngVel]'.
%
% IPVel is a struct as returned by InversePoseVel
% FootVel is a column vector
% JIA and JFA are square 6x6 Jacobian matrices
%
function [IPVel, FootVel, JIA, JFA, FFP, IP, hipPRPos] = InvFromAbsVel(APVel, AP, limbSign, RM) % Note: In final implementation of this function some aspects can still be optimised computationally...

	% Input arguments
	if nargin < 4
		RM = RobotModel;
	end

	% Calculate the abstract rotations
	angleZRot = RotmatFromAxis('z', AP.angleZ);
	angleXRot = RotmatFromAxis('x', AP.angleX);
	angleYRot = RotmatFromAxis('y', AP.angleY);
	footAngleYRot = RotmatFromAxis('y', AP.footAngleY - AP.angleY);
	footAngleXRot = RotmatFromAxis('x', AP.footAngleX - AP.angleX);

	% Calculate the required coordinate frame orientations
	RY = angleZRot;
	RJ = RY*angleXRot;
	RL = RJ*angleYRot;
	RA = RL*footAngleYRot;
	RF = RA*footAngleXRot;

	% Calculate the required abstract axis vectors
	axisz = [0; 0; 1]; % zH
	axisx = RJ(:,1);   % xJ
	axisy = RL(:,2);   % yL
	axisfx = RF(:,1);  % xF
	axisret = RL(:,3); % zL

	% Calculate the foot angular velocity
	footAngVel = (axisx - axisfx)*APVel.angleX + axisz*APVel.angleZ + axisfx*APVel.footAngleX + axisy*APVel.footAngleY;

	% Hip and limb offset vectors
	hipVec = [RM.hipOffsetX; limbSign*RM.hipOffsetY; 0];
	limbVec = [0; 0; -RM.legLinkLength];
	limbAxisVec = 2*(1 - AP.retraction)*limbVec;
	footVec = [RM.footOffsetX; limbSign*RM.footOffsetY; -RM.footOffsetZ];

	% Calculate the leg positions
	PH = -2*limbVec - hipVec;
	PY = PH + RY*hipVec;
	PA = PY + RL*limbAxisVec;
	PF = PA + RF*footVec;

	% Calculate the relative leg displacements
	PYA = PA - PY;
	PHA = PA - PH;
	PYF = PF - PY;
	PAF = PF - PA;
	PHF = PF - PH;

	% Calculate the ankle and foot floor point velocities
	ankleVel = cross(axisx, PYA)*APVel.angleX + cross(axisy, PYA)*APVel.angleY + cross(axisz, PHA)*APVel.angleZ + 2*RM.legLinkLength*axisret*APVel.retraction;
	FFPVel = (cross(axisx, PYF) - cross(axisfx, PAF))*APVel.angleX + cross(axisy, PYA)*APVel.angleY + cross(axisz, PHF)*APVel.angleZ + cross(axisfx, PAF)*APVel.footAngleX + cross(axisy, PAF)*APVel.footAngleY + 2*RM.legLinkLength*axisret*APVel.retraction;

	% Construct the output inverse pose velocities
	IPVel = InversePoseVel(ankleVel, footAngVel);
	FootVel = [FFPVel(:); footAngVel(:)];

	% Compute additional outputs if required
	if nargout >= 3
		JIA = [cross(axisx, PYA) cross(axisy, PYA) cross(axisz, PHA) zeros(3,2) 2*RM.legLinkLength*axisret; axisx-axisfx zeros(3,1) axisz axisfx axisy zeros(3,1)];
	end
	if nargout >= 4
		JFA = [cross(axisx, PYF)-cross(axisfx, PAF) cross(axisy, PYA) cross(axisz, PHF) cross(axisfx, PAF) cross(axisy, PAF) 2*RM.legLinkLength*axisret; axisx-axisfx zeros(3,1) axisz axisfx axisy zeros(3,1)];
	end
	if nargout >= 5
		FFP = PF(:)';
	end
	if nargout >= 6
		IP = InversePose(PA, QuatFromRotmat(RF));
		hipPRPos = PY(:)';
	end

end
% EOF