% Conversion from joint space to inverse space
%
% function [IP, hipPRPos, kneePos] = InvFromJoint(JP, limbSign, RM)
%
function [IP, hipPRPos, kneePos] = InvFromJoint(JP, limbSign, RM)

	% Input arguments
	if nargin < 3
		RM = RobotModel;
	end

	% Calculate the foot rotation
	[thighRot, shankRot] = QuatFromZXYK(JP.hipYaw, JP.hipRoll, JP.hipPitch, JP.kneePitch);
	ankleRot = QuatFromYX(JP.anklePitch, JP.ankleRoll);
	footRot = ComposeQuat(shankRot, ankleRot);

	% Robot dimensions
	L = RM.legLinkLength;
	Ldbl = 2*L;
	hx = RM.hipOffsetX;
	hy = limbSign*RM.hipOffsetY;

	% Calculate the hip PR point
	Cz = cos(JP.hipYaw) - 1;
	sz = sin(JP.hipYaw);
	hipPRPos = [hx*Cz-hy*sz hx*sz+hy*Cz Ldbl];

	% Calculate the ankle position
	kneePos = hipPRPos + QuatRotVecPureZ(thighRot, -L)';
	anklePos = kneePos + QuatRotVecPureZ(shankRot, -L)';

	% Construct the output inverse pose
	IP = InversePose(anklePos, footRot);

end

% Calculate quaternions from successive Z, X, Y and K (also Y) rotations
function [QuatZXY, QuatZXYK] = QuatFromZXYK(angZ, angX, angY, angK)

	hangZ = 0.5*angZ;
	hangX = 0.5*angX;
	hangY = 0.5*angY;
	hangK = 0.5*angK;

	cz = cos(hangZ);
	sz = sin(hangZ);
	cx = cos(hangX);
	sx = sin(hangX);
	cy = cos(hangY);
	sy = sin(hangY);

	hangYK = hangY + hangK;
	cyk = cos(hangYK);
	syk = sin(hangYK);

	czcx = cz*cx;
	czsx = cz*sx;
	szcx = sz*cx;
	szsx = sz*sx;

	QuatZXY = [czcx*cy-szsx*sy czsx*cy-szcx*sy czcx*sy+cy*szsx szcx*cy+czsx*sy];
	QuatZXYK = [czcx*cyk-szsx*syk czsx*cyk-szcx*syk czcx*syk+cyk*szsx szcx*cyk+czsx*syk];

end

% Calculate a quaternion from successive Y and X rotations
function [QuatYX] = QuatFromYX(angY, angX)

	hangY = 0.5*angY;
	hangX = 0.5*angX;

	cy = cos(hangY);
	sy = sin(hangY);
	cx = cos(hangX);
	sx = sin(hangX);

	QuatYX = [cx*cy sx*cy cx*sy -sx*sy];

end
% EOF