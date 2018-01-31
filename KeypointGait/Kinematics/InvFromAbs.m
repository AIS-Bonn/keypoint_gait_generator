% Conversion from abstract space to inverse space
%
% function [IP, hipPRPos, kneePos] = InvFromAbs(AP, limbSign, RM)
%
function [IP, hipPRPos, kneePos] = InvFromAbs(AP, limbSign, RM)

	% Input arguments
	if nargin < 3
		RM = RobotModel;
	end

	% Calculate the foot rotation
	[angleZRot, legAxisRot] = QuatFromZXY(AP.angleZ, AP.angleX, AP.angleY);
	footAngleRot = QuatFromYX(AP.footAngleY - AP.angleY, AP.footAngleX - AP.angleX);
	footRot = ComposeQuat(legAxisRot, footAngleRot);

	% Robot dimensions
	L = RM.legLinkLength;
	Ldbl = 2*L;
	hx = RM.hipOffsetX;
	hy = limbSign*RM.hipOffsetY;

	% Calculate the ankle position
	legAxisLen = Ldbl*(1 - AP.retraction);
	hipPRPos = [-hx -hy Ldbl] + QuatRotVec(angleZRot, [hx hy 0]);
	anklePos = hipPRPos + QuatRotVecPureZ(legAxisRot, -legAxisLen)';

	% Construct the output inverse pose
	IP = InversePose(anklePos, footRot);

	% Calculate the knee position if required
	if nargout >= 3
		w = legAxisRot(1);
		x = legAxisRot(2);
		y = legAxisRot(3);
		z = legAxisRot(4);
		perpKneeDist = L*sqrt(coerceMin(AP.retraction*(2 - AP.retraction), 0.0));
		kneePos = 0.5*(hipPRPos + anklePos) + perpKneeDist*[1-2*(y*y+z*z) 2*(x*y+z*w) 2*(x*z-y*w)];
	end

end

% Calculate quaternions from successive Z, X and Y rotations
function [QuatZ, QuatZXY] = QuatFromZXY(angZ, angX, angY)

	hangZ = 0.5*angZ;
	hangX = 0.5*angX;
	hangY = 0.5*angY;

	cz = cos(hangZ);
	sz = sin(hangZ);
	cx = cos(hangX);
	sx = sin(hangX);
	cy = cos(hangY);
	sy = sin(hangY);

	czcx = cz*cx;
	czsx = cz*sx;
	szcx = sz*cx;
	szsx = sz*sx;

	QuatZ = [cz 0 0 sz];
	QuatZXY = [czcx*cy-szsx*sy czsx*cy-szcx*sy czcx*sy+cy*szsx szcx*cy+czsx*sy];

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