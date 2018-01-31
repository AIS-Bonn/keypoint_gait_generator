% CalcHipYaw.m - Philipp Allgeuer - 04/04/17
%
% function [hipYaw, hipYawAlt] = CalcHipYaw(IP, limbSign, RM)
%
% In general there are two inverse kinematics solutions for the hip yaw.
% hipYaw is the one with the smaller absolute value, and hipYawAlt is then
% the alternative solution.
%
function [hipYaw, hipYawAlt] = CalcHipYaw(IP, limbSign, RM)

	% Input arguments
	if nargin < 3
		RM = RobotModel;
	end

	% Constants
	Tol = 1024*eps;

	% Ensure that the foot rotation is a unit quaternion
	footRotNorm = norm(IP.footRot);
	if footRotNorm <= 0
		IP.footRot = QuatIdentity;
	else
		IP.footRot = IP.footRot / footRotNorm;
	end

	% Aliases
	w = IP.footRot(1);
	x = IP.footRot(2);
	y = IP.footRot(3);
	z = IP.footRot(4);

	% Calculate the foot x-axis unit vector
	xFx = 1 - 2*(y*y + z*z);
	xFy = 2*(x*y + z*w);
	xFz = 2*(x*z - y*w);

	% Robot dimensions
	Ldbl = 2*RM.legLinkLength;
	hx = RM.hipOffsetX;
	hy = limbSign*RM.hipOffsetY;

	% Calculate the position of the ankle point relative to the hip point
	PA = [IP.anklePos(1)+hx IP.anklePos(2)+hy IP.anklePos(3)-Ldbl];

	% Calculate the singularity discriminating factor A
	B = xFy*PA(3) - xFz*PA(2);
	C = xFx*PA(3) - xFz*PA(1);
	A = sqrt(B*B + C*C);

	% Calculate the required hip yaw
	if A < Tol
		hipYaw = 0;     % Every hip yaw is possible, so arbitrarily choose zero => Local sensitivity is extremely high anyway, so there's no point trying to optimise more than this...
		hipYawAlt = pi; % Arbitrary second choice for hip yaw...
	else
		xi = atan2(B, C);
		D = asin(coerceAbs(xFz*hy/A, 1.0));
		hipYawA = picut(xi + D);
		hipYawB = picut(xi + pi - D);
		if abs(hipYawA) <= abs(hipYawB)
			hipYaw = hipYawA;
			hipYawAlt = hipYawB;
		else
			hipYaw = hipYawB;
			hipYawAlt = hipYawA;
		end
	end

end
% EOF