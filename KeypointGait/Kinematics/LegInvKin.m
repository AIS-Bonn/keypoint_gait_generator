% Leg inverse kinematics (conversion from inverse space to joint and abstract space)
%
% function [JP, AP, exact] = LegInvKin(IP, limbSign, RM)
%
function [JP, AP, exact] = LegInvKin(IP, limbSign, RM)

	% Input arguments
	if nargin < 3
		RM = RobotModel;
	end

	% Constants
	Tol = 1024*eps;
	CostTol = 64*eps;

	% Ensure that the foot rotation is a unit quaternion
	footRotNorm = norm(IP.footRot);
	if footRotNorm <= 0
		IP.footRot = QuatIdentity;
	else
		IP.footRot = IP.footRot / footRotNorm;
	end

	% Calculate the foot orientation rotation matrix
	RF = RotmatFromQuat(IP.footRot);

	% Some aliases for the foot orientation rotation matrix entries
	xFx = RF(1,1);
	xFy = RF(2,1);
	xFz = RF(3,1);

	% Robot dimensions
	L = RM.legLinkLength;
	Ldbl = 2*L;
	hx = RM.hipOffsetX;
	hy = limbSign*RM.hipOffsetY;

	% Calculate the position of the ankle point relative to the hip point
	PA = [IP.anklePos(1)+hx IP.anklePos(2)+hy IP.anklePos(3)-Ldbl];

	% Calculate the singularity discriminating factor A
	B = xFy*PA(3) - xFz*PA(2);
	C = xFx*PA(3) - xFz*PA(1);
	A = sqrt(B*B + C*C);

	% Calculate the required abstract space angles
	if A < Tol
		
		% Calculate the two possible solutions
		[APL(1), APL(2), exact] = CalcAbsPose(0, RF, PA, hx, hy, Ldbl); % Every phiz is possible, so arbitrarily choose zero. Local sensitivity is extremely high anyway, so there's no point trying to optimise more than this...
		
		% Calculate the joint pose and costs of each of the solutions
		cost = inf(2,2);
		for k = 2:-1:1
			JPL(k) = JointFromAbs(APL(k));
			[cost(k,1), cost(k,2)] = CalcJointCost(JPL(k));
		end

		% Find the solution with the lowest cost
		if abs(cost(1,1) - cost(2,1)) <= CostTol
			if cost(1,2) <= cost(2,2)
				I = 1;
			else
				I = 2;
			end
		elseif cost(1,1) <= cost(2,1)
			I = 1;
		else
			I = 2;
		end

		% Fill in the output variables
		AP = APL(I);
		JP = JPL(I);
		
	else

		% Calculate the four possible solutions
		xi = atan2(B, C);
		D = asin(coerceAbs(xFz*hy/A, 1.0));
		phizA = picut(xi + D);
		phizB = picut(xi + pi - D);
		[APL(1), APL(2), exactA] = CalcAbsPose(phizA, RF, PA, hx, hy, Ldbl); % Note: We rather choose a reasonable inexact pose, than a crazy exact pose,
		[APL(3), APL(4), exactB] = CalcAbsPose(phizB, RF, PA, hx, hy, Ldbl); % so exact being false should NOT be used to disqualify any of the solutions!

		% Calculate the joint pose and costs of each of the solutions
		cost = inf(4,2);
		for k = 4:-1:1
			JPL(k) = JointFromAbs(APL(k));
			[cost(k,1), cost(k,2)] = CalcJointCost(JPL(k));
		end

		% Find the solution with the lowest cost
		if abs(cost(1,1) - cost(2,1)) <= CostTol
			if cost(1,2) <= cost(2,2)
				bestAI = 1;
			else
				bestAI = 2;
			end
		elseif cost(1,1) <= cost(2,1)
			bestAI = 1;
		else
			bestAI = 2;
		end
		if abs(cost(3,1) - cost(4,1)) <= CostTol
			if cost(3,2) <= cost(4,2)
				bestBI = 3;
			else
				bestBI = 4;
			end
		elseif cost(3,1) <= cost(4,1)
			bestBI = 3;
		else
			bestBI = 4;
		end
		if abs(cost(bestAI,1) - cost(bestBI,1)) <= CostTol
			if cost(bestAI,2) <= cost(bestBI,2)
				I = bestAI;
			else
				I = bestBI;
			end
		elseif cost(bestAI,1) <= cost(bestBI,1)
			I = bestAI;
		else
			I = bestBI;
		end

		% Fill in the output variables
		AP = APL(I);
		JP = JPL(I);
		if I <= 2
			exact = exactA;
		else
			exact = exactB;
		end

	end

end

% Calculate the solution abstract pose corresponding to a given value of phiz
function [AP1, AP2, exact] = CalcAbsPose(phiz, RF, PA, hx, hy, Ldbl)

	% Initialise the exact variable
	exact = true;

	% Aliases for the foot orientation rotation matrix entries
	xFx = RF(1,1);
	xFy = RF(2,1);
	xFz = RF(3,1);
	yFx = RF(1,2);
	yFy = RF(2,2);
	yFz = RF(3,2);
	zFx = RF(1,3);
	zFy = RF(2,3);
	zFz = RF(3,3);

	% Precalculate trigonometric values
	cz = cos(phiz);
	sz = sin(phiz);

	% Precalculate terms of phiz
	fphiz = hx - PA(1)*cz - PA(2)*sz;
	gphiz = PA(2)*cz - PA(1)*sz - hy;

	% Calculate phix
	phix = atan2(gphiz, -PA(3));
	cx = cos(phix);
	sx = sin(phix);

	% Calculate phiy
	if abs(cx) >= abs(sx)
		phiy = atan2(cx*fphiz, -PA(3));
		if cx < 0
			phiy = picut(phiy + pi);
		end
	else
		phiy = atan2(sx*fphiz, gphiz);
		if sx < 0
			phiy = picut(phiy + pi);
		end
	end

	% Calculate phify
	phify = atan2(xFy*cz*sx - xFz*cx - xFx*sz*sx, xFx*cz + xFy*sz);

	% Calculate phifx
	czcx = cz*cx;
	szcx = sz*cx;
	phifx = picut(phix + atan2(zFx*szcx - zFy*czcx - zFz*sx, yFz*sx + yFy*czcx - yFx*szcx));

	% Calculate ret
	lambdasq = coerceMin(PA(1)*PA(1) + PA(2)*PA(2) + PA(3)*PA(3) + hx*hx + hy*hy - 2*(PA(1)*(hx*cz - hy*sz) + PA(2)*(hx*sz + hy*cz)), 0.0); % This should be trimming eps only...
	ret = 1 - sqrt(lambdasq)/Ldbl;
	if ret < 0.0
		ret = 0.0;
		exact = false;
	end

	% Construct the output abstract pose
	AP1 = AbstractPose([phix phiy phiz], [phifx phify], ret);
	AP2 = AbstractPose([picut(phix+pi) picut(pi-phiy) phiz], [phifx picut(-phify)], ret);

end
% EOF