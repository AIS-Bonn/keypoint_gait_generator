% CanonPose.m - Philipp Allgeuer - 22/03/17
% Convert a joint or abstract pose to its canonical joint and abstract forms.
%
% function [JP, AP, JPL, APL, I, canonExact, exact] = CanonPose(P, limbSign, RM)
%
% P can be a joint or abstract pose.
% This function corrects for angle wrapping, as well as angleX/angleY 2-equivalence.
% If P is an abstract pose, its retraction is coerced to [0,1] before further processing.
%
% I returns the index of JP/AP in JPL/APL.
% canonExact returns whether JP/AP is exact with respect to P.
% exact returns whether JPL(3:4)/APL(3:4) are exact with respect to P.
%
% Note that CanonPose(CanonPose(P)) can give a different answer to just CanonPose(P)
% if the latter has canonExact = false! In any case, calculating JPcanon = CanonPose(CanonPose(P))
% guarantees the property that JPcanon = CanonPose(JPcanon), even if JPcanon is a different pose
% (including in the inverse space) from the original P if CanonPose(P) was inexact.
%
function [JP, AP, JPL, APL, I, canonExact, exact] = CanonPose(P, limbSign, RM)

	% Input arguments
	if nargin < 3
		RM = RobotModel;
	end

	% Construct the base primary solution
	if isfield(P, 'retraction')
		APL(1) = PicutAbs(P, true); % P.retraction is coerced to [0,1] here...
		JPL(1) = JointFromAbs(APL(1));
	else
		JPL(1) = PicutJoint(P);
		APL(1) = AbsFromJoint(JPL(1));
	end

	% Construct the base secondary solution
	APL(2) = CalcSecondarySoln(APL(1));
	JPL(2) = JointFromAbs(APL(2));

	% Construct the alternative primary solution
	[APL(3), exact] = CalcAlternativeSoln(APL(1), limbSign, RM);
	JPL(3) = JointFromAbs(APL(3));

	% Construct the alternative secondary solution
	APL(4) = CalcSecondarySoln(APL(3));
	JPL(4) = JointFromAbs(APL(4));

	% Calculate the costs of all the solutions
	cost = inf(4,2);
	for k = 4:-1:1
		[cost(k,1), cost(k,2)] = CalcJointCost(JPL(k));
	end

	% Find the solution with the lowest cost
	costTol = 64*eps;
	if abs(cost(1,1) - cost(2,1)) <= costTol
		if cost(1,2) <= cost(2,2)
			bestBaseI = 1;
		else
			bestBaseI = 2;
		end
	elseif cost(1,1) <= cost(2,1)
		bestBaseI = 1;
	else
		bestBaseI = 2;
	end
	if abs(cost(3,1) - cost(4,1)) <= costTol
		if cost(3,2) <= cost(4,2)
			bestAltI = 3;
		else
			bestAltI = 4;
		end
	elseif cost(3,1) <= cost(4,1)
		bestAltI = 3;
	else
		bestAltI = 4;
	end
	if abs(cost(bestBaseI,1) - cost(bestAltI,1)) <= costTol
		if cost(bestBaseI,2) <= cost(bestAltI,2)
			I = bestBaseI;
		else
			I = bestAltI;
		end
	elseif cost(bestBaseI,1) <= cost(bestAltI,1)
		I = bestBaseI;
	else
		I = bestAltI;
	end

	% Fill in the output variables
	AP = APL(I);
	JP = JPL(I);
	if I <= 2
		canonExact = true;
	else
		canonExact = exact;
	end

end

% Calculate the secondary solution for a particular primary solution
function [APS] = CalcSecondarySoln(AP)

	% Construct the secondary solution as required
	APS = AbstractPose([picut(AP.angleX + pi) picut(pi - AP.angleY) AP.angleZ], [AP.footAngleX picut(-AP.footAngleY)], AP.retraction);

end

% Calculate the alternative solution for a particular base solution
function [APA, exact] = CalcAlternativeSoln(AP, limbSign, RM)

	% Initialise the exact variable
	exact = true;

	% Normalise the problem dimensions
	Ldbl = 2*RM.legLinkLength;
	hxn = RM.hipOffsetX / Ldbl;
	hyn = limbSign*RM.hipOffsetY / Ldbl;
	lamn = 1 - AP.retraction;

	% Precalculate trigonometric terms
	phiax = AP.footAngleX - AP.angleX;
	phiay = AP.footAngleY - AP.angleY;
	cx = cos(AP.angleX);
	sx = sin(AP.angleX);
	cy = cos(AP.angleY);
	sy = sin(AP.angleY);
	cfy = cos(AP.footAngleY);
	sfy = sin(AP.footAngleY);
	cax = cos(phiax);
	sax = sin(phiax);
	cay = cos(phiay);

	% Calculate the alternative phiz
	dphiz = 2*atan2(hyn*sfy, hxn*sfy - lamn*cay) + pi;
	phizt = picut(AP.angleZ + dphiz);
	cdz = cos(dphiz);
	sdz = sin(dphiz);
	Cdz = 1.0 - cdz;

	% Precalculate derived trigonometric terms
	sxcy = sx*cy;
	cfycx = cfy*cx;
	cfysx = cfy*sx;
	sfysx = sfy*sx;
	sfysdz = sfy*sdz;

	% Precalculate terms
	fphizt = lamn*(sy*cdz - sxcy*sdz) + hxn*Cdz - hyn*sdz;
	gphizt = lamn*(sy*sdz + sxcy*cdz) - hyn*Cdz - hxn*sdz;
	hphizt = lamn*cx*cy;

	% Calculate the alternative phix
	phixt = atan2(gphizt, hphizt);
	cxt = cos(phixt);
	sxt = sin(phixt);

	% Calculate the alternative phiy
	if abs(cxt) >= abs(sxt)
		phiyt = atan2(cxt*fphizt, hphizt);
		if cxt < 0
			phiyt = picut(phiyt + pi);
		end
	else
		phiyt = atan2(sxt*fphizt, gphizt);
		if sxt < 0
			phiyt = picut(phiyt + pi);
		end
	end

	% Calculate the alternative phify
	phifyt = atan2(cxt*(sfy*cx) + sxt*(sfysx*cdz - cfy*sdz), cfy*cdz + sfysx*sdz);

	% Calculate the alternative phifx
	phiaxt = atan2(cxt*(cdz*(sax*cx + cax*cfysx) + cax*sfysdz) + sxt*(sax*sx - cax*cfycx), ...
	               cxt*(cdz*(cax*cx - sax*cfysx) - sax*sfysdz) + sxt*(cax*sx + sax*cfycx));
	phifxt = picut(phiaxt + phixt);

	% Calculate the alternative ret
	lamntsq = coerceMin(lamn*lamn + 2*Cdz*(hxn*hxn + hyn*hyn + lamn*(hyn*sxcy - hxn*sy)) - 2*lamn*sdz*(hxn*sxcy + hyn*sy), 0.0); % This should be trimming eps only...
	rett = 1 - sqrt(lamntsq);
	if rett < 0.0
		rett = 0.0;
		exact = false;
	end

	% Construct the alternative solution
	APA = AbstractPose([phixt phiyt phizt], [phifxt phifyt], rett);

end
% EOF