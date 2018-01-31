% Calculate an arm joint pose that places the arm CoM on a specified line through the shoulder point
%
% function [AJP, CoM] = ArmJointFromCoM(CoMLine, alphaNom, shoulderRollLim)
%
% The value of alpha used is nominally alphaNom, but can be reduced down to zero if required
% to ensure kinematic feasibility and sensitivity-avoidance.
%
% CoMLine:  Non-zero vector specifying the direction of the line on which the CoM should lie
%           (relative to shoulder point in body-fixed coords)
% alphaNom: Nominal alpha value to use if it does not affect feasibility
% shoulderRollLim: Maximum shoulder roll and corresponding soft coercion buffer zone size
%                  i.e. [shoulderRollMax shoulderRollMaxBuf]
% AJP:      Output arm joint pose
% CoM:      Output actual location of the placed CoM (relative to shoulder point, in units of
%           L = armLinkLength)
%
function [AJP, CoM] = ArmJointFromCoM(CoMLine, alphaNom, shoulderRollLim)

	% Input arguments
	if nargin < 2
		alphaNom = acos(1 - 0.09);
	end
	if nargin < 3
		shoulderRollLim = [1.5 0.2];
	end

	% Constants
	Tol = 64*eps;

	% Make the CoMLine a unit vector
	CoMLineNorm = norm(CoMLine);
	if CoMLineNorm >= Tol
		CoMLine = CoMLine(:)' / CoMLineNorm;
	else
		warning('CoMLine must be a non-zero vector!');
		CoMLine = [0 0 -1];
	end

	% Coerce the nominal value of alpha
	alphaNom = coerce(alphaNom, 0.0, pi/2);

	% Establish the soft bound on the sin of the shoulder roll
	phimax = coerce(shoulderRollLim(1), Tol, pi/2);
	phibuf = coerce(shoulderRollLim(2), 0.0, phimax);
	abssphimax = sin(phimax);
	abssphibuf = coerce(sin(phimax) - sin(phimax - phibuf), sin(Tol), abssphimax);

	% Calculate the maximum allowed value of alpha, motivated by kinematics and feasibility
	cdalphanom = cos(2*alphaNom);
	pznom = -(3 + cdalphanom) / sqrt(10 + 6*cdalphanom);
	abssphinom = abs(-CoMLine(2) / pznom);
	if abssphinom > abssphimax - abssphibuf % Need to adjust alpha...
		abssphi = coerceSoftMax(abssphinom, abssphimax, abssphibuf); % Numerically abssphi > 0
		K = abs(CoMLine(2) / abssphi); % We wish to solve for abs(pz) >= K
		discr = 9*K*K - 8;
		if discr >= 0
			calphaMax = 3*(K*K - 1) + K*sqrt(discr);
		else
			calphaMax = -1/3;
		end
		alpha = 0.5*acos(coerceAbs(calphaMax, 1.0)); % The coercion in this line deals with K > 1
	else
		alpha = alphaNom;
	end

	% Pre-calculate dependent alpha terms
	dalpha = 2*alpha;
	cdalpha = cos(dalpha);
	sdalpha = sin(dalpha);
	pnorm = sqrt(10 + 6*cdalpha); % Always strictly positive (non-zero) and well-defined
	px = sdalpha/pnorm;
	pz = -(3 + cdalpha)/pnorm; % Always strictly negative (non-zero)

	% Calculate the required elbow pitch
	elbowPitch = -dalpha;

	% Calculate the required shoulder roll
	sphi = coerce(-CoMLine(2) / pz, -1.0, 1.0); % Note that pz is non-zero, and that the coercion is analytically not required because alphaMax should guarantee that abs(CoMLine(2)/pz) <= 1
	shoulderRoll = asin(sphi); % We take the solution to shoulder roll in the range [-pi/2,pi/2], instead of pi - shoulderRoll, to ensure the elbow is pointing in the correct logical direction

	% Calculate the required shoulder pitch
	cphi = cos(shoulderRoll);
	if abs(px) < Tol && abs(cphi) < Tol % Note that pz ~ -1 if px ~ 0
		shoulderPitch = 0;
	elseif all(abs(CoMLine([1 3])) < Tol) % Condition that CoMLine = [0 +-1 0] 
		shoulderPitch = 0;
	else
		shoulderPitch = picutMax(atan2(CoMLine(1), CoMLine(3)) - atan2(px, cphi*pz), 3*pi/4);
	end

	% Construct the output joint angles
	AJP = ArmJointPose([shoulderRoll shoulderPitch], elbowPitch);

	% Calculate the output CoM position
	if nargout >= 2
		CoM = 0.25*pnorm*CoMLine;
	end

end
% EOF