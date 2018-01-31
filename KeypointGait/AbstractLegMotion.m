% AbstractLegMotion.m - Philipp Allgeuer - 01/09/16
% A subset of the abstract leg motion for walking.
% This function is NOT vectorised.
%
% function [AP] = AbstractLegMotion(AP, gcv, gaitPhase, limbSign, config)
%
function [AP] = AbstractLegMotion(AP, gcv, gaitPhase, limbSign, config)

	%%
	% Function arguments
	%

	% Default variables
	if nargin < 5
		config = ConfigVars;
	end
	limbSign = sgn(limbSign); % +1 = Left leg, -1 = Right leg
	leftLegFirst = true; % Note: The left leg is currently always first...

	%%
	% Common motion data
	%

	% Common motion data
	CMD = struct();

	% Gait command velocity
	CMD.gcvX = gcv(1);
	CMD.gcvY = gcv(2);
	CMD.gcvZ = gcv(3);
	CMD.absGcvX = abs(gcv(1));
	CMD.absGcvY = abs(gcv(2));
	CMD.absGcvZ = abs(gcv(3));

	% Gait phase
	CMD.gaitPhase = picut(gaitPhase);
	CMD.oppGaitPhase = picut(CMD.gaitPhase + pi);
	if leftLegFirst == (limbSign >= 0) % If this limb is first...
		CMD.limbPhase = CMD.gaitPhase;
	else
		CMD.limbPhase = CMD.oppGaitPhase;
	end
	if leftLegFirst % If the left leg is first...
		CMD.absPhase = CMD.gaitPhase;
	else
		CMD.absPhase = CMD.oppGaitPhase;
	end

	% Gait phase marks
	CMD.doubleSupportPhase = config.doubleSupportPhaseLen;
	CMD.swingStartPhase = CMD.doubleSupportPhase + config.swingStartPhaseOffset; % Limb phase mark at which the swing starts, in the range [0,pi]
	CMD.swingStopPhase = pi - config.swingStopPhaseOffset;
	CMD.suppTransStartPhase = -config.suppTransStartRatio * (pi - CMD.swingStopPhase);                                            % Phase mark at which the support transition starts (small negative number, i.e. backwards from zero/pi, the ratio interpolates between the beginning of the support phase (zero) and the first negative swing stop phase)
	CMD.suppTransStopPhase = CMD.doubleSupportPhase + config.suppTransStopRatio * (CMD.swingStartPhase - CMD.doubleSupportPhase); % Phase mark at which the support transition stops, in the range [0,pi] (the ratio interpolates between the end of the support phase and the first positive swing start phase)

	% Check for swing phase time violation
	if CMD.swingStopPhase - CMD.swingStartPhase < config.swingMinPhaseLen
		error('Swing phase time violation!');
	end

	% Calculate the extra swing variables
	CMD.liftingPhaseLen = pi - CMD.doubleSupportPhase;                        % Length of each lifting (single support) phase
	CMD.suppPhaseLen = pi - CMD.suppTransStartPhase + CMD.suppTransStopPhase; % Length of the support phase (phase length for which a support coefficient is non-zero at a time)
	CMD.nonSuppPhaseLen = 2*pi - CMD.suppPhaseLen;                            % Length of the non-support phase (phase length for which a support coefficient is zero at a time)
	CMD.sinusoidPhaseLen = CMD.swingStopPhase - CMD.swingStartPhase;          % Length of the sinusoidal forwards swing phase
	CMD.linearPhaseLen = 2*pi - CMD.sinusoidPhaseLen;                         % Length of the linear backwards swing phase

	% Calculate the gait phase dependent dimensionless swing angle (ranging from -1 to 1)
	if CMD.limbPhase >= CMD.swingStartPhase && CMD.limbPhase <= CMD.swingStopPhase
		CMD.swingAngle = -cos(pi * (CMD.limbPhase - CMD.swingStartPhase) / CMD.sinusoidPhaseLen);        % Sinusoid forwards swing from dimensionless angle -1 to +1 (section from phase swingStartPhase to phase swingStopPhase)
	elseif CMD.limbPhase > CMD.swingStopPhase
		CMD.swingAngle = 1.0 - (2.0 / CMD.linearPhaseLen) * (CMD.limbPhase - CMD.swingStopPhase);        % Linear backwards swing from dimensionless angle +1 to a mid-way point C (section from phase swingStopPhase to phase pi)
	else
		CMD.swingAngle = 1.0 - (2.0 / CMD.linearPhaseLen) * (CMD.limbPhase - CMD.swingStopPhase + 2*pi); % Linear backwards swing from dimensionless angle C to -1 (section from phase -pi to phase swingStartPhase)
	end

	%%
	% Leg lifting (retraction, angleY)
	%

	% The leg is alternately lifted off the ground (step) and pushed down into it (push), with a short break in-between given by the double
	% support phase. The support phases start at limb phase 0 and pi, and the leg lifting/pushing phases start immediately after the
	% support phases end, respectively. The lifting/pushing phases always end at limb phases of pi and 0, respectively. The leg push
	% phase, if enabled, assists with achieving foot clearance of the swing foot.

	% Apply leg stepping and pushing to the abstract leg pose
	if ~config.tuningNoLegLifting

		% Calculate the desired step and push height from the current gcv
		stepHeight = config.legStepHeight + CMD.absGcvX*config.legStepHeightGradX + CMD.absGcvY*config.legStepHeightGradY;
		pushHeight = config.legPushHeight + CMD.absGcvX*config.legPushHeightGradX;

		% Precalculate parameters and phases for the following calculation
		sinAngFreq = pi / CMD.liftingPhaseLen;
		stepStartPhase = CMD.limbPhase - CMD.doubleSupportPhase;
		stepStopPhase  = -picut(CMD.limbPhase - pi);
		pushStartPhase = stepStartPhase + pi;
		pushStopPhase  = -CMD.limbPhase;

		% Calculate the basic sinusoid step and push waveforms
		if stepStartPhase >= 0.0
			legRetractionOffset =  stepHeight * sin(sinAngFreq * stepStartPhase); % Leg stepping phase => Limb phase in the positive half
		elseif pushStartPhase >= 0.0 && CMD.limbPhase <= 0.0
			legRetractionOffset = -pushHeight * sin(sinAngFreq * pushStartPhase); % Leg pushing phase => Limb phase in the negative half
		else
			legRetractionOffset = 0.0;                                            % Double support phase
		end

		% Add fillets to the waveforms to avoid overly large acceleration/torque jumps
		legRetractionOffset = legRetractionOffset + LinSinEval(stepStartPhase,  stepHeight, sinAngFreq, 0.5*config.filletStepPhaseLen, config.doubleSupportPhaseLen);
		legRetractionOffset = legRetractionOffset + LinSinEval(stepStopPhase,   stepHeight, sinAngFreq, 0.5*config.filletStepPhaseLen, config.doubleSupportPhaseLen);
		legRetractionOffset = legRetractionOffset + LinSinEval(pushStartPhase, -pushHeight, sinAngFreq, 0.5*config.filletPushPhaseLen, config.doubleSupportPhaseLen);
		legRetractionOffset = legRetractionOffset + LinSinEval(pushStopPhase,  -pushHeight, sinAngFreq, 0.5*config.filletPushPhaseLen, config.doubleSupportPhaseLen);

		% Update the leg retraction
		AP.retraction = AP.retraction + legRetractionOffset;
		AP.angleY = AP.angleY + config.legExtToAngleYGain * legRetractionOffset; % This trims the lift angle of the feet, which can be used to trim walking on the spot, but this also has very positive effects on the remainder of OL walking

	end

	%%
	% Sagittal leg swing (angleY)
	%

	% Apply the sagittal leg swing to the abstract leg pose (responsible for fulfilling the gcv x-velocity)
	% We use a different sagittal leg swing gradient depending on whether we're walking forwards or backwards.
	if CMD.gcvX >= 0.0
		legSagSwingMagGradX = config.legSagSwingMagGradXFwd;
	else
		legSagSwingMagGradX = config.legSagSwingMagGradXBwd;
	end
	legSagSwingMag = CMD.gcvX * legSagSwingMagGradX;
	legSagSwing = -CMD.swingAngle * legSagSwingMag;
	AP.angleY = AP.angleY + legSagSwing;

	%%
	% Lateral leg pushout, lateral leg swing and lateral hip swing (angleX)
	%

	% Apply the lateral leg pushout to the abstract leg pose
	% This rolls the legs outwards (from the hip) in proportion to each of the absolute gcv values. This acts to separate the feet
	% more the faster the robot is walking, and the more the robot is trying to walk with combined velocity components (e.g. forwards
	% velocity coupled with a rotational velocity). This term seeks to prevent foot to foot self-collisions, and should more be
	% seen as a bias to the halt pose, as opposed to a dynamic motion component of the gait. This is because for constant walking
	% velocity command, this term is constant.
	legLatPushoutMag = CMD.absGcvX*config.legLatPushoutMagGradX + CMD.absGcvY*config.legLatPushoutMagGradY + CMD.absGcvZ*config.legLatPushoutMagGradZ;
	if ~config.tuningNoLegPushout
		AP.angleX = AP.angleX + limbSign * legLatPushoutMag;
	end

	% Apply the lateral leg swing to the abstract leg pose
	% This is responsible for fulfilling the gcv y-velocity.
	legLatSwingMag = CMD.gcvY * config.legLatSwingMagGradY;
	legLatSwing = CMD.swingAngle * legLatSwingMag;
	AP.angleX = AP.angleX + legLatSwing;

	% The lateral hip swing motion component sways the hips left and right relative to the feet. This is achieved via direct addition
	% of a hip swing term to the hip roll (i.e. angleX). Hip swing to the left occurs when the left foot is in its support phase
	% (i.e. absolute phase in (-pi,0]), and should correspond to negative roll. Hip swing to the right occurs when the right foot is
	% in its support phase (i.e. absolute phase in (0,pi]), and should correspond to positive roll. The hip swing is applied equally
	% to both hip roll joints, and is calculated as the sum of two overlapping sinusoid 'halves', one for the hip swing to the left,
	% and one for the hip swing to the right. The overlap between the sinusoids occurs during the support transition phase, during which
	% time the sinusoids sum up, and the hip travels from one body side to the other. To be totally explicit, the left hip swing
	% sinusoid completes exactly half a sinusoid from the beginning of the left leg support transition to the end of the next right
	% leg support transition, and is zero everywhere else. The right hip swing sinusoid behaves similarly, and they are added up to
	% get the full dimensionless hip swing waveform. Recall that the absolute phase is in phase or antiphase to the gait phase such
	% that the swing phase of the left leg is in [0,pi].

	% Perform phase calculations for the left and right halves (sinusoids) of the hip swing motion
	hipSwingStartL = CMD.suppTransStartPhase + pi; % The start phase of the period of support of the left leg
	hipSwingStartR = CMD.suppTransStartPhase;      % The start phase of the period of support of the right leg
	if CMD.absPhase < hipSwingStartL
		hipSwingPhaseL = CMD.absPhase + 2*pi - hipSwingStartL; % The current phase relative to the start of the transition to the left leg as the support leg, in the range [0,2*pi]
	else
		hipSwingPhaseL = CMD.absPhase - hipSwingStartL;
	end
	if CMD.absPhase < hipSwingStartR
		hipSwingPhaseR = CMD.absPhase + 2*pi - hipSwingStartR; % The current phase relative to the start of the transition to the right leg as the support leg, in the range [0,2*pi]
	else
		hipSwingPhaseR = CMD.absPhase - hipSwingStartR;
	end

	% Calculate the dimensionless hip swing angle (range -1 to 1) by summing up the two zero-coerced sinusoid sub-waveforms
	hipSwingAngleL = -coerceMin(sin(pi * hipSwingPhaseL / CMD.suppPhaseLen), 0.0);
	hipSwingAngleR =  coerceMin(sin(pi * hipSwingPhaseR / CMD.suppPhaseLen), 0.0);
	hipSwingAngle  =  hipSwingAngleL + hipSwingAngleR; % The hip swing angle is in the range -1 to 1

	% Apply the lateral hip swing to the abstract leg pose
	legLatHipSwingMag = config.legLatHipSwingMag + CMD.absGcvX*config.legLatHipSwingMagGradX + CMD.absGcvY*config.legLatHipSwingMagGradY;
	legLatHipSwing = config.legLatHipSwingBias + legLatHipSwingMag * hipSwingAngle;
	if ~config.tuningNoLegHipSwing
		AP.angleX = AP.angleX + legLatHipSwing;
	end

	%%
	% Rotational leg V pushout and rotational leg swing (angleZ)
	%

	% Apply the rotational leg V pushout to the abstract leg pose
	% Offset the yaw of the feet to be in a tendentially more toe-out configuration the faster we are expecting the robot to turn.
	% This is referred to as rotational V pushout, and has the effect of attempting to avoid toe to toe self-collisions. For a constant
	% walking velocity command, this term is constant, and hence should be considered to be more of a bias to the halt pose, rather
	% than a dynamic motion component of the gait.
	legRotVPushoutMag = CMD.absGcvZ * config.legRotVPushoutMagGradZ;
	if ~config.tuningNoLegPushout
		AP.angleZ = AP.angleZ + limbSign * legRotVPushoutMag;
	end

	% Apply the rotational leg swing to the abstract leg pose
	% This is responsible for fulfilling the gcv z-velocity.
	legRotSwingMag = CMD.gcvZ * config.legRotSwingMagGradZ;
	legRotSwing = CMD.swingAngle * legRotSwingMag;
	AP.angleZ = AP.angleZ + legRotSwing;

	%% Not implemented (before implementing these, check all uses of GenLegMotion, as some assume these features are not present)
	% - Leaning (hipAngleX, hipAngleY)
	% - Basic feedback mechanisms (hipAngleX, hipAngleY, footAngleX, footAngleY)
	% - Hip angle (angleX, angleY, footAngleX, footAngleY, retraction)
	% - Support coefficients

end
% EOF