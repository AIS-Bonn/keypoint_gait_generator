% CalcKeypointTraj.m - Philipp Allgeuer - 15/02/17
% Calculate the foot trajectories based on a keypoint method.
%
% function [out] = CalcKeypointTraj(in, runType, config, RM)
%
% in      ==> Struct of inputs to the open loop gait
% runType ==> 0 = No fig/test, 1 = Figures, 2 = Figures and test, 3 = Figures and verbose test
% config  ==> Configuration variables to use (see ConfigVars)
% RM      ==> Robot model to use (see RobotModel)
% out     ==> Struct of outputs
%
% The struct of inputs 'in' is expected to have the following fields:
% gcv:          Desired dimensionless walking velocity [gcvX gcvY gcvZ] in the x, y and yaw axes, relative to the
%               body-fixed axes untilted relative to the true ground (i.e. the true S plane)
% fusedPitchN:  Fused pitch of the body-fixed axes B relative to the nominal ground plane N
% fusedS:       Fused pitch and roll [pitch roll] of the body-fixed axes B relative to the swing ground plane S
% leanTilt:     Upper body leaning tilt [gamma alpha] relative to the nominal ground plane N
% hipShift:     Normalised xy cartesian shift (in units of 2L) of the robot's hips relative to the leaned ground plane
% hipHeightMax: Maximum normalised hip height to use (in units of F) relative to the leaned ground plane
% footTiltCts:  Continuous foot tilt [gamma alpha] relative to the nominal ground plane N (absolute or relative)
% footTiltSupp: Support foot tilt [gamma alpha] relative to the nominal ground plane N (always absolute)
% swingOut:     Peak swing leg tilt [gamma alpha] relative to the nominal ground plane N
%
% Hip height is defined as the height of the hip centre point over the motion centre point (FFP-related), measured
% perpendicular to the leaned ground plane (i.e. the nominal ground plane if there is no lean). Hip height is always
% normalised by the full length of the leg, i.e. F = 2L + footOffsetZ = 2*RM.legLinkLength + RM.footOffsetZ.
%
% Foot tilts can either be interpreted as absolute or relative. Absolute means that the gamma value is interpreted
% relative to the N plane, resulting in a constant global tilt axis irrespective of the foot yaw. Relative means
% that the tilt axis rotates with the foot yaw, so that the tilt axis stays constant relative to the foot. The
% config.footTiltIsLocalRatio variable is used to interpolate between using absolute (= 0) and relative (= 1) foot
% tilt interpretations for the nominal foot tilt and continuous foot tilt. The support foot tilt is always
% interpreted as absolute, regardless of config.footTiltIsLocalRatio.
%
% The coordinate system for all FFPs is x forwards, y left and z upwards, with the origin at [hx 0 -2L] relative to
% the hip centre point (the centre of the hip points, NOT the hip PR points). The coordinate system for all IPs is
% x forwards, y left and z upwards, with the origin at [hx limbSign*hy -2L] relative to the hip point.
%
function [out] = CalcKeypointTraj(in, runType, config, RM) %#ok<*AGROW>

	%% Notes
	% - Have enable/disable for all inputs/features/feedback mechanisms of the gait
	% - Allow enabling of virtual slope separately between walking forwards and walking backwards
	% - Condition number was important in the choice of doing velocity conversions between inverse and abstract space, as
	%   opposed to joint space.
	% - The inputs are not so much coerced by the open loop gait. It is the responsibility of the closed loop gait to ensure
	%   all inputs are always reasonable.
	% - Use smooth deadband and soft coercion on the fusedS input
	% - Gcv offsets, limiting, prescaling, etc are to be handled by the closed loop gait.
	% - Incorporate halt pose biases on a joint/abstract space level
	% - Would be interesting to generate a map of gcv to ideal step size and long term velocity (idealised, analytic, S = N).
	%   This map could be used to have step sizes, not gcvs, as the higher level input to the gait, correcting for gcv
	%   non-linearities in the execution of the gait.
	% - Basic timing and step size adjustment can be built by modifying the input gcv (need idealised gcv to step size map)
	%   and the rate of input gait phase progression.
	% - The output of the open loop gait is only continuous and smooth if ALL of its inputs are in a temporal fashion.
	% - Identify the exact reason that forwards and backwards walking are so different, and try to get it under control with
	%   tuning and the existing features of the open loop gait.

	%% Input arguments

	% Default input arguments
	if nargin < 1
		in = struct();
	end
	if isnumeric(in) && isvector(in) && numel(in) == 3
		in = struct('gcv', in(:)');
	end
	in = SetIfMissing(in, 'gcv', [2 -0.5 -0.5]);
	in = SetIfMissing(in, 'fusedPitchN', 0.08);
	in = SetIfMissing(in, 'fusedS', [0.15 0.05]);
	in = SetIfMissing(in, 'leanTilt', [1.0 0.10]);
	in = SetIfMissing(in, 'hipShift', [0.03 -0.02]);
	in = SetIfMissing(in, 'hipHeightMax', Inf);
	in = SetIfMissing(in, 'footTiltCts', [-0.7 0.02]);
	in = SetIfMissing(in, 'footTiltSupp', [0.5 0.1]);
	in = SetIfMissing(in, 'swingOut', [-0.2 0.1]);
	if nargin < 2
		runType = 2;
	end
	if nargin < 3
		config = ConfigVars;
	end
	if nargin < 4
		RM = RobotModel;
	end

	% Plotting variables
	fig = 101;
	nomLim = 0.2;
	lims = nomLim*[-0.75 0.75 -1.2 1.2 -0.5 0.75];
	txtOff = 0.008;
	fySize = 0.03;
	ftSize = 0.7;
	vlScale = 0.5;
	vaScale = 0.3;
	footScaleX = 0.02;
	footScaleY = 0.013;

	% Special test cases (uncomment as desired)
% 	in.fusedS = [in.fusedPitchN 0];
% 	config.IRatio = 1;
% 	config.JRatio = 1;

	%% Initialisation and setup

	% Calculate the relevant nominal ground plane rotations and axes
	qNB = QuatFromFused([0 in.fusedPitchN 0 1]);
	qBN = QuatInv(qNB);
	RNB = RotmatFromQuat(qNB);
	RBN = RNB';
	BzN = RNB(3,:);

	% Calculate the relevant swing ground plane rotations and axes
	BzS = ZVecFromFused([0 in.fusedS(1) in.fusedS(2) 1]);
	NzS = QuatRotVec(qNB, BzS);
	qSN = QuatFromZVec(NzS); % Must have qSN(4) = 0 as pure tilt rotation from N to S
	qNS = QuatInv(qSN); % Must have qNS(4) = 0 as pure tilt rotation from N to S
	RSN = RotmatFromQuat(qSN);
	RNS = RSN';
	qBS = QuatMult(qBN, qNS);
	RBS = RotmatFromQuat(qBS);

	% Swing ground plane orientation aliases
	BxS = RBS(:,1)';
	ByS = RBS(:,2)';
	xSx = RBS(1,1);
	xSy = RBS(2,1);
	xSz = RBS(3,1);
	ySx = RBS(1,2);
	ySy = RBS(2,2);
	ySz = RBS(3,2);
	zSx = RBS(1,3);
	zSy = RBS(2,3);
	zSz = RBS(3,3);

	% Common variables
	D = config.doubleSupportPhaseLen;
	L = RM.legLinkLength;
	Ldbl = 2.0*L;
	F = Ldbl + RM.footOffsetZ;
	H = 0.5*RM.hipWidth;
	Hvec = [0 H 0];
	hx = RM.hipOffsetX;
	hy = RM.hipOffsetY; % Note: Hip offset is hy for left leg, and -hy for right leg (i.e. always limbSign*hy)
	absFields = fieldnames(AbstractPose);
	LS = [2 1];

	% Common motion data
	CMD = struct();
	CMD.gcvX = in.gcv(1);
	CMD.gcvY = in.gcv(2);
	CMD.gcvZ = in.gcv(3);
	CMD.absGcvX = abs(in.gcv(1));
	CMD.absGcvY = abs(in.gcv(2));
	CMD.absGcvZ = abs(in.gcv(3));
	CMD.hipPos = [-hx -hy Ldbl]; % Hip point in ankle coordinate system (y coordinate needs to be scaled by limbSign)
	CMD.hipPosFFP = [-hx H Ldbl]; % Hip point in FFP coordinate system (y coordinate needs to be scaled by limbSign)
	CMD.hipCentrePos = [-hx 0 Ldbl]; % Hip centre point in FFP coordinate system
	CMD.ankleToFFP = [0 H+hy 0]; % Add to convert a point in the ankle coordinate system to the equivalent point in the FFP coordinate system (y coordinate needs to be scaled by limbSign)
	
	% Note: The C++ feed_gait code uses a global coordinate system centred at the hip centre point,
	% thus at an offset of [-hx 0 Ldbl] to the FFP coordinate system used in this file.

	%% Retrieve the GCV-dependent gait magnitudes

	% Retrieve the required pushout magnitudes for the given gcv
	legLatPushoutMag = CMD.absGcvX*config.legLatPushoutMagGradX + CMD.absGcvY*config.legLatPushoutMagGradY + CMD.absGcvZ*config.legLatPushoutMagGradZ;
	legRotVPushoutMag = CMD.absGcvZ*config.legRotVPushoutMagGradZ;

	% Retrieve the required leg swing magnitudes for the given gcv
	if CMD.gcvX >= 0.0
		legSagSwingMagGradX = config.legSagSwingMagGradXFwd;
	else
		legSagSwingMagGradX = config.legSagSwingMagGradXBwd;
	end
	legSagSwingMag = CMD.gcvX * legSagSwingMagGradX;
	legLatSwingMag = CMD.gcvY * config.legLatSwingMagGradY;
	legRotSwingMag = CMD.gcvZ * config.legRotSwingMagGradZ;

	% Retrieve the required hip swing magnitude for the given gcv
	legLatHipSwingMag = config.legLatHipSwingMag + CMD.absGcvX*config.legLatHipSwingMagGradX + CMD.absGcvY*config.legLatHipSwingMagGradY;

	% Retrieve the required step height for the given gcv
	legStepHeightDist = Ldbl*coerceMin(config.legStepHeight + CMD.absGcvX*config.legStepHeightGradX + CMD.absGcvY*config.legStepHeightGradY, 0.0);

	%% Calculate the motion profile centre points

	% Calculate the halt pose and the left and right motion centre poses
	for l = LS
		limbSign = LimbSign(l);
		APH(l) = AbstractHaltPose(limbSign, config);
		IPH(l) = InvFromAbs(APH(l), limbSign, RM);
		FFPH(l,:) = FootFloorPoint(IPH(l), limbSign, RM) + limbSign*CMD.ankleToFFP;
		APM(l) = APH(l);
		APM(l).angleX = APM(l).angleX + limbSign*legLatPushoutMag;
		APM(l).angleZ = APM(l).angleZ + limbSign*legRotVPushoutMag;
		IPM(l) = InvFromAbs(APM(l), limbSign, RM);
		FFPM(l,:) = FootFloorPoint(IPM(l), limbSign, RM) + limbSign*CMD.ankleToFFP;
	end

	% Calculate the total motion centre point
	lambda = interpolateCoerced([FFPM(1,2) FFPM(2,2)], [0.0 1.0], 0.0);
	MC = (1.0 - lambda)*FFPM(1,:) + lambda*FFPM(2,:);
	MC(2) = 0.0;

	% Calculate the unit vector corresponding to the motion centre line
	MCLhat = VecSlerp(BzN, CMD.hipCentrePos - MC, config.MCLHipCentreRatio); % Returns a normalised unit vector! Ratio = 0 => MCL is in the direction of the z-axes of N-plane, Ratio = 1 => MCL is towards the hip centre point from the MC

	%% Calculate the raw support keypoints (step size generator black box)

	% Define the number of keypoints
	NS = 5:-1:1;
	ENS = 8:-1:1;

	% Aliases for relevant configs
	hAG = config.GPhaseOffset;
	hDE = config.EPhaseOffset;

	% Define the raw support keypoints of interest
	keyRaw.limbPhase = [-pi D-pi 0.5*(D-pi) 0 D D+hDE 0.5*(D+pi) pi-hAG]; % ABNCDEFG in limb phase (i.e. the swing phase of the respective limb is in [0,pi])

	% Calculate the dimensionless swing values for the keypoints
	keyRaw.swing(6:8) = 0.0;
	for n = NS
		keyRaw.swing(n) = (D - pi - 2.0*keyRaw.limbPhase(n))/(D + pi); % 1 => Swing at touchdown (A), -1 => Swing at lift-off (D), Swing is positive in the direction of the corresponding positive GCV
	end

	% Calculate the dimensionless hip swing values for the keypoints
	dblSuppHipSwing = sin(pi*D/(pi+D));
	for l = LS
		limbSign = LimbSign(l);
		keyRaw.hipSwing(l,:) = limbSign * [dblSuppHipSwing -dblSuppHipSwing -1.0 -dblSuppHipSwing dblSuppHipSwing 1.0 1.0 1.0]; % 1 => Full hip swing to the right (feet to the left), -1 => Full hip swing to the left (feet to the right)
	end

	% Calculate the required raw support keypoints
	for l = LS
		limbSign = LimbSign(l);
		for n = ENS
			AP = APM(l);
			AP.angleX = AP.angleX + keyRaw.swing(n)*legLatSwingMag;
			AP.angleY = AP.angleY - keyRaw.swing(n)*legSagSwingMag;
			AP.angleZ = AP.angleZ + keyRaw.swing(n)*legRotSwingMag;
			AP.angleX = AP.angleX + keyRaw.hipSwing(l,n)*legLatHipSwingMag;
			keyRaw.AP(l,n) = AP;
			keyRaw.IP(l,n) = InvFromAbs(AP, limbSign, RM);
			keyRaw.FFP(l,:,n) = FootFloorPoint(keyRaw.IP(l,n), limbSign, RM) + limbSign*CMD.ankleToFFP;
		end
	end

	%% Calculate the projected support keypoints

	% Calculate the required projected support keypoints
	for l = LS
		keyProj.nomFootTiltAngles(l,:) = TiltFromQuat(IPH(l).footRot); % Note: The halt pose foot orientation relative to B is taken as the nominal foot orientation relative to N. It is important to realise that the gamma value here is NOT relative to N, but offset by the fused yaw.
		for n = ENS
			keyProj.FFP(l,:,n) = keyRaw.FFP(l,:,n);
			keyProj.FFP(l,3,n) = MC(3); % Project the keypoints into the xy-plane through the motion centre (this is interpreted as the desired step size within the N plane)
			keyProj.footYaw(l,n) = FYawOfQuat(keyRaw.IP(l,n).footRot); % Calculate the yaw component relative to B of the raw foot rotation (this is interpreted as the desired foot yaw relative to the N plane)
			keyProj.relFFP(l,:,n) = keyProj.FFP(l,:,n) - MC; % Position of the FFP relative to the motion centre
		end
	end

	%% Calculate the rotated support keypoints

	% Calculate the required rotated support keypoints
	for l = LS
		keyRot.footTiltNom(l,:) = [keyProj.nomFootTiltAngles(l,1) + keyProj.nomFootTiltAngles(l,2), keyProj.nomFootTiltAngles(l,3)]; % Calculate the absolute form of the nominal foot tilt
		for n = ENS
			keyRot.relFFP(l,:,n) = QuatRotVec(qBN, keyProj.relFFP(l,:,n)); % Rotate the FFPs about the motion centre into the N plane
			keyRot.footYaw(l,n) = keyProj.footYaw(l,n); % Foot yaw stays unchanged
			keyRot.FFP(l,:,n) = MC + keyRot.relFFP(l,:,n); % Convert to FFP
		end
	end

	%% Calculate the reconciled support keypoints

	% Calculate the required reconciled support keypoints
	for l = LS

		% Get the limb sign of the other leg
		o = OtherLimbSign(l);

		% Adjust lengths of AC and BD to be equal
		AC = keyRot.relFFP(o,:,4) - keyRot.relFFP(l,:,1);
		BD = keyRot.relFFP(o,:,5) - keyRot.relFFP(l,:,2);
		ACnorm = norm(AC);
		BDnorm = norm(BD);
		ACdecBDinc = 0.25*(ACnorm - BDnorm);
		ACvec = AC * (ACdecBDinc / ACnorm);
		BDvec = BD * (ACdecBDinc / BDnorm);
		keyRecon.relFFP(l,:,1) = keyRot.relFFP(l,:,1) + ACvec;
		keyRecon.relFFP(o,:,4) = keyRot.relFFP(o,:,4) - ACvec;
		keyRecon.relFFP(l,:,2) = keyRot.relFFP(l,:,2) - BDvec;
		keyRecon.relFFP(o,:,5) = keyRot.relFFP(o,:,5) + BDvec;
		keyRecon.relFFP(l,:,3) = keyRot.relFFP(l,:,3);
		keyRecon.relFFP(l,:,6:8) = keyRot.relFFP(l,:,6:8);

		% Adjust the relative foot yaws of AC and BD to be equal
		yawAC = keyRot.footYaw(o,4) - keyRot.footYaw(l,1);
		yawBD = keyRot.footYaw(o,5) - keyRot.footYaw(l,2);
		yawACdecBDinc = 0.5*(yawAC - yawBD);
		keyRecon.footYaw(l,1) = keyRot.footYaw(l,1);
		keyRecon.footYaw(o,4) = keyRot.footYaw(o,4) - yawACdecBDinc;
		keyRecon.footYaw(l,2) = keyRot.footYaw(l,2) - yawACdecBDinc;
		keyRecon.footYaw(o,5) = keyRot.footYaw(o,5);
		keyRecon.footYaw(l,3) = keyRot.footYaw(l,3);
		keyRecon.footYaw(l,6:8) = keyRot.footYaw(l,6:8);

		% Copy across the nominal foot tilt
		keyRecon.footTiltNom(l,:) = keyRot.footTiltNom(l,:); % Nominal foot tilt stays unchanged

	end

	% Calculate the corresponding foot floor points
	for l = LS
		for n = ENS
			keyRecon.FFP(l,:,n) = MC + keyRecon.relFFP(l,:,n); % Convert to FFP
		end
	end

	%% Calculate the adjusted support keypoints

	% Calculate the intermediate ground plane for the BD segments
	qNI = QuatSlerp(QuatIdentity, qNS, config.IRatio); % Note: qNI is always a tilt rotation as qNS is, I is effectively an interpolation between N and S, final implementation of this calculation can maybe be more efficient than full on slerp
	RNI = RotmatFromQuat(qNI);
	qBI = QuatMult(qBN, qNI);
	RBI = RotmatFromQuat(qBI);
	BzI = RBI(:,3)';

	% Calculate the support ground plane for the NN segments
	qNJ = QuatSlerp(QuatIdentity, qNS, config.JRatio); % Note: qNJ is always a tilt rotation as qNS is, J is effectively an interpolation between N and S, final implementation of this calculation can maybe be more efficient than full on slerp
	RNJ = RotmatFromQuat(qNJ);
	qBJ = QuatMult(qBN, qNJ);
	RBJ = RotmatFromQuat(qBJ);
	BzJ = RBJ(:,3)';

	% Interpolated ground plane orientation aliases
	BxI = RBI(:,1)';
	ByI = RBI(:,2)';
	xIx = RBI(1,1);
	xIy = RBI(2,1);
	xIz = RBI(3,1);
	yIx = RBI(1,2);
	yIy = RBI(2,2);
	yIz = RBI(3,2);
	zIx = RBI(1,3);
	zIy = RBI(2,3);
	zIz = RBI(3,3);

	% Tilt rotate the AC and BD segments into the required ground planes
	qBNS = ComposeQuat(qBS, qNB);
	qBNI = ComposeQuat(qBI, qNB);
	qBNJ = ComposeQuat(qBJ, qNB);
	for l = LS
		keyAdj.relFFP(l,:,1) = QuatRotVec(qBNS, keyRecon.relFFP(l,:,1));
		keyAdj.relFFP(l,:,2) = QuatRotVec(qBNI, keyRecon.relFFP(l,:,2));
		keyAdj.relFFP(l,:,3) = QuatRotVec(qBNJ, keyRecon.relFFP(l,:,3));
		keyAdj.relFFP(l,:,4) = QuatRotVec(qBNS, keyRecon.relFFP(l,:,4));
		keyAdj.relFFP(l,:,5) = QuatRotVec(qBNI, keyRecon.relFFP(l,:,5));
		keyAdj.relFFP(l,:,6) = QuatRotVec(qBNJ, keyRecon.relFFP(l,:,6));
		keyAdj.relFFP(l,:,7) = QuatRotVec(qBNJ, keyRecon.relFFP(l,:,7));
		keyAdj.relFFP(l,:,8) = QuatRotVec(qBNJ, keyRecon.relFFP(l,:,8));
	end

	% Calculate how much to adjust AC relative to BD along the MCL
	ACAdjustAB = -0.5*(dot(keyAdj.relFFP(1,:,1) - keyAdj.relFFP(1,:,2), BzS) + dot(keyAdj.relFFP(2,:,1) - keyAdj.relFFP(2,:,2), BzS)); % How much to S-normal adjust AC to make AB S-neutral (i.e. so that AB makes no S-normal adjustment in gait)
	ACAdjustCD = -0.5*(dot(keyAdj.relFFP(1,:,4) - keyAdj.relFFP(1,:,5), BzS) + dot(keyAdj.relFFP(2,:,4) - keyAdj.relFFP(2,:,5), BzS)); % How much to S-normal adjust AC to make CD S-neutral (i.e. so that CD makes no S-normal adjustment in gait)
	ACAdjust = ACAdjustAB*(1 - config.ACAdjustABRatio) + ACAdjustCD*config.ACAdjustABRatio; % How much S-normal adjustment of AC is desired (a balance between adjusting the front keypoints AB, and adjusting the back keypoints CD)
	lambdaAC = ACAdjust / dot(MCLhat,BzS); % How much adjustment of AC is required along the MCL to produce the desired amount of S-normal adjustment

	% Calculate how much to further adjust ABCD relative to NN along the MCL
	ABCDAdjust = -0.25*dot(BzJ, keyAdj.relFFP(1,:,2) + keyAdj.relFFP(1,:,4) + keyAdj.relFFP(2,:,2) + keyAdj.relFFP(2,:,4) - 2*keyAdj.relFFP(1,:,3) - 2*keyAdj.relFFP(2,:,3)); % How much to J-normal adjust ABCD to make BC J-neutral with NN (not accounting for the effect of lambdaAC yet)
	lambdaABCD = ABCDAdjust / dot(MCLhat,BzJ) - 0.5*lambdaAC; % How much adjustment of ABCD is required along the MCL to produce the desired amount of J-normal adjustment (now also accounting for the effect of lambdaAC)

	% Calculate the required final adjustments of AC and BD
	ACAdjustMCL = lambdaABCD + lambdaAC;
	BDAdjustMCL = lambdaABCD;

	% Apply the required adjustments
	for l = LS
		keyAdj.relHeight(l,1) = ACAdjustMCL;
		keyAdj.relHeight(l,2) = BDAdjustMCL;
		keyAdj.relHeight(l,3) = 0;
		keyAdj.relHeight(l,4) = ACAdjustMCL;
		keyAdj.relHeight(l,5) = BDAdjustMCL;
		keyAdj.relHeight(l,6:8) = 0;
	end

	% Calculate the finalised adjusted support keypoints
	for l = LS
		for n = ENS

			% Calculate the finalised adjusted FFP
			keyAdj.FFP(l,:,n) = MC + MCLhat*keyAdj.relHeight(l,n) + keyAdj.relFFP(l,:,n); % Convert to FFP

			% Calculate the finalised adjusted foot yaw
			keyAdj.footYaw(l,n) = keyRecon.footYaw(l,n); % Foot yaw stays unchanged

			% Calculate the finalised adjusted foot tilt
			gammaOffset = [config.footTiltIsLocalRatio*(keyAdj.footYaw(l,n) - keyProj.nomFootTiltAngles(l,1)) 0]; % Calculate the required gamma offset to adjust the interpretation of the required foot tilts between absolute and relative depending on the value of config.footTiltIsLocalRatio (0 => Absolute, 1 => Relative)
			footTiltNom = keyRecon.footTiltNom(l,:) + gammaOffset; % Absolute specification of foot tilt relative to N
			footTiltCts = in.footTiltCts + gammaOffset; % Absolute specification of foot tilt relative to N
			footTiltSupp = in.footTiltSupp; % Absolute specification of foot tilt relative to N
			if n >= 2 && n <= 4 % For BNC keypoints...
				keyAdj.footTilt(l,:,n) = TiltVectorSum(footTiltNom, footTiltCts, footTiltSupp); % Absolute specification of foot tilt relative to N
			else % For ADEFG keypoints...
				keyAdj.footTilt(l,:,n) = TiltVectorSum(footTiltNom, footTiltCts); % Absolute specification of foot tilt relative to N
			end

		end
	end

	% Calculate a suitable F keypoint
	for l = LS

		% Limb signs
		limbSign = LimbSign(l);
		o = OtherLimbSign(l);

		% Calculate the nominal F keypoint FFP that incorporates step height
		pointF = config.FOverNRatio*keyAdj.FFP(l,:,3) + (1.0 - config.FOverNRatio)*keyAdj.FFP(l,:,7);
		keyAdj.FFP(l,:,7) = pointF + coerceMin(legStepHeightDist + dot(keyAdj.FFP(o,:,3) - pointF, BzS), 0.0)*BzS; % The F point height is calculated relative to the S plane through the support foot N point

		% Calculate the nominal F keypoint orientation that respects the S plane
		qSF = QuatFromFootYawTilt(keyAdj.footYaw(l,7), keyAdj.footTilt(l,:,7)); % Reinterpret the foot orientation relative to N at F as being relative to S instead
		qNF = QuatMult(qNS, qSF);

		% Calculate the swing out rotations
		swingOutTiltPhase = in.swingOut(2)*[cos(in.swingOut(1)) sin(in.swingOut(1))];
		swingOutTiltPhase(1) = limbSign*coerceSoftMin(swingOutTiltPhase(1)/limbSign, -config.swingOutMaxIwd, config.swingOutBuf);
		swingOutTilt = [0 atan2(swingOutTiltPhase(2), swingOutTiltPhase(1)) sqrt(swingOutTiltPhase(1)*swingOutTiltPhase(1) + swingOutTiltPhase(2)*swingOutTiltPhase(2))];
		qNO = QuatFromTilt(swingOutTilt);
		qBNO = QuatMult(qBN, qNO, qNB);

		% Apply swing out to the F keypoint FFP
		hipPosFFP = CMD.hipPosFFP;
		hipPosFFP(2) = limbSign*hipPosFFP(2);
		keyAdj.FFP(l,:,7) = QuatRotVec(qBNO, keyAdj.FFP(l,:,7) - hipPosFFP) + hipPosFFP;

		% Apply swing out to the F keypoint orientation
		qNFswung = QuatMult(qNO, qNF);
		[keyAdj.footYaw(l,7), keyAdj.footTilt(l,:,7)] = FootYawTiltFromQuat(qNFswung);

	end

	% Calculate the hip height
	keyAdj.hipHeight = dot(CMD.hipCentrePos - MC, BzN) / F;

	% Transcribe the limb phase
	keyAdj.limbPhase = keyRaw.limbPhase;

	%% Calculate the adjusted support keypoint linear velocities (linVel, linAccSupp)

	% Collect the source data for the cubic spline interpolation
	linSuppU = unwrap(keyAdj.limbPhase(1:5))';
	linSuppX = [squeeze(keyAdj.FFP(1,:,1:5)); squeeze(keyAdj.FFP(2,:,1:5))]'; % Order is [Lx Ly Lz Rx Ry Rz]

	% Calculate derived terms
	linSuppH = picutMod(diff(linSuppU));
	linSuppVbar = diff(linSuppX)./linSuppH;

	% Apply the conditions of continuous velocity and acceleration at every keypoint
	linSuppAsub = zeros(3,5);
	for k = 1:3
		linSuppAsub(k,k) = linSuppH(k+1);
		linSuppAsub(k,k+1) = 2*(linSuppH(k) + linSuppH(k+1));
		linSuppAsub(k,k+2) = linSuppH(k);
	end
	linSuppA = blkdiag(linSuppAsub, linSuppAsub, linSuppAsub, linSuppAsub, linSuppAsub, linSuppAsub);
	linSuppb = 3*(linSuppH(2:4).*linSuppVbar(1:3,:) + linSuppH(1:3).*linSuppVbar(2:4,:));
	linSuppb = linSuppb(:);

	% Make the xS/yS accelerations zero at A, and the xI/yI accelerations zero at D
	zeroRow = zeros(1,15);
	eqnCoeffAx = [xSx*[2 1 0 0 0] xSy*[2 1 0 0 0] xSz*[2 1 0 0 0]];
	eqnCoeffAy = [ySx*[2 1 0 0 0] ySy*[2 1 0 0 0] ySz*[2 1 0 0 0]];
	eqnCoeffDx = [xIx*[0 0 0 1 2] xIy*[0 0 0 1 2] xIz*[0 0 0 1 2]];
	eqnCoeffDy = [yIx*[0 0 0 1 2] yIy*[0 0 0 1 2] yIz*[0 0 0 1 2]];
	linSuppA = [linSuppA; eqnCoeffAx zeroRow; zeroRow eqnCoeffAx];
	linSuppA = [linSuppA; eqnCoeffAy zeroRow; zeroRow eqnCoeffAy];
	linSuppA = [linSuppA; eqnCoeffDx zeroRow; zeroRow eqnCoeffDx];
	linSuppA = [linSuppA; eqnCoeffDy zeroRow; zeroRow eqnCoeffDy];
	linSuppb = [linSuppb; 3*(xSx*linSuppVbar(1,1) + xSy*linSuppVbar(1,2) + xSz*linSuppVbar(1,3)); 3*(xSx*linSuppVbar(1,4) + xSy*linSuppVbar(1,5) + xSz*linSuppVbar(1,6))];
	linSuppb = [linSuppb; 3*(ySx*linSuppVbar(1,1) + ySy*linSuppVbar(1,2) + ySz*linSuppVbar(1,3)); 3*(ySx*linSuppVbar(1,4) + ySy*linSuppVbar(1,5) + ySz*linSuppVbar(1,6))];
	linSuppb = [linSuppb; 3*(xIx*linSuppVbar(4,1) + xIy*linSuppVbar(4,2) + xIz*linSuppVbar(4,3)); 3*(xIx*linSuppVbar(4,4) + xIy*linSuppVbar(4,5) + xIz*linSuppVbar(4,6))];
	linSuppb = [linSuppb; 3*(yIx*linSuppVbar(4,1) + yIy*linSuppVbar(4,2) + yIz*linSuppVbar(4,3)); 3*(yIx*linSuppVbar(4,4) + yIy*linSuppVbar(4,5) + yIz*linSuppVbar(4,6))];

	% Make the mean zS velocity zero at AC, and the mean zI velocity zero at BD
	linSuppA = [linSuppA; zSx 0 0 0 0  zSy 0 0 0 0  zSz 0 0 0 0  0 0 0 zSx 0  0 0 0 zSy 0  0 0 0 zSz 0];
	linSuppA = [linSuppA; 0 0 0 zSx 0  0 0 0 zSy 0  0 0 0 zSz 0  zSx 0 0 0 0  zSy 0 0 0 0  zSz 0 0 0 0];
	linSuppA = [linSuppA; 0 zIx 0 0 0  0 zIy 0 0 0  0 zIz 0 0 0  0 0 0 0 zIx  0 0 0 0 zIy  0 0 0 0 zIz];
	linSuppA = [linSuppA; 0 0 0 0 zIx  0 0 0 0 zIy  0 0 0 0 zIz  0 zIx 0 0 0  0 zIy 0 0 0  0 zIz 0 0 0];
	linSuppb = [linSuppb; 0; 0; 0; 0];

	% Calculate the condition number of the matrix equation
	condSuppA = cond(linSuppA);

	% Solve the square matrix equation for the required velocities
	linSuppV = linSuppA \ linSuppb; % Order is [vLx(1:5) vLy(1:5) vLz(1:5) vRx(1:5) vRy(1:5) vRz(1:5)]
	linSuppV = reshape(linSuppV,5,6); % Order is [vLx vLy vLz vRx vRy vRz]

	% Generate the cubic spline coefficients for plotting reasons
	linSuppC = nan(4,24); % Order is [Lx(1:4) Ly(1:4) Lz(1:4) Rx(1:4) Ry(1:4) Rz(1:4)]
	for n = 1:6
		for k = 1:4
			j = 4*(n-1) + k;
			linSuppC(1,j) = linSuppX(k,n);
			linSuppC(2,j) = linSuppV(k,n);
			linSuppC(3,j) = (3*linSuppVbar(k,n) - 2*linSuppV(k,n) - linSuppV(k+1,n)) / linSuppH(k);
			linSuppC(4,j) = (linSuppV(k,n) + linSuppV(k+1,n) - 2*linSuppVbar(k,n)) / (linSuppH(k)*linSuppH(k));
		end
	end

	% Calculate the accelerations at each of the keypoints
	linSuppAcc(:,1:3) = [2*linSuppC(3,[1 5 9]); 2*linSuppC(3,[2 6 10]); 2*linSuppC(3,[3 7 11]); 2*linSuppC(3,[4 8 12]); 2*linSuppC(3,[4 8 12]) + 6*linSuppC(4,[4 8 12])*linSuppH(4)];
	linSuppAcc(:,4:6) = [2*linSuppC(3,[13 17 21]); 2*linSuppC(3,[14 18 22]); 2*linSuppC(3,[15 19 23]); 2*linSuppC(3,[16 20 24]); 2*linSuppC(3,[16 20 24]) + 6*linSuppC(4,[16 20 24])*linSuppH(4)];

	% Transcribe the results to the keyAdj struct
	keyAdj.linVel = permute(reshape(linSuppV,5,3,2), [3 2 1]);
	keyAdj.linAccSupp = permute(reshape(linSuppAcc,5,3,2), [3 2 1]);

	%% Calculate the adjusted swing keypoint linear velocities (FFP, linVel, linAccSwing)

	% Aliases for relevant configs
	rE = config.EHeightRatio;
	rG = config.GHeightRatio;

	% Calculate the limb phase intervals
	linSwingU = unwrap(keyAdj.limbPhase([5:8 1]))';
	linSwingH = picutMod(diff(linSwingU));

	% Precalculate terms
	hr12 = 3*linSwingH(1)/linSwingH(2);
	hr21 = 3*linSwingH(2)/linSwingH(1);
	hr23 = 3*linSwingH(2)/linSwingH(3);
	hr32 = 3*linSwingH(3)/linSwingH(2);
	hr34 = 3*linSwingH(3)/linSwingH(4);
	hr43 = 3*linSwingH(4)/linSwingH(3);

	% Matrix: Apply the conditions of continuous velocity and acceleration at every keypoint
	linSwingAsub = [2*(linSwingH(1) + linSwingH(2)), linSwingH(1), 0, hr12 - hr21, 0;
	                linSwingH(3), 2*(linSwingH(2) + linSwingH(3)), linSwingH(2), hr32, -hr23;
	                0, linSwingH(4), 2*(linSwingH(3) + linSwingH(4)), 0, hr34 - hr43];
	linSwingA = blkdiag(linSwingAsub, linSwingAsub, linSwingAsub);

	% Matrix: Specify the S-height of E and G
	linSwingA = [linSwingA; 0 0 0 zSx 0  0 0 0 zSy 0  0 0 0 zSz 0; 0 0 0 0 zSx  0 0 0 0 zSy  0 0 0 0 zSz];

	% Matrix: Make the xS/yS accelerations zero at A, and the xI/yI accelerations zero at D
	tmpVec1 = [linSwingH(1) 0 0 -3 0];
	tmpVec4 = [0 0 linSwingH(4) 0 3];
	linSwingA = [linSwingA; xIx*tmpVec1 xIy*tmpVec1 xIz*tmpVec1; yIx*tmpVec1 yIy*tmpVec1 yIz*tmpVec1];
	linSwingA = [linSwingA; xSx*tmpVec4 xSy*tmpVec4 xSz*tmpVec4; ySx*tmpVec4 ySy*tmpVec4 ySz*tmpVec4];

	% Calculate the condition number of the matrix equation
	condSwingA = cond(linSwingA);

	% Initialise variables
	linSwingX = nan(5,6); % Order is [Lx Ly Lz Rx Ry Rz]
	linSwingV = nan(5,6); % Order is [vLx vLy vLz vRx vRy vRz]

	% Calculate the required swing keypoint linear velocities
	for l = LS

		% Column indexing
		j = 3*(l-1) + (1:3);

		% Retrieve the known positions and velocities
		x1 = keyAdj.FFP(l,:,5);
		x3 = keyAdj.FFP(l,:,7);
		x5 = keyAdj.FFP(l,:,1);
		v1 = keyAdj.linVel(l,:,5);
		v5 = keyAdj.linVel(l,:,1);

		% Vector: Apply the conditions of continuous velocity and acceleration at every keypoint
		linSwingb = [hr12*x3 - hr21*x1 - linSwingH(2)*v1; (hr32 - hr23)*x3; hr34*x5 - hr43*x3 - linSwingH(3)*v5];
		linSwingb = linSwingb(:);

		% Vector: Specify the S-height of E and G
		linSwingb = [linSwingb; dot(rE*x3 + (1 - rE)*x1, BzS); dot(rG*x3 + (1 - rG)*x5, BzS)];

		% Vector: Make the xS/yS accelerations zero at A, and the xI/yI accelerations zero at D
		tmpRhs1 = -3*x1 - 2*linSwingH(1)*v1;
		tmpRhs4 = 3*x5 - 2*linSwingH(4)*v5;
		linSwingb = [linSwingb; dot(tmpRhs1, BxI); dot(tmpRhs1, ByI); dot(tmpRhs4, BxS); dot(tmpRhs4, ByS)];

		% Solve the square matrix equation for the required velocities
		linSwingSoln = linSwingA \ linSwingb; % Order is [vx(2:4) x2x x4x vy(2:4) x2y x4y vz(2:4) x2z x4z]

		% Construct summary position and velocity tables
		linSwingX(:,j) = [x1; linSwingSoln([4 9 14])'; x3; linSwingSoln([5 10 15])'; x5];
		linSwingV(:,j) = [v1; reshape(linSwingSoln([1:3 6:8 11:13]),3,3); v5];

	end

	% Calculate the linear interpolating velocities
	linSwingVbar = diff(linSwingX)./linSwingH;

	% Generate the cubic spline coefficients for plotting reasons
	linSwingC = nan(4,24); % Order is [Lx(1:4) Ly(1:4) Lz(1:4) Rx(1:4) Ry(1:4) Rz(1:4)]
	for n = 1:6
		for k = 1:4
			j = 4*(n-1) + k;
			linSwingC(1,j) = linSwingX(k,n);
			linSwingC(2,j) = linSwingV(k,n);
			linSwingC(3,j) = (3*linSwingVbar(k,n) - 2*linSwingV(k,n) - linSwingV(k+1,n)) / linSwingH(k);
			linSwingC(4,j) = (linSwingV(k,n) + linSwingV(k+1,n) - 2*linSwingVbar(k,n)) / (linSwingH(k)*linSwingH(k));
		end
	end

	% Calculate the accelerations at each of the keypoints
	linSwingAcc(:,1:3) = [2*linSwingC(3,[1 5 9]); 2*linSwingC(3,[2 6 10]); 2*linSwingC(3,[3 7 11]); 2*linSwingC(3,[4 8 12]); 2*linSwingC(3,[4 8 12]) + 6*linSwingC(4,[4 8 12])*linSwingH(4)];
	linSwingAcc(:,4:6) = [2*linSwingC(3,[13 17 21]); 2*linSwingC(3,[14 18 22]); 2*linSwingC(3,[15 19 23]); 2*linSwingC(3,[16 20 24]); 2*linSwingC(3,[16 20 24]) + 6*linSwingC(4,[16 20 24])*linSwingH(4)];

	% Transcribe the results to the keyAdj struct
	keyAdj.FFP(:,:,[6 8]) = permute(reshape(linSwingX([2 4],:),2,3,2), [3 2 1]);
	keyAdj.linVel(:,:,6:8) = permute(reshape(linSwingV(2:4,:),3,3,2), [3 2 1]);
	keyAdj.linAccSwing = permute(reshape(linSwingAcc,5,3,2), [3 2 1]);

	%% Calculate the adjusted keypoint angular velocities (footYaw, footTilt, angVel)

	% Indexing to only consider the cyclic loop ABNCDF
	indexMap = [1:5 7];

	% Calculate the limb phase intervals
	rotU = unwrap(keyAdj.limbPhase([indexMap 1]))';
	rotH = picutMod(diff(rotU));

	% Initialise variables
	rotX = nan(6,6); % Order is [Lc Ls Lp Rc Rs Rp]
	rotV = nan(6,6); % Order is [vLc vLs vLp vRc vRs vRp]
	rotVbar = nan(6,6);

	% Calculate the required keypoint angular velocities using a shape-preserving piecewise cubic Hermite interpolating polynomial
	for l = LS

		% Column indexing
		j = 3*(l-1) + (1:3);

		% Construct the source data for the cubic spline interpolation
		for k = 1:6
			kk = indexMap(k);
			rotX(k,j) = TiltPhaseFromFootYawTilt(keyAdj.footYaw(l,kk), keyAdj.footTilt(l,:,kk));
		end

		% Calculate the linear interpolant slopes
		rotVbar(:,j) = [diff(rotX(:,j)); rotX(1,j) - rotX(end,j)]./rotH;

		% Calculate the required keypoint velocities
		for k = 1:6
			nextk = mod(k,6) + 1;
			hA = rotH(k);
			hB = rotH(nextk);
			ws = 3*(hA + hB);
			w1 = (2*hA + hB)/ws;
			w2 = (hA + 2*hB)/ws;
			for jj = j
				vbarA = rotVbar(k,jj);
				vbarB = rotVbar(nextk,jj);
				vbarAB = vbarA*vbarB;
				if vbarAB > 0 % If vbarA and vbarB have the same sign and neither is zero...
					rotV(nextk,jj) = vbarAB/(w1*vbarA + w2*vbarB);
				else
					rotV(nextk,jj) = 0;
				end
			end
		end

	end

	% Calculate the 1x1 matrix A
	rotA = 2*(rotH(5) + rotH(6));

	% Calculate the condition number of the matrix equation
	condRotA = cond(rotA);

	% Recalculate the required swing keypoint velocities using continuous second derivative cubic spline interpolation
	for l = LS

		% Column indexing
		j = 3*(l-1) + (1:3);

		% Solve for the required velocities (analytic shortcut for solving the matrix equation)
		rotV(6,j) = (rotH(5)*(3*rotVbar(6,j) - rotV(1,j)) + rotH(6)*(3*rotVbar(5,j) - rotV(5,j))) / rotA;

	end

	% Generate the cubic spline coefficients for plotting reasons
	rotC = nan(4,36); % Order is [Lc(1:6) Ls(1:6) Lp(1:6) Rc(1:6) Rs(1:6) Rp(1:6)]
	for n = 1:6
		for k = 1:6
			j = 6*(n-1) + k;
			nextk = mod(k,6) + 1;
			rotC(1,j) = rotX(k,n);
			rotC(2,j) = rotV(k,n);
			rotC(3,j) = (3*rotVbar(k,n) - 2*rotV(k,n) - rotV(nextk,n)) / rotH(k);
			rotC(4,j) = (rotV(k,n) + rotV(nextk,n) - 2*rotVbar(k,n)) / (rotH(k)*rotH(k));
		end
	end

	% Transcribe the calculated velocities to the keyAdj struct
	for l = LS
		j = 3*(l-1) + (1:3);
		for k = 1:6
			kk = indexMap(k);
			[~, keyAdj.angVel(l,:,kk)] = AngFromTiltPhaseVel(rotV(k,j), keyAdj.footTilt(l,:,kk), qBN);
		end
	end

	% Calculate the intermediate orientations and velocities at EG
	rotUEG = [keyAdj.limbPhase(6); keyAdj.limbPhase(8)];
	hDE = picutMod(keyAdj.limbPhase(6) - keyAdj.limbPhase(5));
	hFG = picutMod(keyAdj.limbPhase(8) - keyAdj.limbPhase(7));
	for l = LS
		j = 3*(l-1) + (1:3);
		for k = 0:1
			n = 6 + 2*k;
			if l == 1
				m = [5 11 17] + k;
			else
				m = [23 29 35] + k;
			end
			if k == 0
				h = hDE;
			else
				h = hFG;
			end
			tiltPhase = rotC(1,m) + h*(rotC(2,m) + h*(rotC(3,m) + h*rotC(4,m)));
			tiltPhaseVel = rotC(2,m) + h*(2*rotC(3,m) + 3*h*rotC(4,m));
			[keyAdj.footYaw(l,n), keyAdj.footTilt(l,:,n)] = FootYawTiltFromTiltPhase(tiltPhase);
			[~, keyAdj.angVel(l,:,n)] = AngFromTiltPhaseVel(tiltPhaseVel, keyAdj.footTilt(l,:,n), qBN);
			rotXEG(k+1,j) = tiltPhase;
			rotVEG(k+1,j) = tiltPhaseVel;
		end
	end

	%% Finalise the adjusted keypoints (IP)

	% Calculate all adjusted keypoint inverse poses
	for l = LS
		limbSign = LimbSign(l);
		for n = ENS
			[~, qBF] = QuatFromFootYawTilt(keyAdj.footYaw(l,n), keyAdj.footTilt(l,:,n), qBN);
			keyAdj.IP(l,n) = InvFromFFPRot(keyAdj.FFP(l,:,n) - limbSign*CMD.ankleToFFP, qBF, limbSign, RM);
		end
	end

	%% Calculate the leaned support keypoints

	% Calculate the lean rotation
	qNL = QuatFromTilt([0 in.leanTilt(1) -in.leanTilt(2)]);
	RNL = RotmatFromQuat(qNL);
	qBL = QuatMult(qBN, qNL);
	qLB = QuatInv(qBL);
	RBL = RotmatFromQuat(qBL);
	qBNL = QuatMult(qBL, qNB);
	BzL = RBL(:,3)';

	% Apply the required lean to the keypoints
	for l = LS
		limbSign = LimbSign(l);
		hipCentrePosAnkle = CMD.hipCentrePos - limbSign*CMD.ankleToFFP; % Coordinates of the hip centre point in the IP.anklePos coordinate system (limb-dependent), just like CMD.hipCentrePos is the coordinates of the hip centre point in the FFP coordinate system
		for n = ENS
			keyLean.FFP(l,:,n) = QuatRotVec(qBNL, keyAdj.FFP(l,:,n) - CMD.hipCentrePos) + CMD.hipCentrePos;
			keyLean.IP(l,n).anklePos = QuatRotVec(qBNL, keyAdj.IP(l,n).anklePos - hipCentrePosAnkle) + hipCentrePosAnkle;
			keyLean.IP(l,n).footRot = QuatMult(qBNL, keyAdj.IP(l,n).footRot);
			keyLean.linVel(l,:,n) = QuatRotVec(qBNL, keyAdj.linVel(l,:,n));
			keyLean.angVel(l,:,n) = QuatRotVec(qBNL, keyAdj.angVel(l,:,n));
		end
		for n = NS
			keyLean.linAccSupp(l,:,n) = QuatRotVec(qBNL, keyAdj.linAccSupp(l,:,n));
			keyLean.linAccSwing(l,:,n) = QuatRotVec(qBNL, keyAdj.linAccSwing(l,:,n));
		end
	end

	% Apply the required lean to the motion centre and motion centre line
	leanMC = QuatRotVec(qBNL, MC - CMD.hipCentrePos) + CMD.hipCentrePos;
	leanMCLhat = QuatRotVec(qBNL, MCLhat);

	% Calculate the hip height
	keyLean.hipHeight = dot(CMD.hipCentrePos - leanMC, BzL) / F;

	% Transcribe the limb phase
	keyLean.limbPhase = keyAdj.limbPhase;

	%% Calculate the shifted support keypoints

	% Apply the required hip shift in the L plane
	leanFootShift = QuatRotVec(qBL, -[in.hipShift(1) in.hipShift(2) 0]*Ldbl);
	keyShift.IP = keyLean.IP;
	keyShift.FFP = keyLean.FFP;
	keyShift.limbPhase = keyLean.limbPhase;
	keyShift.linVel = keyLean.linVel;
	keyShift.linAccSupp = keyLean.linAccSupp;
	keyShift.linAccSwing = keyLean.linAccSwing;
	keyShift.angVel = keyLean.angVel;
	for l = LS
		for n = ENS
			keyShift.IP(l,n).anklePos = keyShift.IP(l,n).anklePos + leanFootShift;
			keyShift.FFP(l,:,n) = keyShift.FFP(l,:,n) + leanFootShift;
		end
	end

	% Apply the required shift to the motion centre
	shiftMC = leanMC + leanFootShift;
	shiftMCLhat = leanMCLhat;

	% Calculate the hip height
	keyShift.hipHeight = dot(CMD.hipCentrePos - shiftMC, BzL) / F;

	%% Calculate the final support keypoints

	% Calculate how much to finally adjust the motion keypoints along the motion centre line, in consideration of the nominal and maximum hip heights, and the minimum leg retraction
	adjustVec = AdjustRetraction(keyShift.IP, shiftMC, shiftMCLhat, BzL, in.hipHeightMax, RM, config);

	% Apply the required adjustment to the keypoints
	keyFinal.IP = keyShift.IP;
	keyFinal.FFP = keyShift.FFP;
	keyFinal.exactInvKin = true;
	keyFinal.limbPhase = keyShift.limbPhase;
	keyFinal.linVel = keyShift.linVel;
	keyFinal.linAccSupp = keyShift.linAccSupp;
	keyFinal.linAccSwing = keyShift.linAccSwing;
	keyFinal.angVel = keyShift.angVel;
	for l = LS
		limbSign = LimbSign(l);
		for n = ENS

			% Adjust the keypoint IP and FFP
			keyFinal.IP(l,n).anklePos = keyFinal.IP(l,n).anklePos + adjustVec;
			keyFinal.FFP(l,:,n) = keyFinal.FFP(l,:,n) + adjustVec;

			% Protect against excessive ankle point Z values
			offsetZ = keyFinal.IP(l,n).anklePos(3) - config.anklePointZMax*Ldbl;
			if offsetZ > 0
				warning('Coerced the linear ankle Z position of a final keypoint to avoid kinematics and dynamics issues!');
				keyFinal.IP(l,n).anklePos(3) = keyFinal.IP(l,n).anklePos(3) - offsetZ;
				keyFinal.FFP(l,3,n) = keyFinal.FFP(l,3,n) - offsetZ;
			end

			% Perform inverse kinematics to get the keypoint AP and JP
			[keyFinal.JP(l,n), keyFinal.AP(l,n), exact] = LegInvKin(keyFinal.IP(l,n), limbSign, RM);

			% Protect against insufficient leg retractions
			if keyFinal.AP(l,n).retraction < config.legRetMin
				warning('Coerced the leg retraction of a final keypoint to avoid kinematics and dynamics issues!');
				keyFinal.AP(l,n).retraction = config.legRetMin;
				keyFinal.JP(l,n) = JointFromAbs(keyFinal.AP(l,n));
				exact = false;
			end

			% Warn if the leg axis is almost parallel to the foot x-axis
			hipPRPos = CalcHipPose(keyFinal.AP(l,n), limbSign, RM);
			legAxis = keyFinal.IP(l,n).anklePos - hipPRPos;
			legAxis = legAxis / norm(legAxis);
			Rfoot = RotmatFromQuat(keyFinal.IP(l,n).footRot);
			cbeta = abs(dot(legAxis, Rfoot(:,1)));
			if cbeta > cos(config.footXLegAxisAngleMin)
				warning('The angle between the leg axis and foot x-axis of a final keypoint is below the desired threshold!');
			end

			% Calculate extra per keypoint values
			keyFinal.exactInvKin = keyFinal.exactInvKin && exact;
			keyFinal.tiltPhase(n,:,l) = TiltPhaseFromQuat(QuatMult(qNB, keyFinal.IP(l,n).footRot));

		end
	end

	% Apply the required adjustment to the motion centre
	finalMC = shiftMC + adjustVec;
	finalMCLhat = shiftMCLhat;

	% Calculate the hip height
	keyFinal.hipHeight = dot(CMD.hipCentrePos - finalMC, BzL) / F;

	% Calculate diagnostic variables
	keyFinal.minRetraction = min(cell2mat({keyFinal.AP(:).retraction}));

	%% Construct the final abstract/joint space waveforms

	% Calculate the abstract and joint velocities at each keypoint
	for l = LS
		limbSign = LimbSign(l);
		for n = ENS
			footVel = [keyFinal.linVel(l,:,n) keyFinal.angVel(l,:,n)]; % This is an FFP velocity
			keyFinal.APVel(l,n) = AbsFromInvVel(footVel, keyFinal.AP(l,n), limbSign, RM);
			keyFinal.JPVel(l,n) = JointFromAbsVel(keyFinal.APVel(l,n), keyFinal.AP(l,n));
		end
	end

	% Calculate the final limb phase intervals
	finalU = unwrap(keyFinal.limbPhase([1:end 1]))';
	finalH = picutMod(diff(finalU));

	% Generate single matrices of position and velocity data
	for l = LS
		for n = ENS
			finalAP(n,:,l) = StructToMat(keyFinal.AP(l,n));
			finalAPVel(n,:,l) = StructToMat(keyFinal.APVel(l,n));
			finalJP(n,:,l) = StructToMat(keyFinal.JP(l,n));
		end
	end

	% Generate the cubic spline coefficients
	finalC = AbstractPose;
	for k = 1:numel(absFields)
		field = absFields{k};
		for l = LS
			for n = ENS
				nextn = mod(n,8) + 1;
				h = picutMod(keyFinal.limbPhase(nextn) - keyFinal.limbPhase(n));
				xi = keyFinal.AP(l,n).(field);
				xf = keyFinal.AP(l,nextn).(field);
				vi = keyFinal.APVel(l,n).(field);
				vf = keyFinal.APVel(l,nextn).(field);
				vbar = (xf - xi)/h;
				coeff(:,n) = [xi; vi; (3*vbar - 2*vi - vf)/h; (vi + vf - 2*vbar)/(h*h)];
			end
			finalC(l).(field) = coeff;
		end
	end

	%% Evaluate the final abstract/joint space waveforms by gait phase

	% Set up a vector of gait phase values
	if isfield(in, 'wavemu')
		wave.mu = in.wavemu(:);
	else
		wave.mu = linspace(-pi, pi, 10001)';
	end
	M = numel(wave.mu);

	% Initialise variables
	APAL(M,1) = AbstractPose;
	APAR(M,1) = AbstractPose;
	JPAL(M,1) = JointPose;
	JPAR(M,1) = JointPose;
	IPAL(M,1) = InversePose;
	IPAR(M,1) = InversePose;
	IPFL = zeros(M,3);
	IPFR = zeros(M,3);
	IPFD = zeros(M,3);
	FFPL = zeros(M,3);
	FFPR = zeros(M,3);
	suppCoeff = zeros(M,2);
	numCoerced = 0;

	% Limb sign aliases
	leftLS = 1;
	rightLS = -1;

	% Calculate the phase length over which the support coefficient transition should take place
	supportTransitionPhaseLen = coerceMin(D + config.suppCoeffPhaseExtra, 1e-10);
	supportTransitionSlope = 1.0 / supportTransitionPhaseLen;
	margin = 0.5*supportTransitionPhaseLen;

	% Evaluate the gait waveform at each gait phase
	for m = M:-1:1

		% Calculate the required limb phases
		mu = [picut(wave.mu(m)) picut(wave.mu(m) + pi)];

		% Evaluate the gait pose for each leg
		for l = LS
			limbSign = LimbSign(l);
			[AP(l), coerced] = EvalAbsSpline(finalU, finalC(l), mu(l), limbSign, config);
			if coerced
				numCoerced = numCoerced + 1;
			end
		end

		% Save the calculated pose in all forms
		APAL(m) = AP(1);
		APAR(m) = AP(2);
		JPAL(m) = JointFromAbs(APAL(m));
		JPAR(m) = JointFromAbs(APAR(m));
		IPAL(m) = InvFromAbs(APAL(m), leftLS, RM);
		IPAR(m) = InvFromAbs(APAR(m), rightLS, RM);
		IPFL(m,:) = FootFloorPoint(IPAL(m), leftLS, RM); % Note: These FFPs are relative to the IP coordinate system!
		IPFR(m,:) = FootFloorPoint(IPAR(m), rightLS, RM); % Note: These FFPs are relative to the IP coordinate system!
		IPFD(m,:) = IPFL(m,:) - IPFR(m,:); % Note: These relative FFPs do not account for the offset between the left and right IP coordinate systems!
		FFPL(m,:) = IPFL(m,:) + leftLS*CMD.ankleToFFP;
		FFPR(m,:) = IPFR(m,:) + rightLS*CMD.ankleToFFP;

		% Calculate the support coefficient of each leg
		for l = LS
			limbPhase = mu(l);
			relUp = picut(limbPhase - (0.5*D - pi));
			relDown = picut(limbPhase - 0.5*D);
			if abs(relDown) <= margin
				suppCoeff(m,l) = 0.5 - relDown*supportTransitionSlope;
			elseif abs(relUp) <= margin
				suppCoeff(m,l) = 0.5 + relUp*supportTransitionSlope;
			elseif relDown >= 0
				suppCoeff(m,l) = 0;
			else
				suppCoeff(m,l) = 1;
			end
			suppCoeff(m,l) = config.supportCoeffRange*(suppCoeff(m,l) - 0.5) + 0.5;
		end

	end

	% Warn if any poses were coerced
	if numCoerced > 0
		warning('%d poses (%.1f%%) were coerced in the process of generating the final gait waveform!', numCoerced, 100*numCoerced/(2*M));
	end

	% Populate the required wave fields
	wave.APA = {APAL, APAR};
	wave.APP = {SAFromAS(APAL), SAFromAS(APAR)};
	wave.JPA = {JPAL, JPAR};
	wave.JPP = {SAFromAS(JPAL), SAFromAS(JPAR)};
	wave.IPA = {IPAL, IPAR};
	wave.IPF = {IPFL, IPFR, IPFD};
	wave.FFP = {FFPL, FFPR};
	wave.suppCoeff = suppCoeff;

	%% Print data values for debugging purposes
	
	% Print debug values
	if runType == -1
		DebugPrint(in);
		DebugPrint(keyAdj.FFP - CMD.hipCentrePos, 'keyAdj.FFP (hip centre coords)');
		DebugPrint(finalMC - CMD.hipCentrePos, 'finalMC (hip centre coords)');
		DebugPrint(finalMCLhat);
		DebugPrint(keyAdj.footYaw, 'keyAdj.footYaw');
		DebugPrint(keyProj.nomFootTiltAngles, 'keyProj.nomFootTiltAngles');
		DebugPrint(keyRot.footTiltNom, 'keyRot.footTiltNom');
		DebugPrint(legStepHeightDist);
		for l = LS
			for n = ENS
				keyAdjRelLTP(l,:,n) = keyAdj.relFFP(l,:,n) + MCLhat*keyAdj.relHeight(l,n);
			end
		end
		DebugPrint(keyAdjRelLTP);
		DebugPrint(keyAdj.footTilt, 'keyAdj.footTilt');
		DebugPrint(keyLean.linVel, 'keyLean.linVel');
		DebugPrint(keyLean.angVel, 'keyLean.angVel');
		DebugPrint(keyFinal.IP, 'keyFinal.IP');
		DebugPrint(keyFinal.AP, 'keyFinal.AP');
		DebugPrint(finalC, 'AbsSplineCoeff', true);
	end

	%% Start of plotting

	% Stop here if required
	if runType < 1
		return;
	end

	%% Plot the raw to projected keypoint figure

	% Initialise figure
	fig = InitFigure(fig, lims, txtOff, CMD.hipCentrePos, Hvec);

	% Figure title
	title(sprintf('Raw to Projected: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));

	% Generate config variables without leg lifting (leg pushout, swing and hip swing remain)
	configSwing = config;
	configSwing.tuningNoLegLifting = true;
	configSwing.legExtToAngleYGain = 0.0;
	configSwing.swingStartPhaseOffset = 0.0;
	configSwing.swingStopPhaseOffset = 0.0;

	% Generate and plot the raw foot floor point profiles
	[utmp, JPAtmp] = GenLegMotion(2*pi, [CMD.gcvX CMD.gcvY CMD.gcvZ], RM, configSwing);
	PlotLocus(JPAtmp{1}, utmp, fig - 1, -CMD.ankleToFFP, 1, RM, config);
	PlotLocus(JPAtmp{2}, utmp, fig - 1,  CMD.ankleToFFP, -1, RM, config);

	% Plot the motion profile centre points
	plot3([FFPM(1,1) FFPM(2,1)], [FFPM(1,2) FFPM(2,2)], [FFPM(1,3) FFPM(2,3)], 'x', 'Color', [0.15 0.50 0.15], 'MarkerSize', 6);
	plot3(MC(1), MC(2), MC(3), 'x', 'Color', [0.15 0.50 0.15], 'MarkerSize', 10);

	% Extract, plot and label the raw keypoints
	keyRawL = squeeze(keyRaw.FFP(1,:,:))';
	keyRawR = squeeze(keyRaw.FFP(2,:,:))';
	keyRawL = keyRawL([1:end 1],:);
	keyRawR = keyRawR([1:end 1],:);
	plot3(keyRawL(:,1), keyRawL(:,2), keyRawL(:,3), 'bx', 'MarkerSize', 10);
	plot3(keyRawR(:,1), keyRawR(:,2), keyRawR(:,3), 'rx', 'MarkerSize', 10);
	names = {'A', 'B', 'N', 'C', 'D', 'E', 'F', 'G'};
	for k = [1:5 7]
		if k > 5
			off = txtOff;
		else
			off = -txtOff;
		end
		text(keyRawL(k,1), keyRawL(k,2) + off, keyRawL(k,3), names{k}, 'HorizontalAlignment', 'center', 'Color', 'b');
		text(keyRawR(k,1), keyRawR(k,2) - off, keyRawR(k,3), names{k}, 'HorizontalAlignment', 'center', 'Color', 'r');
	end

	% Extract and plot the projected keypoints
	keyProjL = squeeze(keyProj.FFP(1,:,:))';
	keyProjR = squeeze(keyProj.FFP(2,:,:))';
	keyProjL = keyProjL([1:end 1],:);
	keyProjR = keyProjR([1:end 1],:);
	plot3(keyProjL(:,1), keyProjL(:,2), keyProjL(:,3), 'b-o', 'MarkerSize', 6);
	plot3(keyProjR(:,1), keyProjR(:,2), keyProjR(:,3), 'r-o', 'MarkerSize', 6);

	% Plot concurrency links between the projected keypoints
	plot3([keyProjL(1,1) keyProjR(4,1); keyProjL(2,1) keyProjR(5,1)]', [keyProjL(1,2) keyProjR(4,2); keyProjL(2,2) keyProjR(5,2)]', [keyProjL(1,3) keyProjR(4,3); keyProjL(2,3) keyProjR(5,3)]', '--', 'Color', [0.5 0 0.5]);
	plot3([keyProjL(4,1) keyProjR(1,1); keyProjL(5,1) keyProjR(2,1)]', [keyProjL(4,2) keyProjR(1,2); keyProjL(5,2) keyProjR(2,2)]', [keyProjL(4,3) keyProjR(1,3); keyProjL(5,3) keyProjR(2,3)]', '--', 'Color', [1.0 0 1.0]);

	% Plot the foot yaw and tilt arrows
	for l = LS
		plot3(FFPH(l,1), FFPH(l,2), FFPH(l,3), 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k');
		arrow3(FFPH(l,:), fySize*[cos(keyProj.nomFootTiltAngles(l,1)) sin(keyProj.nomFootTiltAngles(l,1)) 0], 0, 0, [0 0 1], '-', 'Color', [0.46 0.67 0.19]);
		ang = keyProj.nomFootTiltAngles(l,1) + keyProj.nomFootTiltAngles(l,2);
		arrow3(FFPH(l,:), ftSize*keyProj.nomFootTiltAngles(l,3)*[cos(ang) sin(ang) 0], 0, 0, [0 0 1], '-', 'Color', [0.93 0.69 0.13]);
		for n = ENS
			arrow3(keyProj.FFP(l,:,n), fySize*[cos(keyProj.footYaw(l,n)) sin(keyProj.footYaw(l,n)) 0], 0, 0, [0 0 1], '-', 'Color', [0.46 0.67 0.19]);
		end
	end

	% Finalise figure
	FinaliseFigure(lims);

	%% Plot the projected to rotated keypoint figure

	% Initialise figure
	fig = InitFigure(fig, lims, txtOff, CMD.hipCentrePos, Hvec);

	% Figure title
	title(sprintf('Projected to Rotated: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));

	% Plot the motion profile centre point
	plot3(MC(1), MC(2), MC(3), 'x', 'Color', [0.15 0.50 0.15], 'MarkerSize', 10);

	% Plot the projected keypoints
	plot3(keyProjL(:,1), keyProjL(:,2), keyProjL(:,3), 'bo', 'MarkerSize', 6);
	plot3(keyProjR(:,1), keyProjR(:,2), keyProjR(:,3), 'ro', 'MarkerSize', 6);

	% Extract and plot the rotated keypoints
	keyRotL = squeeze(keyRot.FFP(1,:,:))';
	keyRotR = squeeze(keyRot.FFP(2,:,:))';
	keyRotL = keyRotL([1:end 1],:);
	keyRotR = keyRotR([1:end 1],:);
	plot3(keyRotL(:,1), keyRotL(:,2), keyRotL(:,3), 'b-+', 'MarkerSize', 10);
	plot3(keyRotR(:,1), keyRotR(:,2), keyRotR(:,3), 'r-+', 'MarkerSize', 10);

	% Plot concurrency links between the rotated keypoints
	plot3([keyRotL(1,1) keyRotR(4,1); keyRotL(2,1) keyRotR(5,1)]', [keyRotL(1,2) keyRotR(4,2); keyRotL(2,2) keyRotR(5,2)]', [keyRotL(1,3) keyRotR(4,3); keyRotL(2,3) keyRotR(5,3)]', '--', 'Color', [0.5 0 0.5]);
	plot3([keyRotL(4,1) keyRotR(1,1); keyRotL(5,1) keyRotR(2,1)]', [keyRotL(4,2) keyRotR(1,2); keyRotL(5,2) keyRotR(2,2)]', [keyRotL(4,3) keyRotR(1,3); keyRotL(5,3) keyRotR(2,3)]', '--', 'Color', [1.0 0 1.0]);

	% Plot the foot yaw and tilt arrows
	for l = LS
		for n = ENS
			arrow3(keyRot.FFP(l,:,n), fySize*(cos(keyRot.footYaw(l,n))*RBN(:,1) + sin(keyRot.footYaw(l,n))*RBN(:,2)), 0, 0, BzN, '-', 'Color', [0.46 0.67 0.19]);
			gammaOffset = [config.footTiltIsLocalRatio*(keyRot.footYaw(l,n) - keyProj.nomFootTiltAngles(l,1)) 0];
			footTiltNom = keyRecon.footTiltNom(l,:) + gammaOffset;
			arrow3(keyRot.FFP(l,:,n), ftSize*footTiltNom(2)*(cos(footTiltNom(1))*RBN(:,1) + sin(footTiltNom(1))*RBN(:,2)), 0, 0, BzN, '-', 'Color', [0.93 0.69 0.13]);
		end
	end

	% Finalise figure
	FinaliseFigure(lims);

	%% Plot the rotated to reconciled keypoint figure

	% Initialise figure
	fig = InitFigure(fig, lims, txtOff, CMD.hipCentrePos, Hvec);

	% Figure title
	title(sprintf('Rotated to Reconciled: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));

	% Plot the motion profile centre point
	plot3(MC(1), MC(2), MC(3), 'x', 'Color', [0.15 0.50 0.15], 'MarkerSize', 10);

	% Plot the rotated keypoints
	plot3(keyRotL(:,1), keyRotL(:,2), keyRotL(:,3), 'b+', 'MarkerSize', 10);
	plot3(keyRotR(:,1), keyRotR(:,2), keyRotR(:,3), 'r+', 'MarkerSize', 10);

	% Extract and plot the reconciled keypoints
	keyReconL = squeeze(keyRecon.FFP(1,:,:))';
	keyReconR = squeeze(keyRecon.FFP(2,:,:))';
	keyReconL = keyReconL([1:end 1],:);
	keyReconR = keyReconR([1:end 1],:);
	plot3(keyReconL(:,1), keyReconL(:,2), keyReconL(:,3), 'b-x', 'MarkerSize', 10);
	plot3(keyReconR(:,1), keyReconR(:,2), keyReconR(:,3), 'r-x', 'MarkerSize', 10);

	% Plot concurrency links between the reconciled keypoints
	plot3([keyReconL(1,1) keyReconR(4,1); keyReconL(2,1) keyReconR(5,1)]', [keyReconL(1,2) keyReconR(4,2); keyReconL(2,2) keyReconR(5,2)]', [keyReconL(1,3) keyReconR(4,3); keyReconL(2,3) keyReconR(5,3)]', '--', 'Color', [0.5 0 0.5]);
	plot3([keyReconL(4,1) keyReconR(1,1); keyReconL(5,1) keyReconR(2,1)]', [keyReconL(4,2) keyReconR(1,2); keyReconL(5,2) keyReconR(2,2)]', [keyReconL(4,3) keyReconR(1,3); keyReconL(5,3) keyReconR(2,3)]', '--', 'Color', [1.0 0 1.0]);

	% Plot the foot yaw and tilt arrows
	for l = LS
		for n = ENS
			arrow3(keyRot.FFP(l,:,n), fySize*(cos(keyRot.footYaw(l,n))*RBN(:,1) + sin(keyRot.footYaw(l,n))*RBN(:,2)), 0, 0, BzN, '-', 'Color', [0.46 0.67 0.19]);
			arrow3(keyRecon.FFP(l,:,n), fySize*(cos(keyRecon.footYaw(l,n))*RBN(:,1) + sin(keyRecon.footYaw(l,n))*RBN(:,2)), 0, 0, BzN, '-', 'Color', [0.3 0.75 0.93]);
			gammaOffset = [config.footTiltIsLocalRatio*(keyRecon.footYaw(l,n) - keyProj.nomFootTiltAngles(l,1)) 0];
			footTiltNom = keyRecon.footTiltNom(l,:) + gammaOffset;
			arrow3(keyRecon.FFP(l,:,n), ftSize*footTiltNom(2)*(cos(footTiltNom(1))*RBN(:,1) + sin(footTiltNom(1))*RBN(:,2)), 0, 0, BzN, '-', 'Color', [0.93 0.69 0.13]);
		end
	end

	% Finalise figure
	FinaliseFigure(lims);

	%% Plot the reconciled to adjusted keypoint figure

	% Initialise figure
	fig = InitFigure(fig, lims, txtOff, CMD.hipCentrePos, Hvec);

	% Figure title
	title(sprintf('Reconciled to Adjusted: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));

	% Plot the motion profile centre point
	plot3(MC(1), MC(2), MC(3), 'x', 'Color', [0.15 0.50 0.15], 'MarkerSize', 10);

	% Plot the reconciled keypoints
	plot3(keyReconL(:,1), keyReconL(:,2), keyReconL(:,3), 'bx', 'MarkerSize', 10);
	plot3(keyReconR(:,1), keyReconR(:,2), keyReconR(:,3), 'rx', 'MarkerSize', 10);

	% Extract and plot the adjusted keypoints
	keyAdjL = squeeze(keyAdj.FFP(1,:,:))';
	keyAdjR = squeeze(keyAdj.FFP(2,:,:))';
	keyAdjL = keyAdjL([1:end 1],:);
	keyAdjR = keyAdjR([1:end 1],:);
	plot3(keyAdjL(:,1), keyAdjL(:,2), keyAdjL(:,3), 'b-+', 'MarkerSize', 10);
	plot3(keyAdjR(:,1), keyAdjR(:,2), keyAdjR(:,3), 'r-+', 'MarkerSize', 10);

	% Plot concurrency links between the adjusted keypoints
	plot3([keyAdjL(1,1) keyAdjR(4,1); keyAdjL(2,1) keyAdjR(5,1)]', [keyAdjL(1,2) keyAdjR(4,2); keyAdjL(2,2) keyAdjR(5,2)]', [keyAdjL(1,3) keyAdjR(4,3); keyAdjL(2,3) keyAdjR(5,3)]', '--', 'Color', [0.5 0 0.5]);
	plot3([keyAdjL(4,1) keyAdjR(1,1); keyAdjL(5,1) keyAdjR(2,1)]', [keyAdjL(4,2) keyAdjR(1,2); keyAdjL(5,2) keyAdjR(2,2)]', [keyAdjL(4,3) keyAdjR(1,3); keyAdjL(5,3) keyAdjR(2,3)]', '--', 'Color', [1.0 0 1.0]);

	% Plot ground planes
	plotPlanePatch(MC, 0.10*RBJ(:,1), 0.16*RBJ(:,2), [0.80 0.30 0.30], 'LineWidth', 1.0, 'FaceAlpha', 0.2);
	plotPlanePatch(MC + BDAdjustMCL*MCLhat, 0.11*RBI(:,1), 0.17*RBI(:,2), [0.15 0.70 0.15], 'LineWidth', 1.0, 'FaceAlpha', 0.2);
	plotPlanePatch(MC + ACAdjustMCL*MCLhat, 0.12*RBS(:,1), 0.18*RBS(:,2), [0.15 0.50 0.70], 'LineWidth', 1.0, 'FaceAlpha', 0.2);

	% Finalise figure
	FinaliseFigure(lims);

	%% Plot the adjusted keypoint figure

	% Initialise figure
	fig = InitFigure(fig, lims, txtOff, CMD.hipCentrePos, Hvec);

	% Figure title
	title(sprintf('Adjusted: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));

	% Plot the motion profile centre point
	plot3(MC(1), MC(2), MC(3), 'x', 'Color', [0.15 0.50 0.15], 'MarkerSize', 10);

	% Plot the adjusted keypoints
	plot3(keyAdjL(:,1), keyAdjL(:,2), keyAdjL(:,3), 'b-+', 'MarkerSize', 10);
	plot3(keyAdjR(:,1), keyAdjR(:,2), keyAdjR(:,3), 'r-+', 'MarkerSize', 10);

	% Plot concurrency links between the adjusted keypoints
	plot3([keyAdjL(1,1) keyAdjR(4,1); keyAdjL(2,1) keyAdjR(5,1)]', [keyAdjL(1,2) keyAdjR(4,2); keyAdjL(2,2) keyAdjR(5,2)]', [keyAdjL(1,3) keyAdjR(4,3); keyAdjL(2,3) keyAdjR(5,3)]', '--', 'Color', [0.5 0 0.5]);
	plot3([keyAdjL(4,1) keyAdjR(1,1); keyAdjL(5,1) keyAdjR(2,1)]', [keyAdjL(4,2) keyAdjR(1,2); keyAdjL(5,2) keyAdjR(2,2)]', [keyAdjL(4,3) keyAdjR(1,3); keyAdjL(5,3) keyAdjR(2,3)]', '--', 'Color', [1.0 0 1.0]);

	% Plot the nominal ground plane
	plotPlanePatch(MC, 0.10*RBN(:,1), 0.16*RBN(:,2), [0.80 0.80 0.30], 'LineWidth', 1.0, 'FaceAlpha', 0.2);

	% Plot the foot yaw and tilt arrows
	for l = LS
		for n = ENS
			qNF = QuatMult(qNB, keyAdj.IP(l,n).footRot);
			[footYaw, footTilt] = FootYawTiltFromQuat(qNF);
			arrow3(keyAdj.FFP(l,:,n), fySize*(cos(footYaw)*RBN(:,1) + sin(footYaw)*RBN(:,2)), 0, 0, BzN, '-', 'Color', [0.3 0.75 0.93]);
			arrow3(keyAdj.FFP(l,:,n), ftSize*footTilt(2)*(cos(footTilt(1))*RBN(:,1) + sin(footTilt(1))*RBN(:,2)), 0, 0, BzN, '-', 'Color', [0.93 0.69 0.13]);
		end
	end

	% Finalise figure
	FinaliseFigure(lims);

	%% Plot the N-projected adjusted vs rotated centering figure

	% Initialise figure
	fig = InitFigure(fig, lims, txtOff, CMD.hipCentrePos, Hvec);

	% Figure title
	title(sprintf('N-projected adjusted vs Rotated: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));

	% Plot the motion profile centre point
	plot3(MC(1), MC(2), MC(3), 'x', 'Color', [0.15 0.50 0.15], 'MarkerSize', 10);

	% Plot the rotated keypoints
	plot3(keyRotL(:,1), keyRotL(:,2), keyRotL(:,3), 'bx', 'MarkerSize', 10);
	plot3(keyRotR(:,1), keyRotR(:,2), keyRotR(:,3), 'rx', 'MarkerSize', 10);

	% Plot the N-projected adjusted keypoints (adjusted keypoints projected into the nominal ground plane for visualisation only)
	projKeyAdjL = keyAdjL - (keyAdjL - MC)*(BzN'*BzN);
	projKeyAdjR = keyAdjR - (keyAdjR - MC)*(BzN'*BzN);
	plot3(projKeyAdjL(:,1), projKeyAdjL(:,2), projKeyAdjL(:,3), 'b-+', 'MarkerSize', 10);
	plot3(projKeyAdjR(:,1), projKeyAdjR(:,2), projKeyAdjR(:,3), 'r-+', 'MarkerSize', 10);

	% Plot the nominal ground plane
	plotPlanePatch(MC, 0.10*RBN(:,1), 0.16*RBN(:,2), [0.80 0.80 0.30], 'LineWidth', 1.0, 'FaceAlpha', 0.2);

	% Finalise figure
	FinaliseFigure(lims);

	%% Plot the adjusted linear velocities figures

	% Initialise figure
	fig = InitFigure(fig, lims, txtOff, CMD.hipCentrePos, Hvec);

	% Figure title
	title(sprintf('Adjusted velocities: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));

	% Plot the motion profile centre point
	plot3(MC(1), MC(2), MC(3), 'x', 'Color', [0.15 0.50 0.15], 'MarkerSize', 10);

	% Plot the adjusted keypoints
	plot3(keyAdjL(:,1), keyAdjL(:,2), keyAdjL(:,3), 'b+', 'MarkerSize', 10);
	plot3(keyAdjR(:,1), keyAdjR(:,2), keyAdjR(:,3), 'r+', 'MarkerSize', 10);

	% Plot the nominal ground plane
	plotPlanePatch(MC, 0.10*RBN(:,1), 0.16*RBN(:,2), [0.80 0.80 0.30], 'LineWidth', 1.0, 'FaceAlpha', 0.2);

	% Plot the cubic splines used to generate the intermediate linear velocities
	[cstA, csxA, csvA, csaA, csjA] = GetCubicSplineData(linSuppC, repmat(linSuppH,6,1));
	plot3(csxA(:,1:4), csxA(:,5:8), csxA(:,9:12), 'b-');
	plot3(csxA(:,13:16), csxA(:,17:20), csxA(:,21:24), 'r-');
	[cstB, csxB, csvB, csaB, csjB] = GetCubicSplineData(linSwingC, repmat(linSwingH,6,1));
	plot3(csxB(:,1:4), csxB(:,5:8), csxB(:,9:12), 'b-');
	plot3(csxB(:,13:16), csxB(:,17:20), csxB(:,21:24), 'r-');

	% Plot foot plane indicators
	for l = LS
		for n = ENS
			R = RotmatFromQuat(keyAdj.IP(l,n).footRot);
			Rx = R(:,1)';
			Ry = R(:,2)';
			data = keyAdj.FFP(l,:,n) + footScaleX*[1 1 -1 -1 1]'*Rx + footScaleY*[1 -1 -1 1 1]'*Ry;
			plot3(data(:,1), data(:,2), data(:,3), 'k-');
		end
	end

	% Plot the foot velocity vectors
	for l = LS
		for n = ENS
			arrow3(keyAdj.FFP(l,:,n), vlScale*keyAdj.linVel(l,:,n), 0.005, 0, BzN, '-', 'Color', [0.46 0.67 0.19]);
			arrow3(keyAdj.FFP(l,:,n), vaScale*keyAdj.angVel(l,:,n), 0.005, 0, BzN, '-', 'Color', [0.93 0.69 0.13]);
		end
	end

	% Finalise figure
	FinaliseFigure(lims);

	% Plot the cubic spline data against limb phase
	linU = [linSuppU(1:end-1,:); linSwingU];
	linX = [linSuppX(1:end-1,:); linSwingX];
	linV = [linSuppV(1:end-1,:); linSwingV];
	ind = [1:4 25:28]' + 4*ones(8,1)*(0:5);
	cst = [cstA cstB]; cst = cst(:,ind);
	csx = [csxA csxB]; csx = csx(:,ind);
	csv = [csvA csvB]; csv = csv(:,ind);
	csa = [csaA csaB]; csa = csa(:,ind);
	csj = [csjA csjB]; csj = csj(:,ind);
	fig = PlotCubicSplineData(fig, 'Linear', linU, linX, linV, cst, csx, csv, csa, csj);

	% Plot the acceleration keypoints on top of the cubic spline data
	ax = subplot(2,2,3);
	hold on;
	ax.ColorOrderIndex = 1;
	plot(linSuppU, linSuppAcc, 'x');
	ax.ColorOrderIndex = 1;
	plot(linSwingU, linSwingAcc, 'x');
	hold off;
	subplot(2,2,4);
	legend('Lx', 'Ly', 'Lz', 'Rx', 'Ry', 'Rz', 'Location', 'NorthWest');

	%% Plot the adjusted angular velocities figures

	% Reset the figure
	figure(fig);
	fig = fig + 1;
	clf;

	% Hold the plot
	hold on;

	% Plot the cubic splines used to generate the intermediate angular velocities
	[cst, csx, csv, csa, csj] = GetCubicSplineData(rotC, repmat(rotH,6,1));
	plot3(csx(:,1:6), csx(:,7:12), csx(:,13:18), 'b-');
	plot3(csx(:,19:24), csx(:,25:30), csx(:,31:36), 'r-');
	plot3(rotX(:,1), rotX(:,2), rotX(:,3), 'bx');
	plot3(rotX(:,4), rotX(:,5), rotX(:,6), 'rx');

	% Plot the EG keypoints
	for l = LS
		for n = [6 8]
			tiltPhase = TiltPhaseFromFootYawTilt(keyAdj.footYaw(l,n), keyAdj.footTilt(l,:,n));
			plot3(tiltPhase(1), tiltPhase(2), tiltPhase(3), 'kx');
		end
	end

	% Unhold the plot
	hold off

	% Finalise the plot
	axis equal;
	xlabel('$\alpha\cos\tilde{\gamma}$', 'Interpreter', 'latex');
	ylabel('$\alpha\sin\tilde{\gamma}$', 'Interpreter', 'latex');
	zlabel('$\psi$', 'Interpreter', 'latex');
	view([40 48]);
	grid on;

	% Plot the cubic spline data against limb phase
	fig = PlotCubicSplineData(fig, 'Angular', rotU, rotX([1:end 1],:), rotV([1:end 1],:), cst, csx, csv, csa, csj);

	% Plot the EG keypoints on top of the cubic spline data
	subplot(2,2,1);
	hold on;
	plot(rotUEG, rotXEG,'kx');
	hold off;
	subplot(2,2,2);
	hold on;
	plot(rotUEG, rotVEG,'kx');
	hold off;
	subplot(2,2,4);
	legend('Lx', 'Ly', 'Lz', 'Rx', 'Ry', 'Rz', 'Location', 'NorthWest');

	%% Plot the leaned keypoint figure

	% Initialise figure
	fig = InitFigure(fig, lims, txtOff, CMD.hipCentrePos, Hvec);

	% Figure title
	title(sprintf('Leaned: %.2f %.2f %.2f by %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ, in.leanTilt(1), in.leanTilt(2)));

	% Plot the motion profile centre point
	plot3(MC(1), MC(2), MC(3), 'x', 'Color', [0.15 0.50 0.15], 'MarkerSize', 10);

	% Extract and plot the leaned keypoints
	keyLeanL = squeeze(keyLean.FFP(1,:,:))';
	keyLeanR = squeeze(keyLean.FFP(2,:,:))';
	keyLeanL = keyLeanL([1:end 1],:);
	keyLeanR = keyLeanR([1:end 1],:);
	plot3(keyLeanL(:,1), keyLeanL(:,2), keyLeanL(:,3), 'b-x', 'MarkerSize', 10);
	plot3(keyLeanR(:,1), keyLeanR(:,2), keyLeanR(:,3), 'r-x', 'MarkerSize', 10);

	% Extract and plot the leaned IP keypoints
	keyLeanIPL = cell2mat({keyLean.IP(1,:).anklePos}') + CMD.ankleToFFP;
	keyLeanIPR = cell2mat({keyLean.IP(2,:).anklePos}') - CMD.ankleToFFP;
	keyLeanIPL = keyLeanIPL([1:end 1],:);
	keyLeanIPR = keyLeanIPR([1:end 1],:);
	plot3(keyLeanIPL(:,1), keyLeanIPL(:,2), keyLeanIPL(:,3), 'b-+', 'MarkerSize', 10);
	plot3(keyLeanIPR(:,1), keyLeanIPR(:,2), keyLeanIPR(:,3), 'r-+', 'MarkerSize', 10);

	% Plot concurrency links between the FFP and IP keypoints
	plot3([keyLeanL(:,1)'; keyLeanIPL(:,1)'], [keyLeanL(:,2)'; keyLeanIPL(:,2)'], [keyLeanL(:,3)'; keyLeanIPL(:,3)'], 'b--');
	plot3([keyLeanR(:,1)'; keyLeanIPR(:,1)'], [keyLeanR(:,2)'; keyLeanIPR(:,2)'], [keyLeanR(:,3)'; keyLeanIPR(:,3)'], 'r--');

	% Plot the leaned motion profile centre point
	plot3(leanMC(1), leanMC(2), leanMC(3), 'o', 'Color', [0.15 0.50 0.15], 'MarkerSize', 6);
	arrow3(leanMC, 0.06*leanMCLhat, 0, 0, [1 1 0], '-', 'Color', [0.15 0.50 0.15]);
	arrow3(MC, 0.06*MCLhat, 0, 0, [1 0 0], '-', 'Color', [0.15 0.50 0.15]);

	% Plot ground planes
	plotPlanePatch(MC, 0.10*RBN(:,1), 0.16*RBN(:,2), [0.80 0.80 0.30], 'LineWidth', 1.0, 'FaceAlpha', 0.2);
	plotPlanePatch(leanMC, 0.11*RBL(:,1), 0.17*RBL(:,2), [0.80 0.30 0.80], 'LineWidth', 1.0, 'FaceAlpha', 0.2);

	% Finalise figure
	FinaliseFigure(lims);

	%% Plot the leaned to shifted keypoint figure

	% Initialise figure
	fig = InitFigure(fig, lims, txtOff, CMD.hipCentrePos, Hvec);

	% Figure title
	title(sprintf('Leaned to Shifted: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));

	% Plot the leaned keypoints and IP keypoints
	plot3(keyLeanL(:,1), keyLeanL(:,2), keyLeanL(:,3), 'b-x', 'MarkerSize', 10);
	plot3(keyLeanR(:,1), keyLeanR(:,2), keyLeanR(:,3), 'r-x', 'MarkerSize', 10);
	plot3(keyLeanIPL(:,1), keyLeanIPL(:,2), keyLeanIPL(:,3), 'b-+', 'MarkerSize', 10);
	plot3(keyLeanIPR(:,1), keyLeanIPR(:,2), keyLeanIPR(:,3), 'r-+', 'MarkerSize', 10);

	% Plot concurrency links between the leaned FFP and IP keypoints
	plot3([keyLeanL(:,1)'; keyLeanIPL(:,1)'], [keyLeanL(:,2)'; keyLeanIPL(:,2)'], [keyLeanL(:,3)'; keyLeanIPL(:,3)'], 'b--');
	plot3([keyLeanR(:,1)'; keyLeanIPR(:,1)'], [keyLeanR(:,2)'; keyLeanIPR(:,2)'], [keyLeanR(:,3)'; keyLeanIPR(:,3)'], 'r--');

	% Extract and plot the shifted keypoints
	keyShiftL = squeeze(keyShift.FFP(1,:,:))';
	keyShiftR = squeeze(keyShift.FFP(2,:,:))';
	keyShiftL = keyShiftL([1:end 1],:);
	keyShiftR = keyShiftR([1:end 1],:);
	plot3(keyShiftL(:,1), keyShiftL(:,2), keyShiftL(:,3), 'b-o', 'MarkerSize', 6);
	plot3(keyShiftR(:,1), keyShiftR(:,2), keyShiftR(:,3), 'r-o', 'MarkerSize', 6);

	% Extract and plot the shifted IP keypoints
	keyShiftIPL = cell2mat({keyShift.IP(1,:).anklePos}') + CMD.ankleToFFP;
	keyShiftIPR = cell2mat({keyShift.IP(2,:).anklePos}') - CMD.ankleToFFP;
	keyShiftIPL = keyShiftIPL([1:end 1],:);
	keyShiftIPR = keyShiftIPR([1:end 1],:);
	plot3(keyShiftIPL(:,1), keyShiftIPL(:,2), keyShiftIPL(:,3), 'b-^', 'MarkerSize', 6);
	plot3(keyShiftIPR(:,1), keyShiftIPR(:,2), keyShiftIPR(:,3), 'r-^', 'MarkerSize', 6);

	% Plot concurrency links between the shifted FFP and IP keypoints
	plot3([keyShiftL(:,1)'; keyShiftIPL(:,1)'], [keyShiftL(:,2)'; keyShiftIPL(:,2)'], [keyShiftL(:,3)'; keyShiftIPL(:,3)'], 'b:');
	plot3([keyShiftR(:,1)'; keyShiftIPR(:,1)'], [keyShiftR(:,2)'; keyShiftIPR(:,2)'], [keyShiftR(:,3)'; keyShiftIPR(:,3)'], 'r:');

	% Plot the leaned motion profile centre point
	plot3(leanMC(1), leanMC(2), leanMC(3), 'x', 'Color', [0.15 0.50 0.15], 'MarkerSize', 6);
	arrow3(leanMC, 0.06*leanMCLhat, 0, 0, [1 1 0], '-', 'Color', [0.15 0.50 0.15]);

	% Plot the shifted motion profile centre point
	plot3(shiftMC(1), shiftMC(2), shiftMC(3), 'o', 'Color', [0.15 0.50 0.15], 'MarkerSize', 6);
	arrow3(shiftMC, 0.06*shiftMCLhat, 0, 0, [1 1 0], '-', 'Color', [0.15 0.50 0.15]);

	% Plot the leaned ground plane
	plotPlanePatch(leanMC, 0.11*RBL(:,1), 0.17*RBL(:,2), [0.80 0.30 0.80], 'LineWidth', 1.0, 'FaceAlpha', 0.2);

	% Finalise figure
	FinaliseFigure(lims);

	%% Plot the shifted to final keypoint figure

	% Initialise figure
	fig = InitFigure(fig, lims, txtOff, CMD.hipCentrePos, Hvec);

	% Figure title
	title(sprintf('Shifted to Final: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));

	% Extract and plot the final keypoints
	keyFinalL = squeeze(keyFinal.FFP(1,:,:))';
	keyFinalR = squeeze(keyFinal.FFP(2,:,:))';
	keyFinalL = keyFinalL([1:end 1],:);
	keyFinalR = keyFinalR([1:end 1],:);
	plot3(keyFinalL(:,1), keyFinalL(:,2), keyFinalL(:,3), 'b-x', 'MarkerSize', 10);
	plot3(keyFinalR(:,1), keyFinalR(:,2), keyFinalR(:,3), 'r-x', 'MarkerSize', 10);

	% Extract and plot the final IP keypoints
	keyFinalIPL = cell2mat({keyFinal.IP(1,:).anklePos}') + CMD.ankleToFFP;
	keyFinalIPR = cell2mat({keyFinal.IP(2,:).anklePos}') - CMD.ankleToFFP;
	keyFinalIPL = keyFinalIPL([1:end 1],:);
	keyFinalIPR = keyFinalIPR([1:end 1],:);
	plot3(keyFinalIPL(:,1), keyFinalIPL(:,2), keyFinalIPL(:,3), 'b-+', 'MarkerSize', 10);
	plot3(keyFinalIPR(:,1), keyFinalIPR(:,2), keyFinalIPR(:,3), 'r-+', 'MarkerSize', 10);

	% Plot the shifted keypoints
	plot3(keyShiftL(:,1), keyShiftL(:,2), keyShiftL(:,3), 'b:o', 'MarkerSize', 6);
	plot3(keyShiftR(:,1), keyShiftR(:,2), keyShiftR(:,3), 'r:o', 'MarkerSize', 6);

	% Plot concurrency links between the FFP and IP keypoints
	plot3([keyFinalL(:,1)'; keyFinalIPL(:,1)'], [keyFinalL(:,2)'; keyFinalIPL(:,2)'], [keyFinalL(:,3)'; keyFinalIPL(:,3)'], 'b--');
	plot3([keyFinalR(:,1)'; keyFinalIPR(:,1)'], [keyFinalR(:,2)'; keyFinalIPR(:,2)'], [keyFinalR(:,3)'; keyFinalIPR(:,3)'], 'r--');

	% Plot the shifted motion profile centre point
	plot3(shiftMC(1), shiftMC(2), shiftMC(3), 'o', 'Color', [0.15 0.50 0.15], 'MarkerSize', 6);
	arrow3(shiftMC, 0.06*shiftMCLhat, 0, 0, [1 1 0], '-', 'Color', [0.15 0.50 0.15]);

	% Plot the final motion profile centre point
	plot3(finalMC(1), finalMC(2), finalMC(3), 'x', 'Color', [0.15 0.50 0.15], 'MarkerSize', 6);
	arrow3(finalMC, 0.06*finalMCLhat, 0, 0, [1 1 0], '-', 'Color', [0.15 0.50 0.15]);

	% Plot final adjustment vectors
	for l = LS
		for n = ENS
			arrow3(keyFinal.FFP(l,:,n) - adjustVec, adjustVec, 0, 0, [1 1 0], '-', 'Color', [0.15 0.50 0.15]);
		end
	end

	% Plot the leaned ground plane
	plotPlanePatch(finalMC, 0.11*RBL(:,1), 0.17*RBL(:,2), [0.80 0.30 0.80], 'LineWidth', 1.0, 'FaceAlpha', 0.2);

	% Finalise figure
	FinaliseFigure(lims);

	%% Plot the raw to final keypoint figure

	% Initialise figure
	fig = InitFigure(fig, lims, txtOff, CMD.hipCentrePos, Hvec);

	% Figure title
	title(sprintf('Raw to Final: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));

	% Plot the raw foot floor point profiles
	PlotLocus(JPAtmp{1}, utmp, fig - 1, -CMD.ankleToFFP, 1, RM, config);
	PlotLocus(JPAtmp{2}, utmp, fig - 1,  CMD.ankleToFFP, -1, RM, config);

	% Plot the motion profile centre point
	plot3(MC(1), MC(2), MC(3), 'x', 'Color', [0.15 0.50 0.15], 'MarkerSize', 10);

	% Plot the raw keypoints
	plot3(keyRawL(:,1), keyRawL(:,2), keyRawL(:,3), 'bo', 'MarkerSize', 6);
	plot3(keyRawR(:,1), keyRawR(:,2), keyRawR(:,3), 'ro', 'MarkerSize', 6);

	% Extract and plot the raw IP keypoints
	keyRawIPL = cell2mat({keyRaw.IP(1,:).anklePos}') + CMD.ankleToFFP;
	keyRawIPR = cell2mat({keyRaw.IP(2,:).anklePos}') - CMD.ankleToFFP;
	keyRawIPL = keyRawIPL([1:end 1],:);
	keyRawIPR = keyRawIPR([1:end 1],:);
	plot3(keyRawIPL(:,1), keyRawIPL(:,2), keyRawIPL(:,3), 'b:^', 'MarkerSize', 6);
	plot3(keyRawIPR(:,1), keyRawIPR(:,2), keyRawIPR(:,3), 'r:^', 'MarkerSize', 6);

	% Plot concurrency links between the raw FFP and IP keypoints
	plot3([keyRawL(:,1)'; keyRawIPL(:,1)'], [keyRawL(:,2)'; keyRawIPL(:,2)'], [keyRawL(:,3)'; keyRawIPL(:,3)'], 'b:');
	plot3([keyRawR(:,1)'; keyRawIPR(:,1)'], [keyRawR(:,2)'; keyRawIPR(:,2)'], [keyRawR(:,3)'; keyRawIPR(:,3)'], 'r:');

	% Plot the final keypoints
	plot3(keyFinalL(:,1), keyFinalL(:,2), keyFinalL(:,3), 'b-x', 'MarkerSize', 10);
	plot3(keyFinalR(:,1), keyFinalR(:,2), keyFinalR(:,3), 'r-x', 'MarkerSize', 10);

	% Plot the final IP keypoints
	plot3(keyFinalIPL(:,1), keyFinalIPL(:,2), keyFinalIPL(:,3), 'b-+', 'MarkerSize', 10);
	plot3(keyFinalIPR(:,1), keyFinalIPR(:,2), keyFinalIPR(:,3), 'r-+', 'MarkerSize', 10);

	% Plot concurrency links between the final FFP and IP keypoints
	plot3([keyFinalL(:,1)'; keyFinalIPL(:,1)'], [keyFinalL(:,2)'; keyFinalIPL(:,2)'], [keyFinalL(:,3)'; keyFinalIPL(:,3)'], 'b--');
	plot3([keyFinalR(:,1)'; keyFinalIPR(:,1)'], [keyFinalR(:,2)'; keyFinalIPR(:,2)'], [keyFinalR(:,3)'; keyFinalIPR(:,3)'], 'r--');

	% Plot the final motion profile centre point
	plot3(finalMC(1), finalMC(2), finalMC(3), 'x', 'Color', [0.15 0.50 0.15], 'MarkerSize', 6);
	arrow3(finalMC, 0.06*finalMCLhat, 0, 0, [1 1 0], '-', 'Color', [0.15 0.50 0.15]);

	% Plot the leaned ground plane
	plotPlanePatch(finalMC, 0.11*RBL(:,1), 0.17*RBL(:,2), [0.80 0.30 0.80], 'LineWidth', 1.0, 'FaceAlpha', 0.2);

	% Finalise figure
	FinaliseFigure(lims);

	%% Plot the final keypoint figure

	% Initialise figure
	fig = InitFigure(fig, lims, txtOff, CMD.hipCentrePos, Hvec);

	% Figure title
	title(sprintf('Final: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));

	% Plot the final motion profile centre point
	plot3(finalMC(1), finalMC(2), finalMC(3), 'x', 'Color', [0.15 0.50 0.15], 'MarkerSize', 6);

	% Plot the final keypoints
	plot3(keyFinalL(:,1), keyFinalL(:,2), keyFinalL(:,3), 'bx', 'MarkerSize', 10);
	plot3(keyFinalR(:,1), keyFinalR(:,2), keyFinalR(:,3), 'rx', 'MarkerSize', 10);

	% Plot the leaned ground plane
	plotPlanePatch(finalMC, 0.11*RBL(:,1), 0.17*RBL(:,2), [0.80 0.30 0.80], 'LineWidth', 1.0, 'FaceAlpha', 0.2);

	% Plot the inverse space interpolated trajectory curves
	[cstA, csxA] = GetCubicSplineData(linSuppC, repmat(linSuppH,6,1));
	[cstB, csxB] = GetCubicSplineData(linSwingC, repmat(linSwingH,6,1));
	offset = leanFootShift + adjustVec;
	for k = 1:size(csxA,1)
		for m = [1:4 13:16]
			indices = m + [0 4 8];
			csxA(k,indices) = QuatRotVec(qBNL, csxA(k,indices) - CMD.hipCentrePos) + CMD.hipCentrePos + offset;
			csxB(k,indices) = QuatRotVec(qBNL, csxB(k,indices) - CMD.hipCentrePos) + CMD.hipCentrePos + offset;
		end
	end
	plot3(csxA(:,1:4), csxA(:,5:8), csxA(:,9:12), 'b-');
	plot3(csxA(:,13:16), csxA(:,17:20), csxA(:,21:24), 'r-');
	plot3(csxB(:,1:4), csxB(:,5:8), csxB(:,9:12), 'b-');
	plot3(csxB(:,13:16), csxB(:,17:20), csxB(:,21:24), 'r-');

	% Plot the abstract space interpolated trajectory curves
	for l = LS
		limbSign = LimbSign(l);
		tmpData = wave.IPF{l} + CMD.ankleToFFP*limbSign;
		plot3(tmpData(:,1), tmpData(:,2), tmpData(:,3), 'm-');
	end

	% Plot foot plane indicators
	for l = LS
		for n = ENS
			R = RotmatFromQuat(keyFinal.IP(l,n).footRot);
			Rx = R(:,1)';
			Ry = R(:,2)';
			data = keyFinal.FFP(l,:,n) + footScaleX*[1 1 -1 -1 1]'*Rx + footScaleY*[1 -1 -1 1 1]'*Ry;
			plot3(data(:,1), data(:,2), data(:,3), 'k-');
		end
	end

	% Plot the velocity vectors
	for l = LS
		for n = ENS
			arrow3(keyFinal.FFP(l,:,n), vlScale*keyFinal.linVel(l,:,n), 0.005, 0, BzN, '-', 'Color', [0.46 0.67 0.19]);
			arrow3(keyFinal.FFP(l,:,n), vaScale*keyFinal.angVel(l,:,n), 0.005, 0, BzN, '-', 'Color', [0.93 0.69 0.13]);
		end
	end

	% Finalise figure
	FinaliseFigure(lims);

	%% Plot the angle space curves

	% Configure the side type strings
	sideType = {'Left', 'Right'};

	% Plot the abstract cubic spline data against limb phase
	for l = LS

		% Compute the cubic spline data
		coeff = StructToMat(finalC(l),2);
		[cst, csx, csv, csa, csj] = GetCubicSplineData(coeff, repmat(finalH,6,1));

		% Plot the abstract space data
		fig = PlotCubicSplineData(fig, [sideType{l} ' Abstract'], finalU, finalAP([1:end 1],:,l), finalAPVel([1:end 1],:,l), cst, csx, csv, csa, csj);
		subplot(2,2,4);
		legend('Leg Angle X', 'Leg Angle Y', 'Leg Angle Z', 'Foot Angle X', 'Foot Angle Y', 'Leg Retraction', 'Location', 'NorthWest');

	end

	% Plot the final waveform data
	for l = LS

		% Get the limb sign
		limbSign = LimbSign(l);

		% Calculate the required phase offset
		if l == 1
			phaseOffset = 0;
		else
			phaseOffset = pi;
		end

		% Reset the figure
		figure(fig);
		fig = fig + 1;
		clf;

		% Abstract space plot
		ax1 = subplot(1,2,1);

		% Plot the waveforms
		ax1.ColorOrderIndex = 1;
		h = plot(wave.mu, StructToMat(wave.APP{l},2), '-');

		% Hold the plot
		hold on;

		% Plot the final keypoints
		ax1.ColorOrderIndex = 1;
		plot(picut(finalU(1:end-1) + phaseOffset), finalAP(:,:,l), 'x');

		% Freeze the y-axis limits
		axis manual;
		ylims = ylim();

		% Plot the soft coercion bounds
		ax1.ColorOrderIndex = 1;
		plot([-pi; pi], repmat(config.legAngleXMin + config.legAngleXBuf,2,1)*limbSign, '--');
		plot([-pi; pi], repmat(config.legAngleYMin + config.legAngleYBuf,2,1), '--');
		plot([-pi; pi], repmat(config.legAngleZMin + config.legAngleZBuf,2,1)*limbSign, '--');
		plot([-pi; pi], repmat(config.footAngleXMin + config.footAngleXBuf,2,1)*limbSign, '--');
		plot([-pi; pi], repmat(config.footAngleYMin + config.footAngleYBuf,2,1), '--');
		plot([-pi; pi], repmat(config.legRetMin + config.legRetBuf,2,1), '--');
		plot([-pi; pi], repmat(config.legAngleXMax - config.legAngleXBuf,2,1)*limbSign, '--');
		plot([-pi; pi], repmat(config.legAngleYMax - config.legAngleYBuf,2,1), '--');
		plot([-pi; pi], repmat(config.legAngleZMax - config.legAngleZBuf,2,1)*limbSign, '--');
		plot([-pi; pi], repmat(config.footAngleXMax - config.footAngleXBuf,2,1)*limbSign, '--');
		plot([-pi; pi], repmat(config.footAngleYMax - config.footAngleYBuf,2,1), '--');
		plot([-pi; pi], repmat(config.legRetMax - config.legRetBuf,2,1), '--');

		% Label the keypoint phases
		tmpY = ylims(1) + 0.03*(ylims(2) - ylims(1));
		names = {'A', 'B', 'N', 'C', 'D', 'E', 'F', 'G'};
		for k = 1:length(keyFinal.limbPhase)
			tmpPhase = picut(keyFinal.limbPhase(k) + phaseOffset);
			plot(repmat(tmpPhase,2,1), ylims', 'b:');
			text(tmpPhase, tmpY, names{k}, 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');
		end

		% Label the support and swing phases
		tmpY = ylims(1) + 0.97*(ylims(2) - ylims(1));
		text(picut(keyFinal.limbPhase(3) + phaseOffset), tmpY, 'SUPPORT', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');
		text(picut(keyFinal.limbPhase(7) + phaseOffset), tmpY, 'SWING', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');

		% Unhold the plot
		hold off;

		% Finalise the plot
		xlim([-pi pi]);
		xlabel('Gait phase \mu');
		ylabel('Value');
		legend(h, 'Leg Angle X', 'Leg Angle Y', 'Leg Angle Z', 'Foot Angle X', 'Foot Angle Y', 'Leg Retraction', 'Location', 'SouthEast');
		title(sprintf('%s abstract waveform: %.2f %.2f %.2f', sideType{l}, CMD.gcvX, CMD.gcvY, CMD.gcvZ));
		grid off;

		% Joint space plot
		ax2 = subplot(1,2,2);

		% Plot the waveforms
		ax2.ColorOrderIndex = 1;
		h = plot(wave.mu, StructToMat(wave.JPP{l},2), '-');

		% Hold the plot
		hold on;

		% Plot the support coefficient
		hs = plot(wave.mu, wave.suppCoeff(:,l), '-');

		% Plot the final keypoints
		ax2.ColorOrderIndex = 1;
		plot(picut(finalU(1:end-1) + phaseOffset), finalJP(:,:,l), 'x');

		% Freeze the y-axis limits
		axis manual;
		ylims = ylim();

		% Label the keypoint phases
		tmpY = ylims(1) + 0.03*(ylims(2) - ylims(1));
		names = {'A', 'B', 'N', 'C', 'D', 'E', 'F', 'G'};
		for k = 1:length(keyFinal.limbPhase)
			tmpPhase = picut(keyFinal.limbPhase(k) + phaseOffset);
			plot(repmat(tmpPhase,2,1), ylims', 'b:');
			text(tmpPhase, tmpY, names{k}, 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');
		end

		% Label the support and swing phases
		tmpY = ylims(1) + 0.97*(ylims(2) - ylims(1));
		text(picut(keyFinal.limbPhase(3) + phaseOffset), tmpY, 'SUPPORT', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');
		text(picut(keyFinal.limbPhase(7) + phaseOffset), tmpY, 'SWING', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');

		% Unhold the plot
		hold off;

		% Finalise the plot
		xlim([-pi pi]);
		xlabel('Gait phase \mu');
		ylabel('Value');
		legend([h; hs], 'Hip Yaw', 'Hip Roll', 'Hip Pitch', 'Knee Pitch', 'Ankle Pitch', 'Ankle Roll', 'Supp Coeff', 'Location', 'SouthEast');
		title(sprintf('%s joint waveform: %.2f %.2f %.2f', sideType{l}, CMD.gcvX, CMD.gcvY, CMD.gcvZ));
		grid off;

		% Link the x-axes of the plots
		linkaxes([ax1 ax2], 'x');

	end

	%% Plot the space comparison curves

	% Reset the figure
	figure(fig);
	fig = fig + 1;
	clf;

	% Linear subplot
	ax1 = subplot(1,2,1);

	% Plot the inverse space spline interpolation
	cstAtmp = repmat(linSuppU(1:end-1)',1,6) + cstA;
	cstAtmp(:,13:24) = picut(cstAtmp(:,13:24) + pi);
	cstBtmp = repmat(linSwingU(1:end-1)',1,6) + cstB;
	cstBtmp(:,13:24) = picut(cstBtmp(:,13:24) + pi);
	linnum = size(cstAtmp,1);
	lincst = [reshape(cstAtmp, 4*linnum, 6); reshape(cstBtmp, 4*linnum, 6)];
	lincst([diff(lincst) < 0; false(1,6)]) = NaN;
	lincsx = [reshape(csxA, 4*size(csxA,1), 6); reshape(csxB, 4*size(csxB,1), 6)];
	plot(lincst, lincsx, '--', 'LineWidth', 1.5);

	% Hold the plot
	hold on;

	% Plot the final waveforms
	ax1.ColorOrderIndex = 1;
	linwave = [wave.IPF{1} + CMD.ankleToFFP, wave.IPF{2} - CMD.ankleToFFP];
	h = plot(wave.mu, linwave, '-');

	% Plot the final keypoints
	ax1.ColorOrderIndex = 1;
	plot(keyFinal.limbPhase', permute(keyFinal.FFP(1,:,:), [3 2 1]), 'x');
	plot(picut(keyFinal.limbPhase + pi)', permute(keyFinal.FFP(2,:,:), [3 2 1]), 'x');

	% Unhold the plot
	hold off;

	% Finalise the plot
	xlim([-pi pi]);
	xlabel('Gait phase \mu');
	ylabel('Value');
	legend(h, 'Left X', 'Left Y', 'Left Z', 'Right X', 'Right Y', 'Right Z', 'Location', 'SouthEast');
	title('Linear Position Comparison');
	grid on;

	% Angular subplot
	ax2 = subplot(1,2,2);

	% Plot the inverse space spline interpolation
	[cst, csx] = GetCubicSplineData(rotC, repmat(rotH,6,1));
	rotcst = repmat(rotU(1:end-1)',1,6) + cst;
	rotcst(:,19:36) = picut(rotcst(:,19:36) + pi);
	rotcst = reshape(rotcst, 6*size(rotcst,1), 6);
	rotcst([diff(rotcst) < -16*eps; false(1,6)]) = NaN;
	rotnum = size(csx,1);
	rotcsx = reshape(csx, 6*rotnum, 6);
	for l = LS
		j = 3*(l-1) + (1:3);
		for k = 1:size(rotcsx,1)
			qNF = QuatFromTiltPhase(rotcsx(k,j));
			rotcsx(k,j) = TiltPhaseFromQuat(QuatMult(qNL, qNF));
		end
	end
	plot(rotcst, rotcsx, '--', 'LineWidth', 1.5);

	% Hold the plot
	hold on;

	% Plot the final waveforms
	rotwave = nan(M,6);
	for l = LS
		j = 3*(l-1) + (1:3);
		for k = 1:M
			qNF = QuatMult(qNB, wave.IPA{l}(k).footRot);
			rotwave(k,j) = TiltPhaseFromQuat(qNF);
		end
	end
	ax2.ColorOrderIndex = 1;
	h = plot(wave.mu, rotwave, '-');

	% Plot the final keypoints
	ax2.ColorOrderIndex = 1;
	plot(keyFinal.limbPhase', keyFinal.tiltPhase(:,:,1), 'x');
	plot(picut(keyFinal.limbPhase + pi)', keyFinal.tiltPhase(:,:,2), 'x');

	% Unhold the plot
	hold off;

	% Finalise the plot
	xlim([-pi pi]);
	xlabel('Gait phase \mu');
	ylabel('Value');
	legend(h, {'Left $\alpha\cos\tilde{\gamma}$', 'Left $\alpha\sin\tilde{\gamma}$', 'Left $\psi$', 'Right $\alpha\cos\tilde{\gamma}$', 'Right $\alpha\sin\tilde{\gamma}$', 'Right $\psi$'}, 'Location', 'SouthEast', 'Interpreter', 'latex');
	title('Angular Orientation Comparison');
	grid on;

	% Link the x-axes of the plots
	linkaxes([ax1 ax2], 'x');

	% Reset the figure
	figure(fig);
	fig = fig + 1;
	clf;

	% Linear subplot
	ax1 = subplot(1,2,1);

	% Calculate the linear inverse space errors between the two interpolations
	num = size(lincst,1);
	linerr = nan(num,6);
	for j = 1:6
		csxdata = lincsx(:,j);
		wavedata = interp1(wave.mu, linwave(:,j), lincst(:,j));
		linerr(:,j) = abs(csxdata - wavedata);
	end

	% Hold the plot
	hold on;

	% Plot the linear errors
	ax1.ColorOrderIndex = 1;
	plot(lincst(:,1:3), linerr(:,1:3));
	tmpdata = picut(lincst(:,4:6) + pi);
	tmpdata([diff(tmpdata) < 0; false(1,3)]) = NaN;
	plot(tmpdata, linerr(:,4:6));

	% Plot the errors at the keypoints
	ax1.ColorOrderIndex = 1;
	indices = 1:linnum:size(linerr,1);
	plot(lincst(indices,1:3), linerr(indices,1:3), '.', 'MarkerSize', 12);
	tmpdata = picut(lincst(indices,4:6) + pi);
	plot(tmpdata, linerr(indices,4:6), '.', 'MarkerSize', 12);

	% Unhold the plot
	hold off;

	% Finalise the plot
	ax1.YScale = 'log';
	xlim([-pi pi]);
	ylim([1e-11 1e-1]);
	xlabel('Limb phase \mu');
	ylabel('Deviation');
	legend(h, 'Left X', 'Left Y', 'Left Z', 'Right X', 'Right Y', 'Right Z', 'Location', 'SouthEast');
	title('Linear Position Deviations');
	grid on;

	% Angular subplot
	ax2 = subplot(1,2,2);

	% Calculate the angular inverse space errors between the two interpolations
	num = size(rotcst,1);
	roterr = nan(num,6);
	for j = 1:6
		csxdata = rotcsx(:,j);
		wavedata = interp1(wave.mu, rotwave(:,j), rotcst(:,j));
		roterr(:,j) = abs(csxdata - wavedata);
	end

	% Hold the plot
	hold on;

	% Plot the angular errors
	ax2.ColorOrderIndex = 1;
	plot(rotcst(:,1:3), roterr(:,1:3));
	tmpdata = picut(rotcst(:,4:6) + pi);
	tmpdata([diff(tmpdata) < 0; false(1,3)]) = NaN;
	plot(tmpdata, roterr(:,4:6));

	% Plot the errors at the keypoints
	ax2.ColorOrderIndex = 1;
	indices = 1:rotnum:size(roterr,1);
	plot(rotcst(indices,1:3), roterr(indices,1:3), '.', 'MarkerSize', 12);
	tmpdata = picut(rotcst(indices,4:6) + pi);
	plot(tmpdata, roterr(indices,4:6), '.', 'MarkerSize', 12);

	% Unhold the plot
	hold off;

	% Finalise the plot
	ax2.YScale = 'log';
	xlim([-pi pi]);
	ylim([1e-11 1e-1]);
	xlabel('Limb phase \mu');
	ylabel('Deviation');
	legend(h, {'Left $\alpha\cos\tilde{\gamma}$', 'Left $\alpha\sin\tilde{\gamma}$', 'Left $\psi$', 'Right $\alpha\cos\tilde{\gamma}$', 'Right $\alpha\sin\tilde{\gamma}$', 'Right $\psi$'}, 'Location', 'SouthEast', 'Interpreter', 'latex');
	title('Angular Orientation Deviations');
	grid on;

	% Link the x-axes of the plots
	linkaxes([ax1 ax2], 'x');

	% Reset the figure
	figure(fig);
	fig = fig + 1;
	clf;

	% Linear subplot
	ax1 = subplot(1,2,1);

	% Plot the inverse space spline interpolation
	lincstmid = 0.5*(lincst(1:end-1,:) + lincst(2:end,:));
	dt = diff(lincst);
	lincsv = diff(lincsx)./dt;
	indices = find(any(dt < 16*eps, 2));
	lincstmid(indices,:) = [];
	lincsv(indices,:) = [];
	plot(lincstmid, lincsv, '--');

	% Hold the plot
	hold on;

	% Plot the final waveforms
	ax1.ColorOrderIndex = 1;
	wavemumid = 0.5*(wave.mu(1:end-1) + wave.mu(2:end));
	linwavevel = diff(linwave)./diff(wave.mu);
	h = plot(wavemumid, linwavevel, '-');

	% Plot the final keypoints
	ax1.ColorOrderIndex = 1;
	keylinvel = interp1(wavemumid, linwavevel, keyFinal.limbPhase, 'linear', 'extrap');
	plot(keyFinal.limbPhase', keylinvel, 'x');

	% Unhold the plot
	hold off;

	% Finalise the plot
	xlim([-pi pi]);
	xlabel('Gait phase \mu');
	ylabel('Value');
	legend(h, 'Left Vx', 'Left Vy', 'Left Vz', 'Right Vx', 'Right Vy', 'Right Vz', 'Location', 'SouthEast');
	title('Linear Velocity Comparison');
	grid on;

	% Angular subplot
	ax2 = subplot(1,2,2);

	% Plot the inverse space spline interpolation
	rotcstmid = 0.5*(rotcst(1:end-1,:) + rotcst(2:end,:));
	dt = diff(rotcst);
	rotcsv = diff(rotcsx)./dt;
	indices = find(any(dt < 16*eps, 2));
	rotcstmid(indices,:) = [];
	rotcsv(indices,:) = [];
	plot(rotcstmid, rotcsv, '--');

	% Hold the plot
	hold on;

	% Plot the final waveforms
	ax2.ColorOrderIndex = 1;
	wavemumid = 0.5*(wave.mu(1:end-1) + wave.mu(2:end));
	rotwavevel = diff(rotwave)./diff(wave.mu);
	h = plot(wavemumid, rotwavevel, '-');

	% Plot the final keypoints
	ax2.ColorOrderIndex = 1;
	keyrotvel = interp1(wavemumid, rotwavevel, keyFinal.limbPhase, 'linear', 'extrap');
	plot(keyFinal.limbPhase', keyrotvel, 'x');

	% Unhold the plot
	hold off;

	% Finalise the plot
	xlim([-pi pi]);
	xlabel('Gait phase \mu');
	ylabel('Value');
	legend(h, {'Left $d/d\mu(\alpha\cos\tilde{\gamma})$', 'Left $d/d\mu(\alpha\sin\tilde{\gamma})$', 'Left $\dot{\psi}$', 'Right $d/d\mu(\alpha\cos\tilde{\gamma})$', 'Right $d/d\mu(\alpha\sin\tilde{\gamma})$', 'Right $\dot{\psi}$'}, 'Location', 'SouthEast', 'Interpreter', 'latex');
	title('Angular Velocity Comparison');
	grid on;

	%% Plot the ground plane interpolating factor

	% Calculate the ground plane interpolating factor for each waveform data point
	groundu = nan(M,1);
	zN = QuatRotVec(qBNL, BzN);
	zS = QuatRotVec(qBNL, BzS);
	om = acos(coerceAbs(dot(zN,zS), 1.0));
	if om ~= 0
		for m = M:-1:1
			w = wave.FFP{1}(m,:) - wave.FFP{2}(m,:);
			groundu(m) = atan(dot(w, zN*sin(om)) / dot(w, zN*cos(om) - zS))/om;
		end
	end

	% Reset the figure
	figure(fig);
	fig = fig + 1;
	clf;

	% Get the default colours
	ax = subplot(1,1,1);
	ax.Box = 'on';
	colours = ax.ColorOrder;

	% Plot variables
	ylims = [-1 2];
	phases = keyFinal.limbPhase([1:5 7]);

	% Hold the plot
	hold on;

	% Plot the ground plane interpolating factor against gait phase
	plot(wave.mu, groundu, '--');
	indices = (wave.mu >= phases(1) & wave.mu <= phases(2));
	plot(wave.mu(indices), groundu(indices), 'r-');
	indices = (wave.mu >= phases(4) & wave.mu <= phases(5));
	plot(wave.mu(indices), groundu(indices), 'r-');

	% Label the keypoint phases
	tmpY = ylims(1) + 0.03*(ylims(2) - ylims(1));
	names = {'AC', 'BD', 'NF', 'CA', 'DB', 'FN'};
	for k = 1:length(phases)
		plot(repmat(phases(k),2,1), ylims', 'b:');
		text(phases(k), tmpY, names{k}, 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');
	end

	% Label the support phases
	tmpY = ylims(2) - 0.03*(ylims(2) - ylims(1));
	text(0.5*(phases(1) + phases(2)), tmpY, 'Double', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');
	text(phases(3), tmpY, 'Left Support', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');
	text(0.5*(phases(4) + phases(5)), tmpY, 'Double', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');
	text(phases(6), tmpY, 'Right Support', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');

	% Plot horizontal guide lines
	values = [0 config.JRatio config.IRatio 1];
	plot([-pi; pi], [1; 1]*values, '-');
	text(phases(3), values(1), 'N-plane', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Color', colours(2,:));
	text(phases(3), values(2), 'J-plane', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Color', colours(3,:));
	text(phases(3), values(3), 'I-plane', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Color', colours(4,:));
	text(phases(3), values(4), 'S-plane', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Color', colours(5,:));

	% Unhold the plot
	hold off;

	% Finalise the plot
	xlim([-pi pi]);
	ylim(ylims);
	xlabel('Gait phase \mu');
	ylabel('Value');
	title('Ground plane interpolating factor');
	grid off;

	%% Start of testing

	% Testing constants
	TestTol = 1e-13;

	% Stop here if required
	if runType < 2
		return;
	end

	%% Test rotations and vectors

	% Test header
	if runType >= 3
		disp('TEST: Rotations and vectors');
	end
	err = zeros(1,0);

	% Test normal vectors
	err(end+1) = abs(norm(BzN) - 1);
	err(end+1) = abs(norm(BzJ) - 1);
	err(end+1) = abs(norm(BzI) - 1);
	err(end+1) = abs(norm(BzS) - 1);
	err(end+1) = abs(norm(BzL) - 1);
	err(end+1) = norm(BzN - RBN(:,3)');
	err(end+1) = norm(BzJ - RBJ(:,3)');
	err(end+1) = norm(BzI - RBI(:,3)');
	err(end+1) = norm(BzS - RBS(:,3)');
	err(end+1) = norm(BzL - RBL(:,3)');
	err(end+1) = norm(BzN - QuatRotVec(qBN,[0 0 1]));
	err(end+1) = norm(BzJ - QuatRotVec(qBJ,[0 0 1]));
	err(end+1) = norm(BzI - QuatRotVec(qBI,[0 0 1]));
	err(end+1) = norm(BzS - QuatRotVec(qBS,[0 0 1]));
	err(end+1) = norm(BzL - QuatRotVec(qBL,[0 0 1]));

	% Test norm of quaternion rotations
	err(end+1) = abs(norm(qNB) - 1);
	err(end+1) = abs(norm(qNJ) - 1);
	err(end+1) = abs(norm(qNI) - 1);
	err(end+1) = abs(norm(qNS) - 1);
	err(end+1) = abs(norm(qNL) - 1);
	err(end+1) = abs(norm(qBN) - 1);
	err(end+1) = abs(norm(qBJ) - 1);
	err(end+1) = abs(norm(qBI) - 1);
	err(end+1) = abs(norm(qBS) - 1);
	err(end+1) = abs(norm(qBL) - 1);
	err(end+1) = abs(norm(qBNJ) - 1);
	err(end+1) = abs(norm(qBNI) - 1);
	err(end+1) = abs(norm(qBNS) - 1);
	err(end+1) = abs(norm(qBNL) - 1);

	% Test zero fused yaw of rotations
	err(end+1) = abs(FYawOfQuat(qBN));
	err(end+1) = abs(FYawOfQuat(qNB));
	err(end+1) = abs(FYawOfQuat(qNJ));
	err(end+1) = abs(FYawOfQuat(qNI));
	err(end+1) = abs(FYawOfQuat(qNS));
	err(end+1) = abs(FYawOfQuat(qNL));
	err(end+1) = abs(FYawOfRotmat(RBN));
	err(end+1) = abs(FYawOfRotmat(RNB));
	err(end+1) = abs(FYawOfRotmat(RNJ));
	err(end+1) = abs(FYawOfRotmat(RNI));
	err(end+1) = abs(FYawOfRotmat(RNS));
	err(end+1) = abs(FYawOfRotmat(RNL));

	% Test the transformation rotations
	err(end+1) = dot(qBNS(2:4),BzN);
	err(end+1) = dot(qBNS(2:4),BzS);
	err(end+1) = dot(qBNJ(2:4),BzN);
	err(end+1) = dot(qBNJ(2:4),BzJ);
	err(end+1) = dot(qBNI(2:4),BzN);
	err(end+1) = dot(qBNI(2:4),BzI);
	err(end+1) = dot(qBNL(2:4),BzN);
	err(end+1) = dot(qBNL(2:4),BzL);
	err(end+1) = abs((2*qBNJ(1)^2 - 1) - dot(BzN,BzJ));
	err(end+1) = abs((2*qBNI(1)^2 - 1) - dot(BzN,BzI));
	err(end+1) = abs((2*qBNS(1)^2 - 1) - dot(BzN,BzS));
	err(end+1) = abs((2*qBNL(1)^2 - 1) - dot(BzN,BzL));
	err(end+1) = norm(QuatRotVec(qBNJ,BzN) - BzJ);
	err(end+1) = norm(QuatRotVec(qBNI,BzN) - BzI);
	err(end+1) = norm(QuatRotVec(qBNS,BzN) - BzS);
	err(end+1) = norm(QuatRotVec(qBNL,BzN) - BzL);

	% Test unit vectors
	err(end+1) = abs(norm(MCLhat) - 1);
	err(end+1) = abs(norm(leanMCLhat) - 1);
	err(end+1) = abs(norm(shiftMCLhat) - 1);
	err(end+1) = abs(norm(finalMCLhat) - 1);

	% Test results
	if runType >= 3
		disp('Should all be zero:');
		disp(err');
	end
	if any(abs(err) > TestTol)
		warning('There are errors above the prescribed tolerance at the following rotvec indices:');
		disp(find(abs(err) > TestTol));
	end

	%% Test the projected support keypoints

	% Test header
	if runType >= 3
		disp('TEST: Projected support keypoints');
	end
	err = zeros(1,0);

	% Test that all keypoints are in the horizontal plane through the motion centre
	err(end+(1:18)) = [0 0 1]*([keyProjL; keyProjR] - MC)';

	% Test results
	if runType >= 3
		disp('Should all be zero:');
		disp(err');
	end
	if any(abs(err) > TestTol)
		warning('There are errors above the prescribed tolerance at the following projected indices:');
		disp(find(abs(err) > TestTol));
	end

	%% Test the rotated support keypoints

	% Test header
	if runType >= 3
		disp('TEST: Rotated support keypoints');
	end
	err = zeros(1,0);

	% Test that all keypoints are in the N-plane through the motion centre
	err(end+(1:18)) = BzN*([keyRotL; keyRotR] - MC)';

	% Test results
	if runType >= 3
		disp('Should all be zero:');
		disp(err');
	end
	if any(abs(err) > TestTol)
		warning('There are errors above the prescribed tolerance at the following rotated indices:');
		disp(find(abs(err) > TestTol));
	end

	%% Test the reconciled support keypoints

	% Test header
	if runType >= 3
		disp('TEST: Reconciled support keypoints');
	end
	err = zeros(1,0);

	% Test that all keypoints are in the N-plane through the motion centre
	err(end+(1:18)) = BzN*([keyReconL; keyReconR] - MC)';

	% Test AC vs BD length
	err(end+1) = abs(norm(keyRecon.FFP(1,:,1)-keyRecon.FFP(2,:,4)) - norm(keyRecon.FFP(1,:,2)-keyRecon.FFP(2,:,5)));
	err(end+1) = abs(norm(keyRecon.FFP(2,:,1)-keyRecon.FFP(1,:,4)) - norm(keyRecon.FFP(2,:,2)-keyRecon.FFP(1,:,5)));

	% Test AC vs BD foot yaw difference
	err(end+1) = abs((keyRecon.footYaw(1,1)-keyRecon.footYaw(2,4)) - (keyRecon.footYaw(1,2)-keyRecon.footYaw(2,5)));
	err(end+1) = abs((keyRecon.footYaw(2,1)-keyRecon.footYaw(1,4)) - (keyRecon.footYaw(2,2)-keyRecon.footYaw(1,5)));

	% Test that foot yaw range is symmetric
	err(end+1) = abs((keyRecon.footYaw(1,1)-keyRecon.footYaw(1,5)) - (keyRecon.footYaw(2,1)-keyRecon.footYaw(2,5)));
	err(end+1) = abs(keyRecon.footYaw(1,1) + keyRecon.footYaw(2,5));
	err(end+1) = abs(keyRecon.footYaw(2,1) + keyRecon.footYaw(1,5));

	% Test that peak foot yaws have not changed since raw
	err(end+1) = abs(keyRecon.footYaw(1,1) - FYawOfQuat(keyRaw.IP(1,1).footRot));
	err(end+1) = abs(keyRecon.footYaw(1,5) - FYawOfQuat(keyRaw.IP(1,5).footRot));
	err(end+1) = abs(keyRecon.footYaw(2,1) - FYawOfQuat(keyRaw.IP(2,1).footRot));
	err(end+1) = abs(keyRecon.footYaw(2,5) - FYawOfQuat(keyRaw.IP(2,5).footRot));

	% Test N hasn't moved since rotation
	err(end+1) = norm(keyRecon.FFP(1,:,3) - keyRot.FFP(1,:,3));
	err(end+1) = norm(keyRecon.FFP(2,:,3) - keyRot.FFP(2,:,3));

	% Test results
	if runType >= 3
		disp('Should all be zero:');
		disp(err');
	end
	if any(abs(err) > TestTol)
		warning('There are errors above the prescribed tolerance at the following reconciled indices:');
		disp(find(abs(err) > TestTol));
	end

	%% Test the adjusted support keypoints

	% Test header
	if runType >= 3
		disp('TEST: Adjusted support keypoints');
	end
	err = zeros(1,0);

	% Test the self-consistency of the output keypoints
	for l = LS
		limbSign = LimbSign(l);
		ankleToFFPLS = limbSign*CMD.ankleToFFP;
		for n = ENS
			err(end+1) = norm(FootFloorPoint(keyAdj.IP(l,n), limbSign, RM) + ankleToFFPLS - keyAdj.FFP(l,:,n));
			IPtmp = InvFromFFPRot(keyAdj.FFP(l,:,n) - ankleToFFPLS, keyAdj.IP(l,n).footRot, limbSign, RM);
			err(end+1) = norm(IPtmp.anklePos - keyAdj.IP(l,n).anklePos);
			err(end+1) = norm(IPtmp.footRot - keyAdj.IP(l,n).footRot);
			err(end+1) = abs(FYawOfQuat(QuatMult(qNB, keyAdj.IP(l,n).footRot)) - keyAdj.footYaw(l,n));
		end
	end

	% Test AC vs BD length
	err(end+1) = abs(norm(keyAdj.FFP(1,:,1)-keyAdj.FFP(2,:,4)) - norm(keyAdj.FFP(1,:,2)-keyAdj.FFP(2,:,5)));
	err(end+1) = abs(norm(keyAdj.FFP(2,:,1)-keyAdj.FFP(1,:,4)) - norm(keyAdj.FFP(2,:,2)-keyAdj.FFP(1,:,5)));

	% Test that AC and BD lengths haven't changed since reconciliation
	err(end+1) = abs(norm(keyAdj.FFP(1,:,1)-keyAdj.FFP(2,:,4)) - norm(keyRecon.FFP(1,:,1)-keyRecon.FFP(2,:,4)));
	err(end+1) = abs(norm(keyAdj.FFP(1,:,2)-keyAdj.FFP(2,:,5)) - norm(keyRecon.FFP(1,:,2)-keyRecon.FFP(2,:,5)));
	err(end+1) = abs(norm(keyAdj.FFP(2,:,1)-keyAdj.FFP(1,:,4)) - norm(keyRecon.FFP(2,:,1)-keyRecon.FFP(1,:,4)));
	err(end+1) = abs(norm(keyAdj.FFP(2,:,2)-keyAdj.FFP(1,:,5)) - norm(keyRecon.FFP(2,:,2)-keyRecon.FFP(1,:,5)));

	% Test AC vs BD foot yaw difference
	err(end+1) = abs((keyAdj.footYaw(1,1)-keyAdj.footYaw(2,4)) - (keyAdj.footYaw(1,2)-keyAdj.footYaw(2,5)));
	err(end+1) = abs((keyAdj.footYaw(2,1)-keyAdj.footYaw(1,4)) - (keyAdj.footYaw(2,2)-keyAdj.footYaw(1,5)));

	% Test that foot yaw range is symmetric
	err(end+1) = abs((keyAdj.footYaw(1,1)-keyAdj.footYaw(1,5)) - (keyAdj.footYaw(2,1)-keyAdj.footYaw(2,5)));
	err(end+1) = abs(keyAdj.footYaw(1,1) + keyAdj.footYaw(2,5));
	err(end+1) = abs(keyAdj.footYaw(2,1) + keyAdj.footYaw(1,5));

	% Test that peak foot yaws have not changed since raw
	err(end+1) = abs(keyAdj.footYaw(1,1) - FYawOfQuat(keyRaw.IP(1,1).footRot));
	err(end+1) = abs(keyAdj.footYaw(1,5) - FYawOfQuat(keyRaw.IP(1,5).footRot));
	err(end+1) = abs(keyAdj.footYaw(2,1) - FYawOfQuat(keyRaw.IP(2,1).footRot));
	err(end+1) = abs(keyAdj.footYaw(2,5) - FYawOfQuat(keyRaw.IP(2,5).footRot));

	% Test that ACAC is coplanar
	err(end+1) = dot(keyAdj.FFP(1,:,1)-keyAdj.FFP(1,:,4), cross(keyAdj.FFP(1,:,1)-keyAdj.FFP(2,:,1), keyAdj.FFP(1,:,1)-keyAdj.FFP(2,:,4)));

	% Test that BDBD is coplanar
	err(end+1) = dot(keyAdj.FFP(1,:,2)-keyAdj.FFP(1,:,5), cross(keyAdj.FFP(1,:,2)-keyAdj.FFP(2,:,2), keyAdj.FFP(1,:,2)-keyAdj.FFP(2,:,5)));

	% Test AC is S
	err(end+1) = dot(keyAdj.FFP(1,:,1)-keyAdj.FFP(1,:,4),BzS);
	err(end+1) = dot(keyAdj.FFP(1,:,1)-keyAdj.FFP(2,:,1),BzS);
	err(end+1) = dot(keyAdj.FFP(1,:,1)-keyAdj.FFP(2,:,4),BzS);
	err(end+1) = dot(keyAdj.FFP(1,:,4)-keyAdj.FFP(2,:,1),BzS);
	err(end+1) = dot(keyAdj.FFP(1,:,4)-keyAdj.FFP(2,:,4),BzS);
	err(end+1) = dot(keyAdj.FFP(2,:,1)-keyAdj.FFP(2,:,4),BzS);

	% Test BD is I
	err(end+1) = dot(keyAdj.FFP(1,:,2)-keyAdj.FFP(1,:,5),BzI);
	err(end+1) = dot(keyAdj.FFP(1,:,2)-keyAdj.FFP(2,:,2),BzI);
	err(end+1) = dot(keyAdj.FFP(1,:,2)-keyAdj.FFP(2,:,5),BzI);
	err(end+1) = dot(keyAdj.FFP(1,:,5)-keyAdj.FFP(2,:,2),BzI);
	err(end+1) = dot(keyAdj.FFP(1,:,5)-keyAdj.FFP(2,:,5),BzI);
	err(end+1) = dot(keyAdj.FFP(2,:,2)-keyAdj.FFP(2,:,5),BzI);

	% Test NN is in the J-plane that passes through the motion centre
	err(end+1) = dot(keyAdj.FFP(1,:,3)-keyAdj.FFP(2,:,3),BzJ);
	err(end+1) = dot(keyAdj.FFP(1,:,3)-MC,BzJ);
	err(end+1) = dot(keyAdj.FFP(2,:,3)-MC,BzJ);

	% Test that AC relative to BD is ok
	ABnormS = dot(keyAdj.FFP(1,:,1)-keyAdj.FFP(1,:,2)+keyAdj.FFP(2,:,1)-keyAdj.FFP(2,:,2),BzS);
	CDnormS = dot(keyAdj.FFP(1,:,4)-keyAdj.FFP(1,:,5)+keyAdj.FFP(2,:,4)-keyAdj.FFP(2,:,5),BzS);
	err(end+1) = interpolate([0 1],[ABnormS CDnormS],config.ACAdjustABRatio);

	% Test that ABCD relative to NN is ok
	err(end+1) = dot(BzJ,keyAdj.FFP(1,:,2)-keyAdj.FFP(1,:,3)+keyAdj.FFP(1,:,4)-keyAdj.FFP(1,:,3)+keyAdj.FFP(2,:,2)-keyAdj.FFP(2,:,3)+keyAdj.FFP(2,:,4)-keyAdj.FFP(2,:,3));

	% Test that the transformation from reconciled to adjusted for ACAC is a pure translation + tilt rotation relative to N
	adjAC = [squeeze(keyAdj.FFP(1,:,[1 4])) squeeze(keyAdj.FFP(2,:,[1 4]))];
	adjAC = RNB*(adjAC - repmat(mean(adjAC,2),1,4));
	reconAC = [squeeze(keyRecon.FFP(1,:,[1 4])) squeeze(keyRecon.FFP(2,:,[1 4]))];
	reconAC = RNB*(reconAC - repmat(mean(reconAC,2),1,4));
	err(end+(1:4)) = sqrt(sum((RNS*reconAC - adjAC).^2,1));
	err(end+1) = abs(FYawOfRotmat(RNS));

	% Test that the transformation from reconciled to adjusted for BDBD is a pure translation + tilt rotation relative to N
	adjBD = [squeeze(keyAdj.FFP(1,:,[2 5])) squeeze(keyAdj.FFP(2,:,[2 5]))];
	adjBD = RNB*(adjBD - repmat(mean(adjBD,2),1,4));
	reconBD = [squeeze(keyRecon.FFP(1,:,[2 5])) squeeze(keyRecon.FFP(2,:,[2 5]))];
	reconBD = RNB*(reconBD - repmat(mean(reconBD,2),1,4));
	err(end+(1:4)) = sqrt(sum((RNI*reconBD - adjBD).^2,1));
	err(end+1) = abs(FYawOfRotmat(RNI));

	% Test that the transformation from reconciled to adjusted for NN is a pure translation + tilt rotation relative to N
	adjN = [keyAdj.FFP(1,:,3)' keyAdj.FFP(2,:,3)'];
	adjN = RNB*(adjN - mean(adjN,2));
	reconN = [keyRecon.FFP(1,:,3)' keyRecon.FFP(2,:,3)'];
	reconN = RNB*(reconN - mean(reconN,2));
	err(end+(1:2)) = sqrt(sum((RNJ*reconN - adjN).^2,1));
	err(end+1) = abs(FYawOfRotmat(RNJ));

	% Test ABNCD N-coplanarity with motion centre (if S = N)
	if in.fusedS(1) == in.fusedPitchN && in.fusedS(2) == 0
		err(end+(1:10)) = BzN*([keyAdjL(1:5,:); keyAdjR(1:5,:)] - MC)';
	end

	% Test ABNCD S-coplanarity (if config.IRatio = 1 and/or config.JRatio = 1)
	if config.IRatio == 1
		err(end+(1:8)) = BzS*([keyAdjL([1 2 4 5],:); keyAdjR([1 2 4 5],:)] - (MC+BDAdjustMCL*MCLhat))';
	end
	if config.JRatio == 1
		err(end+(1:2)) = BzS*([keyAdjL(3,:); keyAdjR(3,:)] - MC)';
	end

	% Test the matrix equation condition numbers
	err(end+1) = coerceMax(100 - condSuppA, 0.0);
	err(end+1) = coerceMax(100 - condSwingA, 0.0);
	err(end+1) = coerceMax(100 - condRotA, 0.0);

	% See how centred the trajectory is relative to the N-plane
	if runType >= 3
		disp('Centering (should be small ~1e-4, but won''t be zero):');
		disp([norm(mean([projKeyAdjL;projKeyAdjR]) - mean([keyProjL;keyProjR])); norm(mean(projKeyAdjL) - mean(keyProjL)); norm(mean(projKeyAdjR) - mean(keyProjR))]);
	end

	% Test results
	if runType >= 3
		disp('Should all be zero:');
		disp(err');
	end
	if any(abs(err) > TestTol)
		warning('There are errors above the prescribed tolerance at the following adjusted indices:');
		disp(find(abs(err) > TestTol));
	end

	%% Test the leaned support keypoints

	% Test header
	if runType >= 3
		disp('TEST: Leaned support keypoints');
	end
	err = zeros(1,0);

	% Test the self-consistency of the output keypoints
	for l = LS
		limbSign = LimbSign(l);
		ankleToFFPLS = limbSign*CMD.ankleToFFP;
		for n = ENS
			err(end+1) = norm(FootFloorPoint(keyLean.IP(l,n),limbSign,RM) + ankleToFFPLS - keyLean.FFP(l,:,n));
			IPtmp = InvFromFFPRot(keyLean.FFP(l,:,n)-ankleToFFPLS,keyLean.IP(l,n).footRot,limbSign,RM);
			err(end+1) = norm(IPtmp.anklePos - keyLean.IP(l,n).anklePos);
			err(end+1) = norm(IPtmp.footRot - keyLean.IP(l,n).footRot);
		end
	end

	% Test the FFP centering about the motion centre
	for l = LS
		for n = ENS
			err(end+1) = norm((keyLean.FFP(l,:,n)-leanMC)*RBL - (keyAdj.FFP(l,:,n)-MC)*RBN);
		end
	end

	% Test the hip height
	err(end+1) = abs(keyLean.hipHeight - keyAdj.hipHeight);

	% Test the leaned motion centre line
	err(end+1) = abs(dot(CMD.hipCentrePos - leanMC, leanMCLhat) - dot(CMD.hipCentrePos - MC, MCLhat));
	err(end+1) = norm(leanMCLhat*RBL - MCLhat*RBN);
	err(end+1) = abs(norm(CMD.hipCentrePos - leanMC) - norm(CMD.hipCentrePos - MC));

	% Test AC vs BD length
	err(end+1) = abs(norm(keyLean.FFP(1,:,1)-keyLean.FFP(2,:,4)) - norm(keyLean.FFP(1,:,2)-keyLean.FFP(2,:,5)));
	err(end+1) = abs(norm(keyLean.FFP(2,:,1)-keyLean.FFP(1,:,4)) - norm(keyLean.FFP(2,:,2)-keyLean.FFP(1,:,5)));

	% Test that AC and BD lengths haven't changed since reconciliation
	err(end+1) = abs(norm(keyLean.FFP(1,:,1)-keyLean.FFP(2,:,4)) - norm(keyRecon.FFP(1,:,1)-keyRecon.FFP(2,:,4)));
	err(end+1) = abs(norm(keyLean.FFP(1,:,2)-keyLean.FFP(2,:,5)) - norm(keyRecon.FFP(1,:,2)-keyRecon.FFP(2,:,5)));
	err(end+1) = abs(norm(keyLean.FFP(2,:,1)-keyLean.FFP(1,:,4)) - norm(keyRecon.FFP(2,:,1)-keyRecon.FFP(1,:,4)));
	err(end+1) = abs(norm(keyLean.FFP(2,:,2)-keyLean.FFP(1,:,5)) - norm(keyRecon.FFP(2,:,2)-keyRecon.FFP(1,:,5)));

	% Test that ACAC is coplanar
	err(end+1) = dot(keyLean.FFP(1,:,1)-keyLean.FFP(1,:,4), cross(keyLean.FFP(1,:,1)-keyLean.FFP(2,:,1), keyLean.FFP(1,:,1)-keyLean.FFP(2,:,4)));

	% Test that BDBD is coplanar
	err(end+1) = dot(keyLean.FFP(1,:,2)-keyLean.FFP(1,:,5), cross(keyLean.FFP(1,:,2)-keyLean.FFP(2,:,2), keyLean.FFP(1,:,2)-keyLean.FFP(2,:,5)));

	% Test NN is in the correct plane through the leaned motion centre
	leanedzJ = RBL*RNJ(:,3);
	err(end+1) = dot(keyLean.FFP(1,:,3)-keyLean.FFP(2,:,3),leanedzJ);
	err(end+1) = dot(keyLean.FFP(1,:,3)-leanMC,leanedzJ);
	err(end+1) = dot(keyLean.FFP(2,:,3)-leanMC,leanedzJ);

	% Test results
	if runType >= 3
		disp('Should all be zero:');
		disp(err');
	end
	if any(abs(err) > TestTol)
		warning('There are errors above the prescribed tolerance at the following leaned indices:');
		disp(find(abs(err) > TestTol));
	end

	%% Test the shifted support keypoints

	% Test header
	if runType >= 3
		disp('TEST: Shifted support keypoints');
	end
	err = zeros(1,0);

	% Test the self-consistency of the output keypoints
	for l = LS
		limbSign = LimbSign(l);
		ankleToFFPLS = limbSign*CMD.ankleToFFP;
		for n = ENS
			err(end+1) = norm(FootFloorPoint(keyShift.IP(l,n),limbSign,RM) + ankleToFFPLS - keyShift.FFP(l,:,n));
			IPtmp = InvFromFFPRot(keyShift.FFP(l,:,n)-ankleToFFPLS,keyShift.IP(l,n).footRot,limbSign,RM);
			err(end+1) = norm(IPtmp.anklePos - keyShift.IP(l,n).anklePos);
			err(end+1) = norm(IPtmp.footRot - keyShift.IP(l,n).footRot);
		end
	end

	% Test the FFP centering about the motion centre and constant shift from the previous keypoints
	err(end+1) = abs(norm(leanFootShift) - Ldbl*norm(in.hipShift));
	for l = LS
		for n = ENS
			err(end+1) = norm((keyShift.FFP(l,:,n)-shiftMC)*RBL - (keyAdj.FFP(l,:,n)-MC)*RBN);
			err(end+1) = norm(keyShift.IP(l,n).footRot - keyLean.IP(l,n).footRot);
			err(end+1) = norm(keyShift.FFP(l,:,n) - keyLean.FFP(l,:,n) - leanFootShift);
		end
	end

	% Test the hip height
	err(end+1) = abs(keyShift.hipHeight - keyLean.hipHeight);

	% Test the shifted motion centre line
	err(end+1) = norm(shiftMCLhat - leanMCLhat);

	% Test AC vs BD length
	err(end+1) = abs(norm(keyShift.FFP(1,:,1)-keyShift.FFP(2,:,4)) - norm(keyShift.FFP(1,:,2)-keyShift.FFP(2,:,5)));
	err(end+1) = abs(norm(keyShift.FFP(2,:,1)-keyShift.FFP(1,:,4)) - norm(keyShift.FFP(2,:,2)-keyShift.FFP(1,:,5)));

	% Test that AC and BD lengths haven't changed since reconciliation
	err(end+1) = abs(norm(keyShift.FFP(1,:,1)-keyShift.FFP(2,:,4)) - norm(keyRecon.FFP(1,:,1)-keyRecon.FFP(2,:,4)));
	err(end+1) = abs(norm(keyShift.FFP(1,:,2)-keyShift.FFP(2,:,5)) - norm(keyRecon.FFP(1,:,2)-keyRecon.FFP(2,:,5)));
	err(end+1) = abs(norm(keyShift.FFP(2,:,1)-keyShift.FFP(1,:,4)) - norm(keyRecon.FFP(2,:,1)-keyRecon.FFP(1,:,4)));
	err(end+1) = abs(norm(keyShift.FFP(2,:,2)-keyShift.FFP(1,:,5)) - norm(keyRecon.FFP(2,:,2)-keyRecon.FFP(1,:,5)));

	% Test that ACAC is coplanar
	err(end+1) = dot(keyShift.FFP(1,:,1)-keyShift.FFP(1,:,4), cross(keyShift.FFP(1,:,1)-keyShift.FFP(2,:,1), keyShift.FFP(1,:,1)-keyShift.FFP(2,:,4)));

	% Test that BDBD is coplanar
	err(end+1) = dot(keyShift.FFP(1,:,2)-keyShift.FFP(1,:,5), cross(keyShift.FFP(1,:,2)-keyShift.FFP(2,:,2), keyShift.FFP(1,:,2)-keyShift.FFP(2,:,5)));

	% Test NN is in the L-plane that passes through the shifted motion centre
	err(end+1) = dot(keyShift.FFP(1,:,3)-keyShift.FFP(2,:,3),leanedzJ);
	err(end+1) = dot(keyShift.FFP(1,:,3)-shiftMC,leanedzJ);
	err(end+1) = dot(keyShift.FFP(2,:,3)-shiftMC,leanedzJ);

	% Test results
	if runType >= 3
		disp('Should all be zero:');
		disp(err');
	end
	if any(abs(err) > TestTol)
		warning('There are errors above the prescribed tolerance at the following shifted indices:');
		disp(find(abs(err) > TestTol));
	end

	%% Test the final support keypoints

	% Test header
	if runType >= 3
		disp('TEST: Final support keypoints');
	end
	err = zeros(1,0);

	% Test that the inverse kinematics calculations were all exact
	if keyFinal.exactInvKin
		err(end+1) = 0;
	else
		err(end+1) = Inf;
	end

	% Test the self-consistency of the output keypoints
	for l = LS
		limbSign = LimbSign(l);
		ankleToFFPLS = limbSign*CMD.ankleToFFP;
		for n = ENS
			err(end+1) = norm(FootFloorPoint(keyFinal.IP(l,n),limbSign,RM) + ankleToFFPLS - keyFinal.FFP(l,:,n));
			IPtmp = InvFromFFPRot(keyFinal.FFP(l,:,n)-ankleToFFPLS,keyFinal.IP(l,n).footRot,limbSign,RM);
			err(end+1) = norm(IPtmp.anklePos - keyFinal.IP(l,n).anklePos);
			err(end+1) = norm(IPtmp.footRot - keyFinal.IP(l,n).footRot);
		end
	end

	% Test the consistency of the adjusted foot yaw and tilt with the final orientations relative to L
	for l = LS
		for n = ENS
			qLF = QuatMult(qLB, keyFinal.IP(l,n).footRot);
			tiltPhaseA = TiltPhaseFromQuat(qLF);
			tiltPhaseB = TiltPhaseFromFootYawTilt(keyAdj.footYaw(l,n), keyAdj.footTilt(l,:,n));
			err(end+1) = norm(tiltPhaseA - tiltPhaseB);
		end
	end

	% Test the FFP centering about the motion centre and constant shift from the previous keypoints
	err(end+1) = norm(cross(adjustVec, shiftMCLhat));
	for l = LS
		for n = ENS
			err(end+1) = norm((keyFinal.FFP(l,:,n)-finalMC)*RBL - (keyAdj.FFP(l,:,n)-MC)*RBN);
			err(end+1) = norm(keyFinal.IP(l,n).footRot - keyLean.IP(l,n).footRot);
			err(end+1) = norm(keyFinal.FFP(l,:,n) - keyShift.FFP(l,:,n) - adjustVec);
		end
	end

	% Test the minimum leg retraction
	for l = LS
		for n = ENS
			err(end+1) = coerceMax(keyFinal.AP(l,n).retraction - config.minLegRetraction, 0)*(TestTol/1e-8);
		end
	end

	% Test the hip height limitations
	err(end+1) = coerceMax(F*min(config.nomHipHeight, in.hipHeightMax) - dot(CMD.hipCentrePos - finalMC, BzL), 0);

	% Test that the final adjustment has done its job
	err(end+1) = coerceMin(keyFinal.hipHeight - config.nomHipHeight, 0);
	err(end+1) = coerceMin(keyFinal.hipHeight - in.hipHeightMax, 0);
	err(end+1) = (TestTol / 1e-8) * coerceMax(keyFinal.minRetraction - config.minLegRetraction, 0);
	if abs(keyFinal.hipHeight - config.nomHipHeight) > TestTol && abs(keyFinal.hipHeight - in.hipHeightMax) > TestTol && abs(keyFinal.minRetraction - config.minLegRetraction) > 1e-8
		err(end+1) = Inf;
	else
		err(end+1) = 0;
	end

	% Test the final motion centre line
	err(end+1) = norm(finalMCLhat - leanMCLhat);

	% Test AC vs BD length
	err(end+1) = abs(norm(keyFinal.FFP(1,:,1)-keyFinal.FFP(2,:,4)) - norm(keyFinal.FFP(1,:,2)-keyFinal.FFP(2,:,5)));
	err(end+1) = abs(norm(keyFinal.FFP(2,:,1)-keyFinal.FFP(1,:,4)) - norm(keyFinal.FFP(2,:,2)-keyFinal.FFP(1,:,5)));

	% Test that AC and BD lengths haven't changed since reconciliation
	err(end+1) = abs(norm(keyFinal.FFP(1,:,1)-keyFinal.FFP(2,:,4)) - norm(keyRecon.FFP(1,:,1)-keyRecon.FFP(2,:,4)));
	err(end+1) = abs(norm(keyFinal.FFP(1,:,2)-keyFinal.FFP(2,:,5)) - norm(keyRecon.FFP(1,:,2)-keyRecon.FFP(2,:,5)));
	err(end+1) = abs(norm(keyFinal.FFP(2,:,1)-keyFinal.FFP(1,:,4)) - norm(keyRecon.FFP(2,:,1)-keyRecon.FFP(1,:,4)));
	err(end+1) = abs(norm(keyFinal.FFP(2,:,2)-keyFinal.FFP(1,:,5)) - norm(keyRecon.FFP(2,:,2)-keyRecon.FFP(1,:,5)));

	% Test that ACAC is coplanar
	err(end+1) = dot(keyFinal.FFP(1,:,1)-keyFinal.FFP(1,:,4), cross(keyFinal.FFP(1,:,1)-keyFinal.FFP(2,:,1), keyFinal.FFP(1,:,1)-keyFinal.FFP(2,:,4)));

	% Test that BDBD is coplanar
	err(end+1) = dot(keyFinal.FFP(1,:,2)-keyFinal.FFP(1,:,5), cross(keyFinal.FFP(1,:,2)-keyFinal.FFP(2,:,2), keyFinal.FFP(1,:,2)-keyFinal.FFP(2,:,5)));

	% Test NN is in the L-plane that passes through the final motion centre
	err(end+1) = dot(keyFinal.FFP(1,:,3)-keyFinal.FFP(2,:,3),leanedzJ);
	err(end+1) = dot(keyFinal.FFP(1,:,3)-finalMC,leanedzJ);
	err(end+1) = dot(keyFinal.FFP(2,:,3)-finalMC,leanedzJ);

	% Test results
	if runType >= 3
		disp('Should all be zero:');
		disp(err');
	end
	if any(abs(err) > TestTol)
		warning('There are errors above the prescribed tolerance at the following final indices:');
		disp(find(abs(err) > TestTol));
	end

	%% Populate the output struct

	% Populate the output struct
	out = struct();
	out.fig = fig;
	out.in = in;
	out.Ldbl = Ldbl;
	out.F = F;
	out.RBN = RBN;
	out.RBS = RBS;
	out.qNB = qNB;
	out.qNJ = qNJ;
	out.qNI = qNI;
	out.qSN = qSN;
	out.BzN = BzN;
	out.BzJ = BzJ;
	out.BzI = BzI;
	out.BzS = BzS;
	out.MC = MC;
	out.MCLhat = MCLhat;
	out.leanMC = leanMC;
	out.shiftMC = shiftMC;
	out.finalMC = finalMC;
	out.keyRaw = keyRaw;
	out.keyProj = keyProj;
	out.keyRot = keyRot;
	out.keyRecon = keyRecon;
	out.keyAdj = keyAdj;
	out.keyLean = keyLean;
	out.keyShift = keyShift;
	out.keyFinal = keyFinal;
	out.finalU = finalU;
	out.finalH = finalH;
	out.finalC = finalC;
	out.wave = wave;

	%% Results

	% Results:
	% - The raw and projected keypoint calculations together form an interchangeable "black box" algorithm for converting
	%   {halt pose, gcv} --> {step sizes, foot tilts}, where 'step sizes' incorporates FFP keypoints in the xy-plane (due
	%   to gcv xyz) and foot yaws (due to gcv z)
	% - The hip swing (purely a manifestation in the black box that generates the raw keypoints) is centred phase-wise
	%   about the double support periods.
	% - If S and N are the same, then all 10 keyAdj points are in the N-plane through the motion centre, and only differ
	%   from the projected points due to reconciliation (generally a minor tweak)
	% - If config.IRatio is 1, then all 8 ABCD keyAdj points are S-coplanar (NOT necessarily through the motion centre).
	%   If config.JRatio is also 1, then all 10 ABNCD keyAdj points are S-coplanar through the motion centre.

end

% Convert a specification of foot yaw and absolute foot tilt relative to N into a quaternion foot orientation relative to N
function [qNF, qBF, tiltNF] = QuatFromFootYawTilt(footYaw, footTilt, qBN)

	% Calculate the foot orientation relative to N
	tiltNF = [footYaw picut(footTilt(1)-footYaw) footTilt(2)];
	qNF = QuatFromTilt(tiltNF);

	% Calculate the foot orientation relative to B (if required)
	if nargout >= 2
		qBF = QuatMult(qBN, qNF);
	end

end

% Convert a quaternion foot orientation relative to N into a specification of foot yaw and absolute foot tilt relative to N
function [footYaw, footTilt, tiltNF] = FootYawTiltFromQuat(qNF)

	% Calculate the required foot yaw and tilt
	tiltNF = TiltFromQuat(qNF);
	footYaw = tiltNF(1);
	footTilt = [picut(tiltNF(2)+footYaw) tiltNF(3)];

end

% Convert a specification of foot yaw and absolute foot tilt relative to N into a tilt phase orientation
function [tiltPhase] = TiltPhaseFromFootYawTilt(footYaw, footTilt)

	% Calculate the required tilt phase
	tiltPhase = [footTilt(2)*cos(footTilt(1)) footTilt(2)*sin(footTilt(1)) footYaw];

end

% Convert a tilt phase orientation into a specification of foot yaw and absolute foot tilt relative to N
function [footYaw, footTilt] = FootYawTiltFromTiltPhase(tiltPhase)

	% Calculate the required foot yaw and absolute foot tilt
	footYaw = tiltPhase(3);
	footTilt = [atan2(tiltPhase(2), tiltPhase(1)) sqrt(tiltPhase(1)*tiltPhase(1) + tiltPhase(2)*tiltPhase(2))];

end

% Convert a tilt phase orientation into a quaternion foot orientation relative to N
function [qNF, qBF, tiltNF] = QuatFromTiltPhase(tiltPhase, qBN)

	% Calculate the foot orientation relative to N
	[footYaw, footTilt] = FootYawTiltFromTiltPhase(tiltPhase);
	if nargout >= 2
		[qNF, qBF, tiltNF] = QuatFromFootYawTilt(footYaw, footTilt, qBN);
	else
		qNF = QuatFromFootYawTilt(footYaw, footTilt);
	end

end

% Convert a quaternion foot orientation relative to N into a tilt phase orientation
function [tiltPhase, tiltNF] = TiltPhaseFromQuat(qNF)

	% Calculate the required tilt phase
	[footYaw, footTilt, tiltNF] = FootYawTiltFromQuat(qNF);
	tiltPhase = TiltPhaseFromFootYawTilt(footYaw, footTilt);

end

% Convert a tilt phase velocity vector into an angular velocity vector (tiltPhaseVel is [cdot sdot pdot], footTilt is [gamtilde alpha] absolute)
function [angVelN, angVelB] = AngFromTiltPhaseVel(tiltPhaseVel, footTilt, qBN)

	% Precalculate trigonometric terms
	cgamtilde = cos(footTilt(1));
	sgamtilde = sin(footTilt(1));
	alpha = footTilt(2);
	if alpha == 0
		S = 1;
		C = 0;
	else
		S = sin(alpha)/alpha;
		C = (1 - cos(alpha))/alpha;
	end

	% Calculate the tilt velocity parameters
	dalpha = tiltPhaseVel(1)*cgamtilde + tiltPhaseVel(2)*sgamtilde;
	dagamma = tiltPhaseVel(2)*cgamtilde - tiltPhaseVel(1)*sgamtilde - alpha*tiltPhaseVel(3);
	dpsi = tiltPhaseVel(3);

	% Calculate the corresponding angular velocity
	angVelN = [cgamtilde*dalpha - S*dagamma*sgamtilde, sgamtilde*dalpha + S*dagamma*cgamtilde, dpsi + C*dagamma];

	% Calculate the angular velocity relative to B
	if nargout >= 2
		angVelB = QuatRotVec(qBN, angVelN);
	end

end

% Calculate the lambda range that can be adjusted by to satisfy the minimum leg retraction, assuming the corresponding hip yaw
function [lambdaMin, lambdaMax] = CalcAdjustLambdaRange(IP, phiz, adjustHat, lmaxsq, Ldbl, hx, hyl)

	% Precalculate values
	Cz = cos(phiz) - 1;
	sz = sin(phiz);
	h = [hx*Cz-hyl*sz hx*sz+hyl*Cz Ldbl]; % Hip PR point in anklePos coordinates

	% Calculate the allowed lambda range that respects the maximum leg length
	hpdiff = h - IP.anklePos;
	hpdotm = dot(hpdiff,adjustHat);
	perp = hpdiff - hpdotm*adjustHat;
	dsq = perp(1)*perp(1) + perp(2)*perp(2) + perp(3)*perp(3);
	lambdaH = sqrt(coerceMin(lmaxsq - dsq, 0));
	lambdaMin = hpdotm - lambdaH;
	lambdaMax = hpdotm + lambdaH;

end

% Adjust the IP profile and MC to satisfy minimum leg retraction, nominal hip height and maximum hip height considerations
function [adjustVec] = AdjustRetraction(IP, MC, adjustHat, heightHat, hipHeightMax, RM, config) % Note: adjustHat and heightHat MUST be unit vectors

	% Constants
	maxIts = 10;
	Tol = 1e-9;

	% Robot dimensions
	Ldbl = 2.0*RM.legLinkLength;
	F = Ldbl + RM.footOffsetZ;
	hx = RM.hipOffsetX;
	hy = RM.hipOffsetY; % Note: Hip offset is hy for left leg, and -hy for right leg (i.e. always limbSign*hy)

	% Calculate the maximum allowed leg length
	lmax = Ldbl*(1 - config.minLegRetraction);
	lmaxsq = lmax*lmax;

	% Loop ranges
	dim = size(IP);
	LS = dim(1):-1:1;
	NS = dim(2):-1:1;

	% Preallocate arrays
	lambda = NaN;
	lambdaMin = nan(dim);
	lambdaMax = nan(dim);

	% Initialise variables
	newIP = IP;
	phiz = zeros(dim);
	converged = false;
	bestDelta = Inf;
	bestLambda = NaN;

	% Numerically solve for a lambda that just satisfies the minimum leg retraction requirement
	for k = 1:maxIts

		% Save the old keypoint lambda ranges
		lambdaOld = lambda;
		lambdaMinOld = lambdaMin;
		lambdaMaxOld = lambdaMax;

		% Calculate the keypoint lambda ranges based on the assumed values of hip yaw
		for l = LS
			limbSign = LimbSign(l);
			hyl = limbSign*hy;
			for n = NS
				[lambdaMin(l,n), lambdaMax(l,n)] = CalcAdjustLambdaRange(IP(l,n), phiz(l,n), adjustHat, lmaxsq, Ldbl, hx, hyl);
			end
		end

		% Calculate the minimum possible value of lambda that satisfies all ranges (if possible)
		lambdaIntMin = max(lambdaMin(:));
		lambdaIntMax = min(lambdaMax(:));
		if lambdaIntMin > lambdaIntMax
			lambda = 0.5*(lambdaIntMin + lambdaIntMax);
		else
			lambda = lambdaIntMin;
		end

		% Check whether the process has converged
		if k > 1
			deltaLambdaRange = [abs(lambdaMin(:) - lambdaMinOld(:)); abs(lambdaMax(:) - lambdaMaxOld(:))];
			maxDeltaLambdaRange = max(deltaLambdaRange);
			if maxDeltaLambdaRange <= bestDelta
				bestDelta = maxDeltaLambdaRange;
				bestLambda = lambdaOld;
			end
			if maxDeltaLambdaRange < Tol
				converged = true;
				break
			end
		end

		% Update the assumed values of hip yaw
		if k < maxIts
			for l = LS
				limbSign = LimbSign(l);
				for n = NS
					newIP(l,n).anklePos = IP(l,n).anklePos + lambda*adjustHat;
					phiz(l,n) = CalcHipYaw(newIP(l,n), limbSign, RM);
				end
			end
		end

	end

	% Choose the best seen lambda if the process did not converge
	if ~converged
		lambda = bestLambda;
	end

	% Calculate the minimum allowed lambda to use based on hip height restrictions
	hipCentrePos = [-hx 0 Ldbl];
	lambdaMin = (dot(hipCentrePos - MC, heightHat) - F*min(config.nomHipHeight, hipHeightMax)) / dot(adjustHat, heightHat);

	% Select the final lambda to use and return the associated adjustment vector
	lambda = coerceMin(lambda, lambdaMin);
	adjustVec = lambda*adjustHat;

end

% Evaluate the final abstract cubic splines for one leg for a particular limb phase
function [AP, coerced] = EvalAbsSpline(finalU, finalC, mu, limbSign, config)

	% Select the correct cubic spline interval to calculate with
	kk = NaN;
	for k = 1:8
		nextk = mod(k,8) + 1;
		u = picutMod(mu - finalU(k));
		h = picutMod(finalU(nextk) - finalU(k));
		if u <= h
			kk = k;
			break;
		end
	end

	% Error checking
	if isnan(kk)
		error('Logical error in choosing the correct cubic spline!');
	end

	% Construct the required abstract pose
	fields = fieldnames(finalC);
	for f = 1:numel(fields)
		field = fields{f};
		coeff = finalC.(field);
		AP.(field) = coeff(1,kk) + u*(coeff(2,kk) + u*(coeff(3,kk) + u*coeff(4,kk)));
	end

	% Soft coerce the evaluated abstract space pose
	coerced = false(1,5);
	[angleX, coerced(1)] = coerceSoft(AP.angleX/limbSign, config.legAngleXMin, config.legAngleXMax, config.legAngleXBuf);
	[AP.angleY, coerced(2)] = coerceSoft(AP.angleY, config.legAngleYMin, config.legAngleYMax, config.legAngleYBuf);
	[angleZ, coerced(3)] = coerceSoft(AP.angleZ/limbSign, config.legAngleZMin, config.legAngleZMax, config.legAngleZBuf);
	[footAngleX, coerced(4)] = coerceSoft(AP.footAngleX/limbSign, config.footAngleXMin, config.footAngleXMax, config.footAngleXBuf);
	[AP.footAngleY, coerced(5)] = coerceSoft(AP.footAngleY, config.footAngleYMin, config.footAngleYMax, config.footAngleYBuf);
	AP.retraction = coerceSoft(AP.retraction, config.legRetMin, config.legRetMax, config.legRetBuf);
	AP.angleX = limbSign*angleX;
	AP.angleZ = limbSign*angleZ;
	AP.footAngleX = limbSign*footAngleX;
	coerced = any(coerced);

end

% Initialise figure function
function [fig] = InitFigure(fig, lims, txtOff, hipCentrePos, Hvec)

	% The origin of the figure coordinate system is directly below the hip centre point (assumed CoM),
	% and z = 0 corresponds to the height of the ankle point in the zero position (i.e. z = 0 is twice
	% the leg link length below the hip points).

	% Reset the figure
	figure(fig);
	fig = fig + 1;
	clf;

	% Hold the plot
	hold on;

	% Plot axes arrows
	arrowSize = 0.5*min(lims(2), lims(4));
	arrow3([0 0 lims(5)], [arrowSize 0 0], 0, 0, [0 0 1], '-', 'Color', 'k');
	arrow3([0 0 lims(5)], [0 arrowSize 0], 0, 0, [0 0 1], '-', 'Color', 'k');
	arrow3([0 0 lims(5)], [0 0 1.2*arrowSize], 0, 0, [1 0 0], '-', 'Color', 'k');
	text(arrowSize + txtOff, 0, lims(5), 'x', 'HorizontalAlignment', 'center', 'Color', 'k');
	text(0, arrowSize + txtOff, lims(5), 'y', 'HorizontalAlignment', 'center', 'Color', 'k');
	text(0, 0, lims(5) + 1.2*arrowSize + 1.5*txtOff, 'z', 'HorizontalAlignment', 'center', 'Color', 'k');

	% Plot the hip points
	tmp = [hipCentrePos+Hvec; hipCentrePos-Hvec];
	plot3(tmp(:,1), tmp(:,2), tmp(:,3), 'ko', 'MarkerFaceColor', 'k');

end

% Finalise figure function
function FinaliseFigure(lims)

	% Unhold the plot
	hold off

	% Finalise the plot
	axis equal;
	xlim(lims(1:2));
	ylim(lims(3:4));
	zlim(lims(5:6));
	xlabel('x \rightarrow');
	ylabel('y \rightarrow');
	zlabel('z \rightarrow');
	view(-75, 66);
	grid on;

end

% Generate spline data from a set of coefficients (coeff is 4xN with entries [a0; a1; a2; a3], h is an N-vector, cst/csx/csv/csa/csj are DxN)
function [cst, csx, csv, csa, csj] = GetCubicSplineData(coeff, h, D)

	% Default arguments
	if nargin < 3
		D = 101;
	end

	% Cubic spline function
	evalcs = @(c,u) c(1) + u.*(c(2) + u.*(c(3) + u*c(4)));

	% Get the number of splines
	N = size(coeff,2);

	% Generate the time vector
	csthat = linspace(0, 1, D)';

	% Initialise variables
	cst = nan(D,N);
	csx = nan(D,N);
	csv = nan(D,N);
	csa = nan(D,N);
	csj = nan(D,N);

	% Process each spline
	for k = 1:N
		cst(:,k) = csthat*h(k);
		c = coeff(1:4,k);
		dc = [coeff(2:4,k).*[1; 2; 3]; 0];
		ddc = [coeff(3:4,k).*[2; 6]; 0; 0];
		dddc = [coeff(4,k)*6; 0; 0; 0];
		csx(:,k) = evalcs(c, cst(:,k));
		csv(:,k) = evalcs(dc, cst(:,k));
		csa(:,k) = evalcs(ddc, cst(:,k));
		csj(:,k) = evalcs(dddc, cst(:,k));
	end

end

% Plot a set of cubic splines
function [fig] = PlotCubicSplineData(fig, type, U, X, V, cst, csx, csv, csa, csj)

	% Reset the figure
	figure(fig);
	fig = fig + 1;
	clf;

	% Get number of spline segments (N) and number of parallel spline curves (M)
	N = size(U,1) - 1;
	M = size(cst,2) / N;

	% Clarify the type string
	if ~isempty(type)
		type = [type ' '];
	end

	% Position plot
	ax = subplot(2,2,1);
	ax.ColorOrderIndex = 1;
	plot(U, X, 'x');
	hold on;
	for k = 1:N
		j = (k - 1) + (1:N:N*M);
		ax.ColorOrderIndex = 1;
		plot(U(k) + cst(:,j), csx(:,j), '-');
	end
	hold off;
	xlim([-pi pi]);
	xlabel('Limb phase \mu');
	title([type 'Position']);
	grid on;

	% Velocity plot
	ax = subplot(2,2,2);
	ax.ColorOrderIndex = 1;
	plot(U, V, 'x');
	hold on;
	for k = 1:N
		j = (k - 1) + (1:N:N*M);
		ax.ColorOrderIndex = 1;
		plot(U(k) + cst(:,j), csv(:,j), '-');
	end
	hold off;
	xlim([-pi pi]);
	xlabel('Limb phase \mu');
	title([type 'Velocity']);
	grid on;

	% Acceleration plot
	ax = subplot(2,2,3);
	ax.Box = 'on';
	hold on;
	for k = 1:N
		j = (k - 1) + (1:N:N*M);
		ax.ColorOrderIndex = 1;
		plot(U(k) + cst(:,j), csa(:,j), '-');
	end
	hold off;
	xlim([-pi pi]);
	xlabel('Limb phase \mu');
	title([type 'Acceleration']);
	grid on;

	% Jerk plot
	ax = subplot(2,2,4);
	ax.Box = 'on';
	hold on;
	for k = 1:N
		j = (k - 1) + (1:N:N*M);
		ax.ColorOrderIndex = 1;
		plot(U(k) + cst(:,j), csj(:,j), '-');
	end
	hold off;
	xlim([-pi pi]);
	xlabel('Limb phase \mu');
	title([type 'Jerk']);
	grid on;

end

% Debug print function
function DebugPrint(var, str, byfield)

	% Input arguments
	if nargin < 2
		str = inputname(1);
	end
	if nargin < 3
		byfield = false;
	end

	% Display the required variable
	if isstruct(var) && numel(var) > 1
		for i = 1:size(var,1)
			for j = 1:size(var,2)
				for k = 1:size(var,3)
					if byfield
						varnames = fieldnames(var(i,j,k));
						for m = 1:numel(varnames)
							fieldname = varnames{m};
							disp([str '(' num2str(i) ',' num2str(j) ',' num2str(k) ').' fieldname ' =']);
							disp(var(i,j,k).(fieldname));
						end
					else
						disp([str '(' num2str(i) ',' num2str(j) ',' num2str(k) ') =']);
						disp(var(i,j,k));
					end
				end
			end
		end
	else
		disp([str ' =']);
		disp(var);
	end

end
% EOF