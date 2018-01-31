% CalcArmAngle.m - Philipp Allgeuer - 15/02/17
% Calculate the arm trajectories.
%
% function [out] = CalcArmAngle(in, config, RM)
%
% in     ==> Struct of inputs to the open loop gait
% config ==> Configuration variables to use (see ConfigVars)
% RM     ==> Robot model to use (see RobotModel)
% out    ==> Struct of outputs
%
% The struct of inputs 'in' is expected to have the following fields:
% gcv:         Desired dimensionless walking velocity [gcvX gcvY gcvZ] in the x, y and yaw axes, relative to the
%              body-fixed axes untilted relative to the true ground (i.e. the true S plane)
% fusedPitchN: Fused pitch of the body-fixed axes B relative to the nominal ground plane N
% leanTilt:     Upper body leaning tilt [gamma alpha] relative to the nominal ground plane N
% armTilt:     Desired arm tilt rotation [gamma alpha] of the arms relative to the nominal ground plane N
% config:      Configuration variables to use (see ConfigVars)
% RM:          Robot model to use (see RobotModel)
%
function [out] = CalcArmAngle(in, config, RM)

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
	in = SetIfMissing(in, 'leanTilt', [1.0 0.10]);
	in = SetIfMissing(in, 'armTilt', [0.6 0.3]);
	if nargin < 2
		config = ConfigVars;
	end
	if nargin < 3
		RM = RobotModel;
	end
	
	% Gait phase input
	if isfield(in, 'wavemu')
		u = in.wavemu(:);
	else
		u = linspace(-pi, pi, 201)';
	end
	N = numel(u);

	% General variables
	normTol = 64*eps;

	% Plotting variables
	fig = 201;
	lims = [-0.30 0.30 -0.40 0.40 0 0.35];
	anglePlotLimsY = [-pi/2 pi/2];
	txtOff = 0.008;

	%% Initialisation and setup

	% Calculate the relevant nominal ground plane rotations and axes
	qNB = QuatFromFused([0 in.fusedPitchN 0 1]);
	qBN = QuatInv(qNB);
	RNB = RotmatFromQuat(qNB);
	BzN = RNB(3,:);

	% Calculate the lean rotation
	qNL = QuatFromTilt([0 in.leanTilt(1) -in.leanTilt(2)]);
	qBL = QuatMult(qBN, qNL);
	qLB = QuatInv(qBL);
	RLB = RotmatFromQuat(qLB);

	% Common variables
	D = config.doubleSupportPhaseLen;
	L = RM.armLinkLength;
	Lvec = [0 0 2.0*L];
	S = 0.5*RM.shoulderWidth;
	Svec = [0 S 0];

	% Set up data to pass to the PlotAngleFigure function
	plotAngleData = struct();
	plotAngleData.D = D;
	plotAngleData.limY = anglePlotLimsY;

	%% Calculate the arm trajectories

	% Initialise the common motion data
	CMD = struct();
	CMD.gcvX = in.gcv(1);
	CMD.gcvY = in.gcv(2);
	CMD.gcvZ = in.gcv(3);
	CMD.absGcvX = abs(in.gcv(1));
	CMD.absGcvY = abs(in.gcv(2));
	CMD.absGcvZ = abs(in.gcv(3));
	CMD.shoulderPos = Lvec;

	% Calculate the CoM tilt axis
	comTiltAxisL = [cos(in.armTilt(1)) sin(in.armTilt(1)) 0];
	comTiltAxisB = QuatRotVec(qBL, comTiltAxisL); % For plotting and testing

	% Calculate the arm trajectory data
	for k = N:-1:1

		% Common motion data
		CMD.gaitPhase = picut(u(k));
		CMD.oppGaitPhase = picut(u(k) + pi);

		% Calculate the trajectories for each arm
		for I = [2 1]; limbSign = LimbSign(I);

			% Common motion data
			CMD.limbSign = limbSign;
			if config.leftLegFirst == (limbSign >= 0)
				CMD.limbPhase = CMD.oppGaitPhase;
			else
				CMD.limbPhase = CMD.gaitPhase;
			end

			%
			% Raw pose
			%

			% Calculate the raw pose
			raw.AAP(I,k) = AbstractRawMotion(CMD, config);

			% Compute the remaining fields of the limited pose
			raw.limbPhase(I,k) = CMD.limbPhase;
			raw.AJP(I,k) = ArmJointFromAbs(raw.AAP(I,k));
			[raw.AIP(I,k), raw.elbowPos(I,1:3,k)] = ArmInvFromJoint(raw.AJP(I,k), RM);
			raw.CoM(I,1:3,k) = 0.25*CMD.shoulderPos + 0.50*raw.elbowPos(I,:,k) + 0.25*raw.AIP(I,k).handPos;

			%
			% Tilted pose
			%

			% Calculate the nominal CoM line (vector relative to the shoulder in N coordinates) after the application of arm tilt
			rawCoMLineB = raw.CoM(I,:,k) - CMD.shoulderPos;
			comTiltAngleL = coerceSoftAbs(in.armTilt(2), config.armTiltAngleMax, config.armTiltAngleBuf);
			qLC = [cos(0.5*comTiltAngleL) sin(0.5*comTiltAngleL)*comTiltAxisL];
			CoMLineN = QuatRotVec(QuatMult(qNL, qLC, qLB), rawCoMLineB); % Convert to L coordinates then apply the rotation from L to C and convert to N coordinates

			% Make the nominal CoM line a unit vector
			CoMLineNNorm = norm(CoMLineN);
			if CoMLineNNorm >= normTol
				CoMLineN = CoMLineN(:)' / CoMLineNNorm;
			else
				warning('It should not be possible for the norm of CoMLineN to be zero!');
				CoMLineN = [0 0 -1];
			end

			% Soft-limit the absolute tilt of the CoM line relative to straight down in N coordinates in an elliptical fashion
			CoMLineYawN = atan2(CoMLineN(2), CoMLineN(1)); % In (-pi,pi] where 0 is tilt in the direction of xN
			CoMLineTiltN = acos(coerceAbs(-CoMLineN(3), 1.0)); % In [0, pi]
			a = config.comLineTiltMaxP; % Max absolute tilt along the xN direction (must have a > 0)
			b = config.comLineTiltMaxR; % Max absolute tilt along the yN direction (must have b > 0)
			m = config.comLineTiltBuf;  % Soft coercion buffer value for the absolute tilt (must have m <= a,b)
			cosCoMYaw = cos(CoMLineYawN);
			sinCoMYaw = sin(CoMLineYawN);
			CoMLineTiltNMax = a*b / sqrt(a*a*sinCoMYaw*sinCoMYaw + b*b*cosCoMYaw*cosCoMYaw); % In [a,b]
			if CoMLineTiltN > CoMLineTiltNMax - m
				CoMLineTiltN = coerceSoftMax(CoMLineTiltN, CoMLineTiltNMax, m);
				cosCoMTilt = cos(CoMLineTiltN);
				sinCoMTilt = sin(CoMLineTiltN);
				CoMLineN = [cosCoMYaw*sinCoMTilt sinCoMYaw*sinCoMTilt -cosCoMTilt];
			end

			% Adjust the CoM line in N coordinates to respect an inwards y-bound
			yhat = limbSign*CoMLineN(2);
			if yhat < config.comLineYHatMin + config.comLineYHatBuf
				yhat = coerceSoftMin(yhat, config.comLineYHatMin, config.comLineYHatBuf);
				yhat = coerceAbs(limbSign*yhat, 1.0);
				yhatsq = yhat*yhat;
				xysqsum = CoMLineN(1)*CoMLineN(1) + yhatsq;
				if xysqsum > 1.0
					CoMLineN(1) = sgn(CoMLineN(1)) * sqrt(1.0 - yhatsq);
					CoMLineN(3) = 0.0;
				else
					CoMLineN(3) = -sqrt(1.0 - xysqsum); % Always pick the solution in the bottom N-hemisphere
				end
				CoMLineN(2) = yhat;
			end

			% Convert the CoM line to body-fixed coordinates
			CoMLineB = QuatRotVec(qBN, CoMLineN);

			% Perform inverse kinematics to place the arm CoM on the CoM line, with an arm retraction of at least that of the raw pose (i.e. same or longer arm)
			alphaNom = acos(coerce(1 - raw.AAP(I,k).retraction, 0.0, 1.0)); % In [0,pi/2]
			shoulderRollLim = [config.shoulderRollMax config.shoulderRollMaxBuf];
			tilted.AJP(I,k) = ArmJointFromCoM(CoMLineB, alphaNom, shoulderRollLim);

			% Compute the remaining fields of the tilted pose
			tilted.limbPhase(I,k) = raw.limbPhase(I,k);
			tilted.AAP(I,k) = ArmAbsFromJoint(tilted.AJP(I,k));
			[tilted.AIP(I,k), tilted.elbowPos(I,1:3,k)] = ArmInvFromJoint(tilted.AJP(I,k), RM);
			tilted.CoM(I,1:3,k) = 0.25*CMD.shoulderPos + 0.50*tilted.elbowPos(I,:,k) + 0.25*tilted.AIP(I,k).handPos;

			%
			% Limited pose
			%

			% Enforce joint limits
			limited.AJP(I,k) = tilted.AJP(I,k);
			limited.AJP(I,k).shoulderPitch = coerceSoft(limited.AJP(I,k).shoulderPitch, config.shoulderPitchMin, config.shoulderPitchMax, config.shoulderPitchBuf);

			% Compute the remaining fields of the limited pose
			limited.limbPhase(I,k) = tilted.limbPhase(I,k);
			limited.AAP(I,k) = ArmAbsFromJoint(limited.AJP(I,k));
			[limited.AIP(I,k), limited.elbowPos(I,1:3,k)] = ArmInvFromJoint(limited.AJP(I,k), RM);
			limited.CoM(I,1:3,k) = 0.25*CMD.shoulderPos + 0.50*limited.elbowPos(I,:,k) + 0.25*limited.AIP(I,k).handPos;

		end
	end
	
	% Calculate output waveform data
	wave = struct();
	wave.mu = u;
	wave.AJPP = {SAFromAS(limited.AJP(1,:)), SAFromAS(limited.AJP(2,:))};

	%% Plot the arm trajectories

	% Initialise figure
	fig = InitFigure(fig, lims, txtOff, Lvec, Svec);

	% Figure title
	title(sprintf('Arm trajectories: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));

	% Plot the raw hand trajectories
	rawLeftHand = cell2mat({raw.AIP(1,:).handPos}') + Svec;
	rawRightHand = cell2mat({raw.AIP(2,:).handPos}') - Svec;
	rawLeftCoM = squeeze(raw.CoM(1,:,:))' + Svec;
	rawRightCoM = squeeze(raw.CoM(2,:,:))' - Svec;
	plot3([rawLeftHand(:,1) rawRightHand(:,1)], [rawLeftHand(:,2) rawRightHand(:,2)], [rawLeftHand(:,3) rawRightHand(:,3)], 'b');
	plot3([rawLeftCoM(:,1) rawRightCoM(:,1)], [rawLeftCoM(:,2) rawRightCoM(:,2)], [rawLeftCoM(:,3) rawRightCoM(:,3)], 'Color', [0.0 0.5 1.0]);

	% Plot the tilted trajectories
	tiltedLeftHand = cell2mat({tilted.AIP(1,:).handPos}') + Svec;
	tiltedRightHand = cell2mat({tilted.AIP(2,:).handPos}') - Svec;
	tiltedLeftCoM = squeeze(tilted.CoM(1,:,:))' + Svec;
	tiltedRightCoM = squeeze(tilted.CoM(2,:,:))' - Svec;
	plot3([tiltedLeftHand(:,1) tiltedRightHand(:,1)], [tiltedLeftHand(:,2) tiltedRightHand(:,2)], [tiltedLeftHand(:,3) tiltedRightHand(:,3)], 'r');
	plot3([tiltedLeftCoM(:,1) tiltedRightCoM(:,1)], [tiltedLeftCoM(:,2) tiltedRightCoM(:,2)], [tiltedLeftCoM(:,3) tiltedRightCoM(:,3)], 'Color', [1.0 0.5 0.0]);

	% Plot the limited trajectories
	limitedLeftHand = cell2mat({limited.AIP(1,:).handPos}') + Svec;
	limitedRightHand = cell2mat({limited.AIP(2,:).handPos}') - Svec;
	limitedLeftCoM = squeeze(limited.CoM(1,:,:))' + Svec;
	limitedRightCoM = squeeze(limited.CoM(2,:,:))' - Svec;
	plot3([limitedLeftHand(:,1) limitedRightHand(:,1)], [limitedLeftHand(:,2) limitedRightHand(:,2)], [limitedLeftHand(:,3) limitedRightHand(:,3)], 'Color', [0.5 0.5 0.5]);
	plot3([limitedLeftCoM(:,1) limitedRightCoM(:,1)], [limitedLeftCoM(:,2) limitedRightCoM(:,2)], [limitedLeftCoM(:,3) limitedRightCoM(:,3)], 'Color', [0.5 0.5 0.5]);

	% Plot link lines between the raw and tilted trajectories
	index = 1:ceil(N/8):N;
	plot3([rawLeftHand(index,1) tiltedLeftHand(index,1)]', [rawLeftHand(index,2) tiltedLeftHand(index,2)]', [rawLeftHand(index,3) tiltedLeftHand(index,3)]', '-', 'Color', [0.15 0.70 0.15]);
	plot3([rawRightHand(index,1) tiltedRightHand(index,1)]', [rawRightHand(index,2) tiltedRightHand(index,2)]', [rawRightHand(index,3) tiltedRightHand(index,3)]', '-', 'Color', [0.15 0.70 0.15]);
	plot3([rawLeftCoM(index,1) tiltedLeftCoM(index,1)]', [rawLeftCoM(index,2) tiltedLeftCoM(index,2)]', [rawLeftCoM(index,3) tiltedLeftCoM(index,3)]', '-', 'Color', [0.15 0.70 0.15]);
	plot3([rawRightCoM(index,1) tiltedRightCoM(index,1)]', [rawRightCoM(index,2) tiltedRightCoM(index,2)]', [rawRightCoM(index,3) tiltedRightCoM(index,3)]', '-', 'Color', [0.15 0.70 0.15]);

	% Plot the arm tilt vector
	arrow3([0 0 0.08], 0.2*comTiltAxisB, 0, 0, [0 0 1], '-', 'Color', 'g');

	% Plot the arm pose for one of the keypoints
	K = index(2);
	plot3([0 tilted.elbowPos(1,1,K) tilted.AIP(1,K).handPos(1)], S + [0 tilted.elbowPos(1,2,K) tilted.AIP(1,K).handPos(2)], [2*L tilted.elbowPos(1,3,K) tilted.AIP(1,K).handPos(3)], 'mo-', 'LineWidth', 1.5);
	plot3([0 tilted.AIP(1,K).handPos(1)], S + [0 tilted.AIP(1,K).handPos(2)], [2*L tilted.AIP(1,K).handPos(3)], 'k:', 'LineWidth', 1.5);
	plot3(tilted.CoM(1,1,K), S + tilted.CoM(1,2,K), tilted.CoM(1,3,K), 'mx');
	plot3([0 tilted.elbowPos(2,1,K) tilted.AIP(2,K).handPos(1)], -S + [0 tilted.elbowPos(2,2,K) tilted.AIP(2,K).handPos(2)], [2*L tilted.elbowPos(2,3,K) tilted.AIP(2,K).handPos(3)], 'mo-', 'LineWidth', 1.5);
	plot3([0 tilted.AIP(2,K).handPos(1)], -S + [0 tilted.AIP(2,K).handPos(2)], [2*L tilted.AIP(2,K).handPos(3)], 'k:', 'LineWidth', 1.5);
	plot3(tilted.CoM(2,1,K), -S + tilted.CoM(2,2,K), tilted.CoM(2,3,K), 'mx');

	% Finalise figure
	FinaliseFigure(lims);

	%% Plot a 2D version of the arm CoM adjustments (relative to L plane)

	% Reset the figure
	figure(fig);
	fig = fig + 1;
	clf;

	% Hold the plot
	hold on;

	% Plot axes arrows
	arrowSize = 0.4*min(lims(2), lims(4));
	arrow(0, [arrowSize 0], 0, 0, 'Color', 'k');
	arrow(0, [0 arrowSize], 0, 0, 'Color', 'k');
	text(arrowSize + txtOff, 0, 'x', 'HorizontalAlignment', 'center', 'Color', 'k');
	text(0, arrowSize + txtOff, 'y', 'HorizontalAlignment', 'center', 'Color', 'k');

	% Plot the arm tilt vector
	arrow(0, 0.12*comTiltAxisL(1:2), 0, 0, 'Color', 'g');

	% Plot lines perpendicular to the tilt vector
	perpAxis = [-comTiltAxisL(2) comTiltAxisL(1)];
	u = linspace(-0.3, 0.3, 31);
	perpLineX = [u*comTiltAxisL(1) - 0.3*perpAxis(1); u*comTiltAxisL(1) + 0.3*perpAxis(1)];
	perpLineY = [u*comTiltAxisL(2) - 0.3*perpAxis(2); u*comTiltAxisL(2) + 0.3*perpAxis(2)];
	plot(perpLineX, perpLineY, '-', 'Color', [0.8 0.8 0.8]);

	% Transform and project the CoM waveforms into the L plane
	rawLeftCoML = rawLeftCoM*RLB';
	rawRightCoML = rawRightCoM*RLB';
	tiltedLeftCoML = tiltedLeftCoM*RLB';
	tiltedRightCoML = tiltedRightCoM*RLB';
	limitedLeftCoML = limitedLeftCoM*RLB';
	limitedRightCoML = limitedRightCoM*RLB';

	% Plot the CoM trajectories
	plot([rawLeftCoML(:,1) rawRightCoML(:,1)], [rawLeftCoML(:,2) rawRightCoML(:,2)], 'Color', [0.0 0.5 1.0]);
	plot([tiltedLeftCoML(:,1) tiltedRightCoML(:,1)], [tiltedLeftCoML(:,2) tiltedRightCoML(:,2)], 'Color', [1.0 0.5 0.0]);
	plot([limitedLeftCoML(:,1) limitedRightCoML(:,1)], [limitedLeftCoML(:,2) limitedRightCoML(:,2)], 'Color', [0.5 0.5 0.5]);

	% Plot link lines between the raw and tilted trajectories
	index = 1:ceil(N/12):N;
	plot([rawLeftCoML(index,1) tiltedLeftCoML(index,1)]', [rawLeftCoML(index,2) tiltedLeftCoML(index,2)]', '-', 'Color', [0.15 0.70 0.15]);
	plot([rawRightCoML(index,1) tiltedRightCoML(index,1)]', [rawRightCoML(index,2) tiltedRightCoML(index,2)]', '-', 'Color', [0.15 0.70 0.15]);

	% Unhold the plot
	hold off

	% Finalise the plot
	axis equal;
	xlim(0.50*lims(1:2));
	ylim(0.75*lims(3:4));
	xlabel('x \rightarrow');
	ylabel('y \rightarrow');
	view([-90 90]);
	box on;
	grid on;

	%% Plot the required angle space curves

	% Plot the raw angle space curves for the left and right arms
	fig = PlotAngleFigure(fig, raw, 1, plotAngleData);
	title(sprintf('Raw left arm angle space profile: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));
	fig = PlotAngleFigure(fig, raw, 2, plotAngleData);
	title(sprintf('Raw right arm angle space profile: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));

	% Plot the tilted angle space curves for the left and right arms
	fig = PlotAngleFigure(fig, tilted, 1, plotAngleData);
	title(sprintf('Tilted left arm angle space profile: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));
	fig = PlotAngleFigure(fig, tilted, 2, plotAngleData);
	title(sprintf('Tilted right arm angle space profile: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));

	% Plot the limited angle space curves for the left and right arms
	fig = PlotAngleFigure(fig, limited, 1, plotAngleData);
	title(sprintf('Limited left arm angle space profile: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));
	fig = PlotAngleFigure(fig, limited, 2, plotAngleData);
	title(sprintf('Limited right arm angle space profile: %.2f %.2f %.2f', CMD.gcvX, CMD.gcvY, CMD.gcvZ));

	%% Testing

% 	% Testing constants
% 	TestTol = 1e-13;
% 
% 	% Test that the CoMs have moved in the plane perpendicular to the arm tilt vector
% 	err = zeros(1,0);
% 	for k=1:size(tilted.CoM,3)
% 		err(end+1) = dot(tilted.CoM(1,:,k) - raw.CoM(1,:,k), comTiltAxisB); %#ok<AGROW>
% 		err(end+1) = dot(tilted.CoM(2,:,k) - raw.CoM(2,:,k), comTiltAxisB); %#ok<AGROW>
% 	end
% 	if any(abs(err) > TestTol)
% 		warning(['There are errors above the prescribed tolerance for ' num2str(sum(abs(err) > TestTol)) ' indices! Are you at the inward or elliptical CoM line tilt limits?']);
% 	end

	%% Populate the output struct

	% Populate the output struct
	out = struct();
	out.in = in;
	out.fig = fig;
	out.qNB = qNB;
	out.qBN = qBN;
	out.RNB = RNB;
	out.BzN = BzN;
	out.qNL = qNL;
	out.qBL = qBL;
	out.raw = raw;
	out.tilted = tilted;
	out.limited = limited;
	out.wave = wave;

end

% Calculate the raw arm motion as a function of the limb phase
function [AAP] = AbstractRawMotion(CMD, config)

	% Common variables
	D = config.doubleSupportPhaseLen;

	% Start with the abstract arm pose
	AAP = ArmAbstractPose([CMD.limbSign*config.haltArmAngleX config.haltArmAngleY], config.haltArmRetraction);

	% Calculate the limb phase dependent dimensionless swing angle (ranging from -1 to 1)
	if CMD.limbPhase >= D
		swingAngle = -cos(pi*(CMD.limbPhase - D)/(pi - D));   % Sinusoid forwards swing from dimensionless angle -1 to +1 (section from phase D to pi)
	else
		swingAngle = 1.0 - 2.0*(CMD.limbPhase + pi)/(D + pi); % Linear backwards swing from dimensionless angle +1 to -1 (section from phase -pi to D)
	end

	% Apply the sagittal arm swing to the abstract arm pose
	armSagSwingMag = config.armSagSwingMag + CMD.gcvX*config.armSagSwingMagGradX;
	armSagSwing = -swingAngle * armSagSwingMag;
	AAP.angleY = AAP.angleY + armSagSwing;

end

% Initialise figure function
function [fig] = InitFigure(fig, lims, txtOff, Lvec, Svec)

	% The origin of the figure coordinate system is directly below the shoulder centre point,
	% and z = 0 corresponds to the height of the hand point in the zero position (i.e. z = 0
	% is twice the arm link length below the shoulder points).

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
	arrow3([0 0 lims(5)], [0 0 arrowSize], 0, 0, [1 0 0], '-', 'Color', 'k');
	text(arrowSize + txtOff, 0, lims(5), 'x', 'HorizontalAlignment', 'center', 'Color', 'k');
	text(0, arrowSize + txtOff, lims(5), 'y', 'HorizontalAlignment', 'center', 'Color', 'k');
	text(0, 0, arrowSize + 1.5*txtOff, 'z', 'HorizontalAlignment', 'center', 'Color', 'k');

	% Plot the shoulder points
	tmp = [Lvec+Svec; Lvec-Svec];
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
	view(-144, 34);
	grid on;

end

% Plot angle space figure function
function [fig] = PlotAngleFigure(fig, data, limbIndex, plotAngleData)

	% Calculate the support keypoint limb phases for the leg opposite to the arm
	suppKeyLimbPhase = [-pi plotAngleData.D-pi 0.5*(plotAngleData.D-pi) 0 plotAngleData.D];

	% Reset the figure
	figure(fig);
	fig = fig + 1;
	clf;

	% Hold the plot
	hold on;

	% Plot vertical lines for the keypoints
	plot(repmat(suppKeyLimbPhase,2,1), repmat(plotAngleData.limY',1,5), 'b:');

	% Plot the arm trajectory angle data
	xdata = repmat(cell2mat({data.limbPhase(limbIndex,:)})',1,3);
	ydata = [cell2mat({data.AAP(limbIndex,:).angleX}); cell2mat({data.AAP(limbIndex,:).angleY}); cell2mat({data.AAP(limbIndex,:).retraction})]';
	yydata = [cell2mat({data.AJP(limbIndex,:).shoulderPitch}); cell2mat({data.AJP(limbIndex,:).shoulderRoll}); cell2mat({data.AJP(limbIndex,:).elbowPitch})]';
	xdata([(diff(xdata) < 0); false(1,3)]) = NaN;
	h = plot([xdata xdata], [ydata yydata], '-');

	% Label the keypoint phases
	tmpY = plotAngleData.limY(1) + 0.03*(plotAngleData.limY(2) - plotAngleData.limY(1));
	text(suppKeyLimbPhase(1), tmpY, 'A', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');
	text(suppKeyLimbPhase(2), tmpY, 'B', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');
	text(suppKeyLimbPhase(3), tmpY, 'N', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');
	text(suppKeyLimbPhase(4), tmpY, 'C', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');
	text(suppKeyLimbPhase(5), tmpY, 'D', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', 'b');

	% Label the support and swing phases
	tmpY = plotAngleData.limY(1) + 0.97*(plotAngleData.limY(2) - plotAngleData.limY(1));
	text(0.5*(plotAngleData.D-pi), tmpY, 'SUPPORT', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center');
	text(0.5*(plotAngleData.D+pi), tmpY, 'SWING', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center');

	% Unhold the plot
	hold off

	% Finalise the plot
	axis([-pi pi plotAngleData.limY]);
	xlabel('Limb phase \mu');
	ylabel('Value');
	legend(h, 'Arm Angle X', 'Arm Angle Y', 'Arm Retraction', 'Shoulder Pitch', 'Shoulder Roll', 'Elbow Pitch', 'Location', 'SouthEast');
	grid on;

end
% EOF