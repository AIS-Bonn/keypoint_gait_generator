% ConfigVars.m - Philipp Allgeuer - 01/09/16
function [config] = ConfigVars()

	% Initialise config struct
	config = struct();

	% Halt pose
	config.haltArmAngleX = 0.17; % 0.17
	config.haltArmAngleY = -0.15; % -0.15
	config.haltArmRetraction = 0.09; % 0.09
	config.haltFootAngleX = -0.01; % -0.01
	config.haltFootAngleY = -0.055; % -0.055
	config.haltLegAngleX = 0.14; % 0.14
	config.haltLegAngleY = 0.075; % 0.075
	config.haltLegAngleZ = 0; % 0
	config.haltLegRetraction = 0.03; % 0.03

	% Arm motion
	config.armSagSwingMag = 0; % 0
	config.armSagSwingMagGradX = 0.16; % 0.16

	% Leg motion
	config.legExtToAngleYGain = 0.75; % 0.75
	config.legHipAngleXLegExtGain = 0.2; % 0.2
	config.legLatHipSwingBias = 0; % 0
	config.legLatHipSwingMag = 0.02; % 0.02
	config.legLatHipSwingMagGradX = 0; % 0
	config.legLatHipSwingMagGradY = 0; % 0
	config.legLatLeanGradXZBwd = 0; % 0
	config.legLatLeanGradXZFwd = 0; % 0
	config.legLatPushoutMagGradX = 0; % 0
	config.legLatPushoutMagGradY = 0.05; % 0.05
	config.legLatPushoutMagGradZ = 0.05; % 0.05
	config.legLatSwingMagGradY = 0.1; % 0.1
	config.legPushHeight = 0; % 0
	config.legPushHeightGradX = 0; % 0
	config.legRotSwingMagGradZ = 0.15; % 0.15
	config.legRotVPushoutMagGradZ = 0.05; % 0.05
	config.legSagLeanGradAccXBwd = 0; % 0
	config.legSagLeanGradAccXFwd = 0; % 0
	config.legSagLeanGradVelXBwd = 0.005; % 0.005
	config.legSagLeanGradVelXFwd = 0.05; % 0.05
	config.legSagLeanGradVelZAbs = 0; % 0
	config.legSagSwingMagGradXBwd = 0.1; % 0.1
	config.legSagSwingMagGradXFwd = 0.1; % 0.1
	config.legStepHeight = 0.07; % 0.07
	config.legStepHeightGradX = 0.025; % 0.025
	config.legStepHeightGradY = 0.02; % 0.02

	% Gait phase
	config.doubleSupportPhaseLen = 0.6; % 0.6
	config.filletPushPhaseLen = 0.4; % 0.4
	config.filletStepPhaseLen = 0.4; % 0.4
	config.startBlendPhaseLen = 3.14159; % 3.14159
	config.stopBlendPhaseLen = 3.14159; % 3.14159
	config.suppTransStartRatio = 1; % 1
	config.suppTransStopRatio = 1; % 1
	config.swingMinPhaseLen = 1; % 1
	config.swingStartPhaseOffset = 0; % 0
	config.swingStopPhaseOffset = 0.25; % 0.25

	% Pose limits
	config.armAngleXBuf = 0.1; % 0.1
	config.armAngleXMax = 0.8; % 0.8
	config.armAngleXMin = 0.05; % 0.05
	config.armAngleYBuf = 0.15; % 0.15
	config.armAngleYMax = 0.8; % 0.8
	config.armAngleYMin = -0.8; % -0.8
	config.footAngleXBuf = 0.1; % 0.1
	config.footAngleXMax = 0.6; % 0.4
	config.footAngleXMin = -0.6; % -0.4
	config.footAngleYBuf = 0.1; % 0.1
	config.footAngleYMax = 0.7; % 0.5
	config.footAngleYMin = -0.7; % -0.5
	config.legAngleXBuf = 0.1; % 0.1
	config.legAngleXMax = 1.1; % 0.9
	config.legAngleXMin = -0.2; % -0.2
	config.legAngleYBuf = 0.1; % 0.1
	config.legAngleYMax = 1.1; % 0.6
	config.legAngleYMin = -1.1; % -0.5
	config.legAngleZBuf = 0.1; % 0.1
	config.legAngleZMax = 1.1; % 1.1
	config.legAngleZMin = -0.3; % -0.3
	config.legRetBuf = 0.010; % 0.015
	config.legRetMin = 0.015; % 0
	config.legRetMax = 1.0; % 0.5
	config.anklePointZMax = 0.7; % 0.7 (in units of 2L)
	config.footXLegAxisAngleMin = 0.25; % 0.25

	% Tuning variables
	config.tuningNoLegLifting = false; % false
	config.tuningNoLegPushout = false; % false
	config.tuningNoLegHipSwing = false; % false

	% General variables
	config.supportCoeffRange = 1;

	% New general variables
	config.leftLegFirst = true;

	% New arm variables
	config.armTiltAngleMax = 1.5;     % Limits the magnitude of arm tilt that is added to the arms, irrespective of what the initial arm pose is
	config.armTiltAngleBuf = 0.15;    % Softness of the above max
	config.comLineTiltMaxP = 1.4;     % The limit of the tilt angle between the CoM line and straight down in the N plane if the CoM line is purely pitch
	config.comLineTiltMaxR = 1.2;     % The limit of the tilt angle between the CoM line and straight down in the N plane if the CoM line is purely roll
	config.comLineTiltBuf = 0.15;     % Softness of the above max
	config.comLineYHatMin = 0.05;     % Minimum normalised y component of the CoM line (limiting inwards arm motions, outwards is positive for this value)
	config.comLineYHatBuf = 0.10;     % Softness of the above min
	config.shoulderPitchMax = 1.55;   % Simply limits the final shoulder pitch produced by the gait
	config.shoulderPitchMin = -1.55;  % Simply limits the final shoulder pitch produced by the gait
	config.shoulderPitchBuf = 0.1;    % Softness of the above min/max
	config.shoulderRollMax = 1.5;     % Sets the limit for when alpha values are adjusted to avoid further increases in shoulder roll (see shoulderRollLim as input to ArmJointFromCoM)
	config.shoulderRollMaxBuf = 0.2;  % Softness of the above max (see shoulderRollLim as input to ArmJointFromCoM)

	% New leg variables
	config.MCLHipCentreRatio = 0; % Interpolation factor for defining the MCL between pointing in the N direction (0.0) and pointing towards the hip centre point (1.0) (nominally this should kept at 0 in final implementation)
	config.footTiltIsLocalRatio = 0.5; % 0 => Absolute, 1 => Relative
	config.IRatio = 0.5; % 0 => I is N, 1 => I is S
	config.JRatio = 0.5; % 0 => J is N, 1 => J is S
	config.ACAdjustABRatio = 0.5; % 0 => On average all AC adjustment is done by CD, 1 => On average all AC adjustment is done by AB
	config.nomHipHeight = 0.94; % Nominal hip height
	config.FOverNRatio = 0.6; % 0 => Adjusted F is S-above the reconciled F point, 1 => Adjusted F is S-above the reconciled N point
	config.EHeightRatio = 0.2; % 0 => E keypoint is at same S-height as D, 1 => E keypoint is at same S-height as F
	config.GHeightRatio = 0.2; % 0 => G keypoint is at same S-height as A, 1 => G keypoint is at same S-height as F
	config.EPhaseOffset = 0.3; % Phase offset from D forwards to E
	config.GPhaseOffset = 0.3; % Phase offset from A backwards to G
	config.swingOutMaxIwd = 0.03; % Maximum allowed inwards swing out in units of tilt phase
	config.swingOutBuf = 0.03; % Soft coercion buffer for swing out in units of tilt phase
	config.suppCoeffPhaseExtra = 0.25; % Phase added half-half symmetrically to the double support phase to define the interval over which the support coefficient transition occurs

	% Derived variables
	config.minLegRetraction = config.legRetMin + config.legRetBuf;

end
% EOF