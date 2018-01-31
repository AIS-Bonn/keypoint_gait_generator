% Return a struct with the dimensions of the cap gait robot model.
function [RM] = RobotModel()

	% Populate the robot model struct
	RM = struct();
	RM.shoulderWidth = 0.24982; % Perpendicular distance between the two shoulder roll axes
	RM.armLinkLength = 0.16965; % Distance between the shoulder pitch and elbow pitch axes for zero shoulder roll
	RM.hipWidth = 0.11000; % Perpendicular distance between the two hip yaw axes
	RM.hipOffsetX = 0.02275; % Forward x-offset from the hip yaw axis to the hip PR point
	RM.hipOffsetY = 0.00000; % Outward y-offset from the hip yaw axis to the hip PR point (limb sign dependent)
	RM.legLinkLength = 0.19950; % Equivalent distance between the hip pitch and knee pitch, and knee pitch and ankle pitch axes
	RM.footWidth = 0.13156; % Width (y-dimension) of the rectangular bounding box of the bottom surface of the foot
	RM.footLength = 0.21401; % Length (x-dimension) of the rectangular bounding box of the bottom surface of the foot
	RM.footOffsetX = 0.02257; % Forward x-offset from the ankle point to the underfoot geometric centre
	RM.footOffsetY = 0.01093; % Outward y-offset from the ankle point to the underfoot geometric centre (limb sign dependent)
	RM.footOffsetZ = 0.05785; % Downward z-offset from the ankle point to the underfoot geometric centre (including cleats)

	% Additional parameters
	RM.shoulderOffsetZ = 0.01000; % Perpendicular distance between the shoulder pitch and roll axes (positive if the elbow is closer to the roll axis than to the pitch axis)
	RM.upperArmX = 0.02690; % x-offset from shoulder roll axis (point where perp to shoulder pitch axis is) to the elbow pitch axis
	RM.upperArmZ = 0.15750; % z-offset from shoulder roll axis (point where perp to shoulder pitch axis is) to the elbow pitch axis
	RM.lowerArmZ = 0.22480; % Distance (pure z) between the elbow pitch axis and the tip of the hand

end
% EOF