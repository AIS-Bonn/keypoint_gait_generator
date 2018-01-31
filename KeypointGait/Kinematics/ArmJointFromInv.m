% Conversion from inverse arm space to arm joint space
%
% function [AJP] = ArmJointFromInv(AIP, RM)
%
function [AJP] = ArmJointFromInv(AIP, RM)

	% Notes:
	% - The shoulder origin is the fixed point of intersection between the two shoulder joint axes.
	% - The hand origin is the point at the tip of the lower arm.
	% - We assume that the lengths of the upper and lower arms are equal. This common length is denoted L and is given
	%   by RM.armLinkLength.
	% - The zero hand position, defined to be the vector displacement from the shoulder origin to the hand origin in
	%   body-fixed coordinates when the robot is in its zero position, is given by (0,0,-2L).
	% - The displacement from the shoulder origin to the hand origin in body-fixed coordinates is given by the sum of
	%   handPos and the zero hand position, as handPos is defined relative to the zero hand position.
	% - Define the shoulder frame {S} to be a frame located at the shoulder origin, aligned with the body-fixed frame.
	% - Define the hand frame {H} to be a frame located at the hand origin, aligned with the local coordinate system of
	%   the hand. In the zero pose this frame should have zero rotation relative to {S}.

	% Input arguments
	if nargin < 2
		RM = RobotModel;
	end

	% Constants
	Tol = 64*eps;

	% Calculate the coordinates of the hand origin in terms of the shoulder frame {S}
	L = RM.armLinkLength;
	nominalArmLength = 2*L;
	handInShoulderFrame = AIP.handPos - [0 0 nominalArmLength];

	% Calculate the arm length as the distance between the shoulder and hand origins
	armLength = norm(handInShoulderFrame);
	if armLength > nominalArmLength
		handInShoulderFrame = handInShoulderFrame * (nominalArmLength / armLength); % If armLength were zero, this if-block wouldn't be executing...
		armLength = nominalArmLength;
	end

	% Handle the special case of zero arm length
	if armLength < Tol
		AJP = ArmJointPose([0 pi/2], -pi);
		return;
	end

	% Calculate the arm alpha angle based on the arm length
	calpha = coerce(armLength / nominalArmLength, 0, 1); % Non-zero as zero arm length case was handled specially above
	alpha = acos(calpha); % Alpha is in the range [0,pi/2]
	salpha = sin(alpha);

	% Calculate the required elbow pitch
	elbowPitch = -2*alpha;

	% Calculate the required shoulder roll
	yhat = handInShoulderFrame(2) / armLength; % The arm length zero case was handled specially above
	sphi = coerce(yhat / calpha, -1.0, 1.0); % Note that calpha is non-zero. The coercion effectively rotates invalid inverse poses to the nearest pose on the conical border of feasibility (it is equivalent to reducing yhat to calpha while keeping the xhat-to-zhat ratio the same)
	shoulderRoll = asin(sphi); % We take the solution to shoulder roll in the range [-pi/2,pi/2], instead of pi - shoulderRoll, to ensure the elbow is pointing in the correct logical direction

	% Calculate the required shoulder pitch
	cphi = cos(shoulderRoll);
	if abs(salpha) < Tol && abs(cphi) < Tol % Note that calpha ~ 1 if salpha ~ 0
		shoulderPitch = 0;
	elseif all(abs(handInShoulderFrame([1 3])) < Tol) % If this is true then cphi is approximately zero, but alpha can be anything
		shoulderPitch = 0;
	else
		shoulderPitch = picutMax(atan2(handInShoulderFrame(1), handInShoulderFrame(3)) - atan2(salpha, -cphi*calpha), 3*pi/4);
	end

	% Construct the output joint angles
	AJP = ArmJointPose([shoulderRoll shoulderPitch], elbowPitch);

end
% EOF