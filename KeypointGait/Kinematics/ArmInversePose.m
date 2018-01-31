% Construct an inverse arm pose representation
%
% function [AIP] = ArmInversePose(handPos)
%
% Note that due to the limited degrees of freedom of the arm, specifying just the position
% of the hand almost uniquely specifies the arm pose, considering that the elbow only bends
% one way and the elbow should in general be facing 'backwards'. There are many unachievable
% inverse poses however, and multiple ambiguous inverse poses, so arm poses can be converted
% to the inverse representation, but in enough scenarios converting back will not result in
% the same original pose.
%
function [AIP] = ArmInversePose(handPos)

	% Return zero pose if no input arguments are given
	if nargin == 0
		handPos = [0 0 0];
	end

	% Error checking
	handPos = handPos(:)';
	if numel(handPos) ~= 3
		error('ArmInversePose: Invalid hand position!');
	end

	% Construct the required representation
	AIP = struct();
	AIP.handPos = handPos;

end
% EOF