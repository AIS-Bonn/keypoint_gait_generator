% Construct an inverse pose representation
%
% function [IP] = InversePose(anklePos, footRot)
%
function [IP] = InversePose(anklePos, footRot)

	% Return zero pose if no input arguments are given
	if nargin == 0
		anklePos = [0 0 0];
		footRot = QuatIdentity;
	end

	% Error checking
	anklePos = anklePos(:)';
	if numel(anklePos) ~= 3
		error('InversePose: Invalid ankle position!');
	end
	footRot = footRot(:)';
	if numel(footRot) ~= 4 || abs(norm(footRot) - 1) > 128*eps
		error('InversePose: Invalid foot rotation!');
	end

	% Construct the required representation
	IP = struct();
	IP.anklePos = anklePos;
	IP.footRot = footRot;

end
% EOF