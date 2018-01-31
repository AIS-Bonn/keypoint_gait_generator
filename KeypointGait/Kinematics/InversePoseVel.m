% Construct an inverse pose velocity representation
%
% function [IPVel] = InversePoseVel(ankleVel, footAngVel)
%
function [IPVel] = InversePoseVel(ankleVel, footAngVel)

	% Return zero pose velocity if no input arguments are given
	if nargin == 0
		ankleVel = [0 0 0];
		footAngVel = [0 0 0];
	end

	% Error checking
	ankleVel = ankleVel(:)';
	if numel(ankleVel) ~= 3
		error('InversePose: Invalid ankle velocity!');
	end
	footAngVel = footAngVel(:)';
	if numel(footAngVel) ~= 3
		error('InversePose: Invalid foot angular velocity!');
	end

	% Construct the required representation
	IPVel = struct();
	IPVel.ankleVel = ankleVel;
	IPVel.footAngVel = footAngVel;

end
% EOF