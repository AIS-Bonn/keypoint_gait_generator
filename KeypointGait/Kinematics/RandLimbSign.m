% RandLimbSign.m - Philipp Allgeuer - 31/10/16
% Generate a random limb sign (1 = Left, -1 = Right)
%
% function [limbSign] = RandLimbSign(n, m)
%
function [limbSign] = RandLimbSign(n, m)

	% Input arguments
	if nargin < 1
		n = 1;
	end
	if nargin < 2
		m = n;
	end

	% Generate the required limb sign
	limbSign = randi([0 1],n,m);
	limbSign(limbSign == 0) = -1;

end
% EOF