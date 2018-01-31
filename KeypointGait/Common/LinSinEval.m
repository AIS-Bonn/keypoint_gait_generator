% LinSinEval.m - Philipp Allgeuer - 01/09/16
% Fillet a linear-sinusoid transition (hacked to be unsuccessful if a maxPre is exceeded)

% Main function
function [Delta, Success, Coeff] = LinSinEval(t, A, B, T, maxPre)

	% Calculate stuff
	[Delta, Success, Coeff] = LinSinFillet(A, B, T, t);

	% Make unsuccessful if tPre exceeds maxPre
	tPre = Coeff(6); % Careful: This is negative, but maxPre is positive!
	if tPre < -maxPre
		error('LinSinFillet: maxPre was violated!');
	end

end
% EOF