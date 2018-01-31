% Convert a limb sign index into a limb sign.
%
% function [limbSign] = LimbSign(I)
%
% Left:  limbSign =  1, I = 1 (limbSign >= 0, I <= 1)
% Right: limbSign = -1, I = 2 (limbSign < 0, I >= 2)
%
function [limbSign] = LimbSign(I)

	% Return the required limb sign
	if I <= 1.5
		limbSign = 1;
	else
		limbSign = -1;
	end

end
% EOF