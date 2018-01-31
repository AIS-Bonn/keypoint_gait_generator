% Given a limb sign index, return the limb sign index of the other limb.
%
% function [Iother] = OtherLimbSign(I)
%
% Left:  limbSign =  1, I = 1 (limbSign >= 0, I <= 1)
% Right: limbSign = -1, I = 2 (limbSign < 0, I >= 2)
%
function [Iother] = OtherLimbSign(I)

	% Return the required limb sign index
	if I <= 1.5
		Iother = 2;
	else
		Iother = 1;
	end

end
% EOF