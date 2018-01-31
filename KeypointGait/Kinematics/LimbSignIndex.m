% Convert a limb sign into a limb sign index.
%
% function [I] = LimbSignIndex(limbSign)
%
% Left:  limbSign =  1, I = 1 (limbSign >= 0, I <= 1)
% Right: limbSign = -1, I = 2 (limbSign < 0, I >= 2)
%
function [I] = LimbSignIndex(limbSign)

	% Return the required limb sign index
	if limbSign >= 0
		I = 1;
	else
		I = 2;
	end

end
% EOF