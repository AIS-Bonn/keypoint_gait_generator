% StructSum.m - Philipp Allgeuer - 10/11/16
% Numerically add (S = A + B) two structures, matching by field.
%
% function [S] = StructSum(A, B)
%
function [S] = StructSum(A, B)

	% Get a cell array of fields that are in common between A and B
	FA = fieldnames(A);
	F = FA(isfield(B, FA));
	if isempty(F)
		warning('A and B have no fields in common!');
	end

	% Calculate the required addition
	S = struct();
	for k = 1:length(F)
		field = F{k};
		Afield = A.(field);
		Bfield = B.(field);
		if ischar(Afield) && ischar(Bfield)
			continue;
		elseif isstruct(Afield) && isstruct(Bfield)
			S.(field) = StructSum(Afield, Bfield);
		else
			S.(field) = Afield + Bfield;
		end
	end

end
% EOF