% StructDiff.m - Philipp Allgeuer - 15/09/16
% Take the numerical difference (D = A - B) between two structures, matching by field.
%
% function [D] = StructDiff(A, B)
%
function [D] = StructDiff(A, B)

	% Get a cell array of fields that are in common between A and B
	FA = fieldnames(A);
	F = FA(isfield(B, FA));
	if isempty(F)
		warning('A and B have no fields in common!');
	end

	% Calculate the required differences
	D = struct();
	for k = 1:length(F)
		field = F{k};
		Afield = A.(field);
		Bfield = B.(field);
		if ischar(Afield) && ischar(Bfield)
			continue;
		elseif isstruct(Afield) && isstruct(Bfield)
			D.(field) = StructDiff(Afield, Bfield);
		else
			D.(field) = Afield - Bfield;
		end
	end

end
% EOF