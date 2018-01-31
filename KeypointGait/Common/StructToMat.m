% StructToMat.m - Philipp Allgeuer - 03/03/17
% Convert a structure to a matrix by concatenation in the order of the field names.
%
% function [M] = StructToMat(S, dim)
%
% Concatenation is along dimension dim. If dim is not provided, concatenation is vertical
% unless the first encountered field contains a non-scalar column vector. If dim is
% non-positive (e.g. zero) then concatenation is vertical and all fields are turned into
% column vectors using (:).
%
function [M] = StructToMat(S, dim)

	% Concatenate the struct fields as required
	first = true;
	fnames = fieldnames(S);
	for k = 1:numel(fnames)
		field = S.(fnames{k});
		if first
			if nargin < 2
				if iscolumn(field) && ~isscalar(field)
					dim = 2;
				else
					dim = 1;
				end
			end
			M = [];
			first = false;
		end
		if dim > 0
			M = cat(dim, M, field);
		else
			M = cat(1, M, field(:));
		end
	end

end
% EOF