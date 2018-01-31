% StructDiffNorm.m - Philipp Allgeuer - 03/03/17
% Take the norm of the numerical difference (D = A - B) between two structures, matching by field.
%
% function [N] = StructDiffNorm(A, B)
%
function [N] = StructDiffNorm(A, B)

	% Get a cell array of fields that are in common between A and B
	FA = fieldnames(A);
	F = FA(isfield(B, FA));
	if isempty(F)
		warning('A and B have no fields in common!');
	end

	% Calculate the required difference norm
	Nvec = zeros(0,1);
	for k = 1:length(F)
		field = F{k};
		Afield = A.(field);
		Bfield = B.(field);
		if ischar(Afield) && ischar(Bfield)
			continue;
		elseif isstruct(Afield) && isstruct(Bfield)
			continue;
		elseif isequal(size(Afield), size(Bfield))
			Nvec = [Nvec; norm(Afield - Bfield)]; %#ok<*AGROW>
		elseif isvector(Afield) && isvector(Bfield) && numel(Afield) == numel(Bfield)
			Nvec = [Nvec; norm(Afield(:) - Bfield(:))];
		else
			warning(['Failed to process struct field ''' field '''']);
		end
	end
	N = norm(Nvec);

end
% EOF