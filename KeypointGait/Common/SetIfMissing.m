% Set a field of a structure to a default value if it does not exist
%
% function [S] = SetIfMissing(S, field, default)
%
function [S] = SetIfMissing(S, field, default)
	if ~isfield(S, field)
		S.(field) = default;
	end
end
% EOF