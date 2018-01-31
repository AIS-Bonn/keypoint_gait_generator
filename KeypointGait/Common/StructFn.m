% StructFn.m - Philipp Allgeuer - 10/11/16
% Evaluate a function on a per-field basis on one or more structures.
%
% function [S] = StructFn(fn, varargin)
%
function [S] = StructFn(fn, varargin)

	% Input arguments
	if nargin < 2
		error('Require at least one struct input!');
	end

	% Get a cell array of the fields that are in common between all structures
	F = fieldnames(varargin{1});
	N = length(varargin);
	for k = 2:N
	   F = F(isfield(varargin{k}, F));
	end
	if isempty(F)
		warning('The input structs have no fields in common!');
	end

	% Evaluate the required function
	S = struct();
	for k = 1:length(F)
		field = F{k};
		val = cell(1,N);
		valIsChar = nan(N,1);
		valIsStruct = nan(N,1);
		for m = 1:N
			val{m} = varargin{m}.(field);
			valIsChar(m) = ischar(val{m});
			valIsStruct(m) = isstruct(val{m});
		end
		if any(valIsStruct)
			if all(valIsStruct)
				S.(field) = StructFn(fn, val{:});
			else
				continue;
			end
		elseif any(valIsChar)
			continue;
		else
			S.(field) = feval(fn, val{:});
		end
	end

end
% EOF