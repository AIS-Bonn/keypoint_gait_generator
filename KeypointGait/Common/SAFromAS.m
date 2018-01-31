% SAFromAS.m - Philipp Allgeuer - 01/09/16
% Convert an array of structures to a structure of arrays
function [SA] = SAFromAS(AS)

	% Convert array of structures (AS) to structure of arrays (SA)
	F = fieldnames(AS);
	for k = 1:length(F)
		SA.(F{k}) = cell2mat({AS(:).(F{k})})';
	end

end
% EOF