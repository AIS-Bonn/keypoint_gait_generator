% ASFromSA.m - Philipp Allgeuer - 01/09/16
% Convert a structure of arrays to an array of structures
function [AS] = ASFromSA(SA)

	% Convert structure of arrays (SA) to array of structures (AS)
	F = fieldnames(SA);
	for k = 1:length(F)
		cellData = num2cell(SA.(F{k}));
		[AS(1:length(cellData)).(F{k})] = cellData{:};
	end

end
% EOF