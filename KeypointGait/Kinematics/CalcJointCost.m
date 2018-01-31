% CalcJointCost.m - Philipp Allgeuer - 22/03/17
% Calculate the cost of a joint pose (if there are multiple options, lower cost means a better solution).
% 
% function [maxCost, sumCost] = CalcJointCost(JP)
%
function [maxCost, sumCost] = CalcJointCost(JP)

	% Calculate the required costs
	absValues = [abs(JP.hipYaw) abs(JP.hipRoll) abs(JP.hipPitch)/1.5];
	maxCost = max(absValues);
	sumCost = sum(absValues);

end
% EOF