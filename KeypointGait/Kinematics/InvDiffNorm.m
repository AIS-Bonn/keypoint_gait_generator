% InvDiffNorm.m - Philipp Allgeuer - 22/03/17
% Calculate the normed difference between two inverse poses.
%
% function [diff] = InvDiffNorm(IPA, IPB)
%
function [diff] = InvDiffNorm(IPA, IPB)

	% Calculate the difference in the ankle positions
	pdiff = norm(IPA.anklePos - IPB.anklePos);

	% Calculate the difference in the foot rotations
	qdiffM = norm(IPA.footRot - IPB.footRot);
	qdiffP = norm(IPA.footRot + IPB.footRot);
	qdiff = min(qdiffM, qdiffP);

	% Return the total difference
	diff = pdiff + qdiff;

end
% EOF