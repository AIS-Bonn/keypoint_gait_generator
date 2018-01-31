% InvDiffNormWeak.m - Philipp Allgeuer - 22/03/17
% Calculate the weak normed difference between two inverse poses.
%
% function [diff] = InvDiffNormWeak(IPA, IPB, hipPRPosB, exact)
%
% If exact is true, then this function behaves just like InvDiffNorm, otherwise it is only
% checked how closely IPA corresponds to IPB up to the leg retraction of IPB.
%
% Be aware that the condition of being equal to another inverse pose by changing just the
% retraction is NOT a symmetric condition!
%
function [diff] = InvDiffNormWeak(IPA, IPB, hipPRPosB, exact)

	% Calculate the difference in the ankle positions
	if exact
		pdiff = norm(IPA.anklePos - IPB.anklePos);
	else
		pdiff = norm(cross(IPA.anklePos - IPB.anklePos, IPB.anklePos - hipPRPosB));
	end

	% Calculate the difference in the foot rotations
	qdiffM = norm(IPA.footRot - IPB.footRot);
	qdiffP = norm(IPA.footRot + IPB.footRot);
	qdiff = min(qdiffM, qdiffP);

	% Return the total difference
	diff = pdiff + qdiff;

end
% EOF