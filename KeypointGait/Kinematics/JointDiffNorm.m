% JointDiffNorm.m - Philipp Allgeuer - 22/03/17
% Calculate the normed difference between two joint poses.
%
% function [diff] = JointDiffNorm(JPA, JPB)
%
% The output normed difference is not affected by shifts by 2*pi.
%
function [diff] = JointDiffNorm(JPA, JPB)

	% Calculate the difference in joint angles
	diff = norm(picut([JPA.hipYaw JPA.hipRoll JPA.hipPitch JPA.kneePitch JPA.anklePitch JPA.ankleRoll] - [JPB.hipYaw JPB.hipRoll JPB.hipPitch JPB.kneePitch JPB.anklePitch JPB.ankleRoll]));

end
% EOF