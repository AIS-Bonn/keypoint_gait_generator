% AbsDiffNorm.m - Philipp Allgeuer - 22/03/17
% Calculate the normed difference between two abstract poses.
%
% function [diff] = AbsDiffNorm(APA, APB)
%
% The output normed difference is not affected by shifts by 2*pi.
%
function [diff] = AbsDiffNorm(APA, APB)

	% Calculate the difference in joint angles
	diff = norm([picut([APA.angleX APA.angleY APA.angleZ APA.footAngleX APA.footAngleY] - [APB.angleX APB.angleY APB.angleZ APB.footAngleX APB.footAngleY]) APA.retraction-APB.retraction]);

end
% EOF