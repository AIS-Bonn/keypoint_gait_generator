% PicutAbs.m - Philipp Allgeuer - 22/03/17
% Picut the relevant fields of an abstract pose.
%
% function [AP] = PicutAbs(AP, coerceRet)
%
function [AP] = PicutAbs(AP, coerceRet)

	% Wrap all angles to (-pi,pi]
	AP.angleX = picut(AP.angleX);
	AP.angleY = picut(AP.angleY);
	AP.angleZ = picut(AP.angleZ);
	AP.footAngleX = picut(AP.footAngleX);
	AP.footAngleY = picut(AP.footAngleY);

	% Coerce the retraction to [0,1]
	if nargin >= 2 && coerceRet
		AP.retraction = coerce(AP.retraction, 0.0, 1.0);
	end

end
% EOF