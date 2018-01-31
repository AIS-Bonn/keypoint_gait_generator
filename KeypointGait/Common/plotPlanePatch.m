% plotPlanePatch.m - Philipp Allgeuer - 10/11/16
% Plot an arbitrary plane patch based on a point and two spanning vectors, whose magnitude
% dictate the size of the plane in each direction from the point.
%
% function plotPlanePatch(point, u, v, varargin)
%
% varargin: The first optional argument should be a ColorSpec. After that, name/value property
%           pairs can be specified to adjust e.g. LineWidth and FaceAlpha.
%
function plotPlanePatch(point, u, v, varargin)

	% Plot the required plan patch
	plane = repmat(point(:),1,5) + u(:)*[1 1 -1 -1 1] + v(:)*[1 -1 -1 1 1];
	patch(plane(1,:), plane(2,:), plane(3,:), varargin{:});

end
% EOF