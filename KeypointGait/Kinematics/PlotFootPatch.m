% PlotFootPatch.m - Philipp Allgeuer - 17/10/16
% Plot a foot patch in the currently active 3D plot.
%
% function [h] = PlotFootPatch(Rot, FFP, colour, showArrows, RM, varargin)
%
% Rot ==> Orientation of the foot (rotation matrix or quaternion)
% FFP ==> Foot floor point (centre of the bottom of the foot)
% colour ==> Fill colour of the patch
% showArrows ==> Boolean whether to show the arrow annotations
% RM ==> Robot model
% varargin ==> Patch properties to pass on to the patch() function
% h ==> Output patch plot handle

% Main function
function [h] = PlotFootPatch(Rot, FFP, colour, showArrows, RM, varargin)

	% Input arguments
	if nargin < 1
		Rot = eye(3);
	end
	if size(Rot,1) == 1 && size(Rot,2) == 4
		Rot = RotmatFromQuat(Rot);
	end
	if any(size(Rot) ~= [3 3])
		error('Invalid Rot input!');
	end
	if norm(Rot*Rot'-eye(3),'fro') > 1024*eps
		warning('Rot is not orthogonal to 1024*eps!');
	end
	if nargin < 2 || (isscalar(FFP) && FFP == 0)
		FFP = [0; 0; 0];
	end
	FFP = FFP(:);
	if nargin < 3 || (isscalar(colour) && colour == 0)
		colour = [0.73 0.33 0.83];
	end
	if nargin < 4
		showArrows = false;
	end
	if nargin < 5 || (~isstruct(RM) && RM == 0)
		RM = RobotModel;
	end

	% Hold the plot
	washeld = ishold;
	hold on;

	% Plot the foot patch
	data = repmat(FFP,1,5) + 0.5*RM.footLength*Rot(:,1)*[1 1 -1 -1 1] + 0.5*RM.footWidth*Rot(:,2)*[1 -1 -1 1 1];
	h = patch(data(1,:), data(2,:), data(3,:), colour, 'LineWidth', 1.0, varargin{:});

	% Plot the foot patch toe wedge
	wedgeData = [data(:,1) 0.5*(data(:,1) + data(:,2) - RM.footWidth*Rot(:,1)) data(:,2)];
	plot3(wedgeData(1,:), wedgeData(2,:), wedgeData(3,:), 'Color', h.EdgeColor, 'LineWidth', h.LineWidth, 'LineStyle', h.LineStyle);

	% Plot arrows if required
	if showArrows
		arrow3(FFP, 0.5*(RM.footLength - RM.footWidth)*Rot(:,1), 0, 0, Rot(:,3), '-', 'Color', [0.15 0.15 0.70]);
		arrow3(FFP, 0.5*(RM.footLength - RM.footWidth)*Rot(:,2), 0, 0, Rot(:,3), '-', 'Color', [0.15 0.70 0.15]);
		arrow3(FFP, 0.5*(RM.footLength - RM.footWidth)*Rot(:,3), 0, 0, Rot(:,1), '-', 'Color', [0.70 0.15 0.15]);
	end

	% Restore the initial hold state
	if ~washeld
		hold off;
	end

	% Don't return handle if not asked
	if nargout < 1
		clear h
	end

end
% EOF