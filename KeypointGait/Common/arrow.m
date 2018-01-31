% arrow.m - Philipp Allgeuer - 06/03/15
% Draw an arrow in a 2D plot.
%
% function [ax,ay] = arrow(origin,vec,b,theta,varargin)
%
% INPUTS:
% origin:   The coordinates (x,y) of the base of the vector
% vec:      The vector to draw
% b:        The absolute length of each of the hat (arrow tip) lines
% theta:    The angle between the hat lines and the body of the vector (degrees)
% varargin: Property-value pairs to pass to the plot3 via the set function (i.e. set(h,varargin{:}))
%
% OUTPUTS:
% ax,ay: The x and y plot coordinates for the arrow, in case you want to plot it yourself
%
% Note: The plot command is only executed for you if there are no output arguments to the function call.
%       Plot the resulting arrow yourself using plot(ax,ay).
%       At least two input variables are required, but the rest have default values.

% Main function
function [ax,ay] = arrow(origin,vec,b,theta,varargin)

	% Supply parameters if the user has not supplied them
	if isscalar(origin) && origin == 0
		origin = [0 0];
	end
	if nargin < 2
		error('Insufficient arguments: This function requires at least origin and vec!');
	end
	if nargin < 3 || b <= 0
		b = 0.15*norm(vec);
	end
	thetaoff = 0;
	if nargin >= 4 && numel(theta) == 2
		thetaoff = theta(2);
		theta = theta(1);
	end
	if nargin < 4 || theta <= 0
		theta = 25;
	end

	% Make sure every vector is a row vector
	origin = origin(:)';
	vec = vec(:)';

	% Calculate our basis vectors
	if norm(vec) > 1024*eps
		vhat = vec/norm(vec);
	else
		vhat = [1 0];
	end
	uhat = [-vhat(2) vhat(1)];

	% Calculate the vector plot coordinates
	hat = b*[0 0;sind(thetaoff+theta) -cosd(thetaoff+theta);0 0;sind(thetaoff-theta) -cosd(thetaoff-theta)]*[uhat;vhat];
	arrow = [origin;repmat(origin+vec,4,1)+hat];

	% Make data aliases
	ax = arrow(:,1);
	ay = arrow(:,2);

	% Plot the arrows in the current figure if the output arguments are not being retrieved
	if nargout < 1
		h = plot(ax,ay);
		if ~isempty(varargin)
			set(h,varargin{:});
		end
	end

end
% EOF