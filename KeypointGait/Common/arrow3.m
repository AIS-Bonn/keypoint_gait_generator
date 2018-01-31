% arrow3.m - Philipp Allgeuer - 21/11/13
% Draw an arrow in a 3D plot
%
% function [ax,ay,az] = arrow3(origin,vec,b,theta,normal,linestyle,varargin)
%
% INPUTS:
% origin:    The coordinates of the base of the vector
% vec:       The vector to draw
% b:         The length of each of the hat lines
% theta:     The angle between the hat lines and the body of the vector
% normal:    The desired 'viewing direction' of the arrow - the arrow is drawn in a plane perpendicular to this vector
% linestyle: The line style for the stem of the arrow
% varargin:  Property-value pairs to pass to the plot3 via the set function (i.e. set(h,varargin{:}))
%
% OUTPUTS:
% ax,ay,az: The x, y and z plot coordinates for the arrow, in case you want to plot it yourself
%
% Note: The plot3 command is only executed for you if there are no output arguments to the function call.
%       Plot the resulting arrow yourself using plot3(ax,ay,az).
%       At least two input variables are required, but the rest have default values.

% Main function
function [ax,ay,az] = arrow3(origin,vec,b,theta,normal,linestyle,varargin)

	% Supply parameters if the user has not supplied them
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
	if nargin < 5 || isscalar(normal)
		normal = [1 0 1];
	end
	if nargin < 6 || (isscalar(linestyle) && linestyle == 0)
		linestyle = '-';
	end
	
	% Make sure every vector is a row vector
	if isscalar(origin) && origin == 0
		origin = [0 0 0];
	end
	origin = origin(:)';
	vec = vec(:)';
	normal = normal(:)';
	
	% Calculate our basis vectors
	if norm(vec) > 1024*eps
		vhat = vec/norm(vec);
	else
		vhat = [0 0 1];
	end
	if norm(normal) > 1024*eps
		nhat = normal/norm(normal);
	else
		nhat = [1 0 1]/sqrt(2);
		if cross(vhat,nhat) == 0
			nhat = [0 1 1]/sqrt(2);
		end
	end
	uhat = cross(vhat,nhat);
    uhat = uhat/norm(uhat);
	
	% Calculate the vector plot coordinates
	hat = b*[0 0;sind(thetaoff+theta) -cosd(thetaoff+theta);0 0;sind(thetaoff-theta) -cosd(thetaoff-theta)]*[uhat;vhat];
	arrow = [origin;repmat(origin+vec,4,1)+hat];
	
	% Make data aliases
	ax = arrow(:,1);
	ay = arrow(:,2);
	az = arrow(:,3);

	% Plot the arrows in the current figure if the output arguments are not being retrieved
	if nargout < 1
		if ~strcmp(linestyle, 'none')
			h = plot3(ax(1:2),ay(1:2),az(1:2));
			if ~isempty(varargin)
				set(h,varargin{:});
			end
			set(h,'LineStyle',linestyle);
		end
		h = plot3(ax(3:end),ay(3:end),az(3:end));
		if ~isempty(varargin)
			set(h,varargin{:});
		end
	end

end
% EOF