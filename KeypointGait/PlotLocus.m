% Plot the 3D locus of a given joint pose array (plots the locus of the foot floor point).
%
% function [fig, offset] = PlotLocus(JPA, u, fig, useOffset, limbSign, RM, config)
%
% JPA is an array of JP, AP or IP structs, or an Nx3 IPF matrix (then limbSign unimportant).
function [fig, offset] = PlotLocus(JPA, u, fig, useOffset, limbSign, RM, config)

	% Input parameters
	N = length(JPA);
	if nargin < 1 || N < 1
		error('Please supply valid JPA data to plot the locus of!');
	end
	if nargin < 2
		u = 0;
	end
	haveu = ~isscalar(u);
	if nargin < 3 || fig == 0
		fig = figure();
	elseif fig > 0
		figure(fig);
	else
		fig = gcf;
	end
	if nargin < 4
		useOffset = true;
	end
	offset = [0 0 0];
	if isscalar(useOffset)
		if useOffset
			autoOffset = true;
			useDecor = true;
		else
			autoOffset = false;
			useDecor = false;
		end
	else
		autoOffset = false;
		useDecor = false;
		offset = useOffset;
	end
	if nargin < 5
		limbSign = +1;
	end
	if nargin < 6
		RM = RobotModel;
	end
	if nargin < 7
		config = ConfigVars;
	end

	% Get the type of the input array
	if isfield(JPA, 'retraction')
		atype = 1; % Abstract pose array
	elseif isfield(JPA, 'anklePos')
		atype = 2; % Inverse pose array
	elseif isstruct(JPA)
		atype = 0; % Joint pose array
	else
		atype = 3; % Inverse foot position array
		sz = size(JPA);
		N = sz(1);
		if sz(2) ~= 3
			error('If not a struct, then JPA must be position data in Nx3 matrix format!');
		end
	end

	% Calculate and plot the locus as required
	if atype == 3
		Data = JPA;
		if autoOffset
			offset = FootFloorPoint(InvFromAbs(AbstractHaltPose(1, config), 1, RM), 1, RM) - FootFloorPoint(InvFromAbs(AbstractHaltPose(-1, config), -1, RM), -1, RM);
		end
		Data = Data - repmat(offset, N, 1);
		label = 'LR';
	else
		if autoOffset
			offset = FootFloorPoint(InvFromAbs(AbstractHaltPose(limbSign, config), limbSign, RM), limbSign, RM);
		end
		Data = nan(N,3);
		for k = 1:N
			if atype == 1
				IP = InvFromAbs(JPA(k), limbSign, RM);
			elseif atype == 2
				IP = JPA(k);
			else
				IP = InvFromJoint(JPA(k), limbSign, RM);
			end
			Data(k,:) = FootFloorPoint(IP, limbSign, RM) - offset;
		end
		if limbSign >= 0
			label = 'L';
		else
			label = 'R';
		end
	end

	% Calculate the indices of certain phase keypoints
	keyphase = [0 config.doubleSupportPhaseLen+[0 config.swingStartPhaseOffset] pi-config.swingStopPhaseOffset];
	keyphase = [keyphase mean(keyphase(1:2))];
	keyphase = [keyphase pi+keyphase];
	M = numel(keyphase);
	keyvalue = nan(M, 3);
	for k = 1:(N-1)
		for m = 1:M
			if u(k) <= keyphase(m) && keyphase(m) <= u(k+1) && any(isnan(keyvalue(m,:)))
				v = interp1([u(k) u(k+1)], [1 0], keyphase(m));
				keyvalue(m,1) = v*Data(k,1) + (1-v)*Data(k+1,1);
				keyvalue(m,2) = v*Data(k,2) + (1-v)*Data(k+1,2);
				keyvalue(m,3) = v*Data(k,3) + (1-v)*Data(k+1,3);
			end
		end
	end

	% Plot the locus
	if useDecor
		PlotCsys(eye(3), label, [0 0 0], 0.08);
	end
	washeld = ishold;
	hold on;
	if haveu
		patch('Vertices', Data, 'Faces', 1:N, 'FaceVertexCData', jet(N), 'FaceVertexAlphaData', [ones(N-1,1);0], 'EdgeColor','flat','EdgeAlpha','flat','FaceColor','none','LineWidth',1.5);
	else
		plot3(Data(:,1), Data(:,2), Data(:,3), '-','Color', [0.5 0 0.5], 'LineWidth', 1.5);
	end
	if useDecor
		ind = [1 2 6 7]; plot3(keyvalue(ind,1), keyvalue(ind,2), keyvalue(ind,3), 'mx', 'MarkerSize', 12);
		ind = [3 4 8 9]; plot3(keyvalue(ind,1), keyvalue(ind,2), keyvalue(ind,3), 'bx', 'MarkerSize', 12);
		ind = [5 10]; plot3(keyvalue(ind,1), keyvalue(ind,2), keyvalue(ind,3), 'k.-', 'LineWidth', 1.5, 'MarkerSize', 12);
	end
	if ~washeld
		hold off;
	end
	axis equal;
	xlim([-0.1 0.1]);
	ylim([-0.1 0.1]);
	zlim([-0.1 0.1]);
	xlabel('x \rightarrow');
	ylabel('y \rightarrow');
	zlabel('z \rightarrow');
	view(-210, 30);
	grid on;

	% Manage outputs
	if nargout < 1
		clear fig;
	end
	if nargout < 2
		clear offset
	end

end
% EOF