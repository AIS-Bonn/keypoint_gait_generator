% Plot a 3D visualisation of a particular pose representation (left leg)
%
% function [fig, IP] = PlotPose(P, fig, limbSign, quick, hipCentric, RM)
%
% P can be a JP, AP or IP pose representation.
%
function [fig, IP] = PlotPose(P, fig, limbSign, quick, hipCentric, RM)

	% Input arguments
	if nargin < 2 || fig == 0
		fig = figure();
	elseif fig > 0
		figure(fig);
	else
		fig = gcf;
	end
	if nargin < 3
		limbSign = +1;
	end
	if nargin < 4
		quick = false;
	end
	if nargin < 5
		hipCentric = false;
	end
	if nargin < 6
		RM = RobotModel;
	end

	% Robot dimensions
	L = 2.0*RM.legLinkLength;
	H = 0.5*RM.hipWidth*limbSign;
	hx = RM.hipOffsetX;
	hy = limbSign*RM.hipOffsetY;
	Hoffset = [hx hy 0];

	% Decide on coordinate system offsets
	if hipCentric
		hipPos = [0 0 0];
	else
		hipPos = [0 H L];
	end
	hipCentrePos = hipPos - [0 H 0];
	ankleZeroPos = hipPos + Hoffset - [0 0 L];

	% Set up the figure
	if ~quick
		buffer = L + abs(RM.footOffsetX) + abs(RM.footOffsetZ);
		Xmin = hipPos(1) - buffer;
		Xmax = hipPos(1) + buffer;
		Ymin = hipPos(2) - buffer;
		Ymax = hipPos(2) + buffer;
		Zmin = hipPos(3) - buffer;
		Zmax = hipPos(3) + buffer;
	end

	% Save the hold state
	washeld = ishold;
	hold on;
	cla;

	% Pose calculations
	if isfield(P, 'retraction')
		[IP, hipPRPos, kneePos] = InvFromAbs(P, limbSign, RM);
	elseif isfield(P, 'anklePos')
		IP = P;
		hipPRPos = hipPos - ankleZeroPos;
		kneePos = IP.anklePos;
	else
		[IP, hipPRPos, kneePos] = InvFromJoint(P, limbSign, RM);
	end
	hipPRPos = ankleZeroPos + hipPRPos;
	kneePos = ankleZeroPos + kneePos;
	anklePos = ankleZeroPos + IP.anklePos;

	% Calculate the intermediate hip position
	uvec = hipPRPos - hipPos;
	beta = atan2(hy, hx);
	cbeta = cos(beta);
	sbeta = sin(beta);
	hipPosY = hipPos + cbeta*[cbeta*uvec(1)+sbeta*uvec(2) cbeta*uvec(2)-sbeta*uvec(1) 0];

	% Calculate the floor positions
	footRotmat = RotmatFromQuat(IP.footRot);
	footX = footRotmat(:,1)';
	footY = footRotmat(:,2)';
	footZ = footRotmat(:,3)';
	floorPosZ = anklePos - RM.footOffsetZ*footZ;
	floorPosZY = floorPosZ + limbSign*RM.footOffsetY*footY;
	floorPosZYX = floorPosZY + RM.footOffsetX*footX;

	% Calculate foot coordinates
	footCoords = repmat(floorPosZYX, 5, 1) + 0.5*RM.footLength*[1 1 -1 -1 1]'*footX + 0.5*RM.footWidth*[1 -1 -1 1 1]'*footY;

	% Plot the pose
	PlotCsys(eye(3), '', hipCentrePos, RM.hipWidth);
	plot3([hipCentrePos(1) hipPos(1)], [hipCentrePos(2) hipPos(2)], [hipCentrePos(3) hipPos(3)], 'm.-', 'LineWidth', 1.5, 'MarkerSize', 12);
	plot3([hipPos(1) hipPosY(1)], [hipPos(2) hipPosY(2)], [hipPos(3) hipPosY(3)], 'b-', 'LineWidth', 1.5);
	plot3([hipPosY(1) hipPRPos(1)], [hipPosY(2) hipPRPos(2)], [hipPosY(3) hipPRPos(3)], '-', 'Color', [0.15 0.70 0.15], 'LineWidth', 1.5);
	plot3([hipPRPos(1) kneePos(1) anklePos(1)], [hipPRPos(2) kneePos(2) anklePos(2)], [hipPRPos(3) kneePos(3) anklePos(3)], 'm.-', 'LineWidth', 1.5, 'MarkerSize', 12);
	plot3([anklePos(1) floorPosZ(1)], [anklePos(2) floorPosZ(2)], [anklePos(3) floorPosZ(3)], 'r-', 'LineWidth', 1.5);
	plot3([floorPosZ(1) floorPosZY(1)], [floorPosZ(2) floorPosZY(2)], [floorPosZ(3) floorPosZY(3)], '-', 'Color', [0.15 0.70 0.15], 'LineWidth', 1.5);
	plot3([floorPosZY(1) floorPosZYX(1)], [floorPosZY(2) floorPosZYX(2)], [floorPosZY(3) floorPosZYX(3)], 'b-', 'LineWidth', 1.5);
	plot3(floorPosZYX(1), floorPosZYX(2), floorPosZYX(3), 'b.', 'LineWidth', 1.5, 'MarkerSize', 12);
	plot3([hipPRPos(1) anklePos(1)], [hipPRPos(2) anklePos(2)], [hipPRPos(3) anklePos(3)], 'k:', 'LineWidth', 1.5);
	plot3(footCoords(:,1), footCoords(:,2), footCoords(:,3), 'k-', 'LineWidth', 1.5); 

	% Restore the hold state
	if ~washeld
		hold off
	end

	% Finish up the figure
	if ~quick
		axis equal;
		xlim([Xmin Xmax]);
		ylim([Ymin Ymax]);
		zlim([Zmin Zmax]);
		xlabel('x \rightarrow');
		ylabel('y \rightarrow');
		zlabel('z \rightarrow');
		view(-210, 30);
		grid on;
	end

	% Return figure if required
	if nargout < 1
		clear fig;
	end

end
% EOF