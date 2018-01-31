% Plot a 3D visualisation of a particular arm pose representation
%
% function [fig, AIP, elbowPos, comPos] = PlotArmPose(AJP, fig, type, shoulderCentric, RM)
%
% JP can actually be a JP, AP or IP pose representation.
%
% type = 0 => Setup figure, clear axes, draw everything
% type = 1 => Hold, draw arm only
% type = 2 => Hold, draw everything
% type = 3 => Clear axes, draw everything
%
function [fig, AIP, elbowPos, comPos] = PlotArmPose(AJP, fig, type, shoulderCentric, RM)

	% Input arguments
	if nargin < 2 || fig == 0
		fig = figure();
	elseif fig > 0
		figure(fig);
	else
		fig = gcf;
	end
	if nargin < 3
		type = 0;
	end
	if nargin < 4
		shoulderCentric = false;
	end
	if nargin < 5
		RM = RobotModel;
	end

	% Robot dimensions
	L = 2.0*RM.armLinkLength;
	S = 0.5*RM.shoulderWidth;

	% Decide on coordinate system offsets
	if shoulderCentric
		shoulderPos = [0 0 0];
	else
		shoulderPos = [0 S L];
	end
	shoulderCentrePos = shoulderPos - [0 S 0];
	handZeroPos = shoulderPos - [0 0 L];

	% Set up the figure
	if type == 0
		buffer = L*1.05;
		Xmin = shoulderPos(1) - buffer;
		Xmax = shoulderPos(1) + buffer;
		Ymin = shoulderPos(2) - buffer;
		Ymax = shoulderPos(2) + buffer;
		Zmin = shoulderPos(3) - buffer;
		Zmax = shoulderPos(3) + buffer;
	end

	% Save the hold state
	washeld = ishold;
	hold on;
	if type == 0 || type == 3
		cla;
	end

	% Pose calculations
	if isfield(AJP, 'retraction')
		AJP = ArmJointFromAbs(AJP);
	end
	if isfield(AJP, 'handPos')
		AIP = AJP;
		elbowPos = AIP.handPos;
	else
		[AIP, elbowPos] = ArmInvFromJoint(AJP, RM);
	end
	elbowPos = handZeroPos + elbowPos;
	handPos = handZeroPos + AIP.handPos;
	comPos = 0.25*shoulderPos + 0.5*elbowPos + 0.25*handPos;

	% Plot the pose
	if type ~= 1
		PlotCsys(eye(3), '', shoulderCentrePos, RM.shoulderWidth);
		plot3([shoulderCentrePos(1) shoulderPos(1)], [shoulderCentrePos(2) shoulderPos(2)], [shoulderCentrePos(3) shoulderPos(3)], 'm-', 'LineWidth', 1.5);
	end
	plot3([shoulderPos(1) elbowPos(1) handPos(1)], [shoulderPos(2) elbowPos(2) handPos(2)], [shoulderPos(3) elbowPos(3) handPos(3)], 'mo-', 'LineWidth', 1.5);
	plot3([shoulderPos(1) handPos(1)], [shoulderPos(2) handPos(2)], [shoulderPos(3) handPos(3)], 'k:', 'LineWidth', 1.5);
	plot3(comPos(1), comPos(2), comPos(3), 'mx');

	% Restore the hold state
	if ~washeld
		hold off
	end

	% Finish up the figure
	if type == 0
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