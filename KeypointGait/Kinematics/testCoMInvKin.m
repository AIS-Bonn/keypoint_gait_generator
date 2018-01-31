
RM = RobotModel;
alphaNom = acos(1-0.09);

PlotArmPose(ArmJointPose,13,0,true);

CoMLineOrig = [0 0 -1];
for gamma = -1.5:0.1:1.5

	alpha = [];
	pitch = [];
	roll = [];

	k = 1;
	H = 0.01;
	thrange = (0:H:1.8)';

	for theta = thrange'
		T = [0 gamma theta];
		CoMLine = TiltRotVec(T,CoMLineOrig);

% 		phiLim = [1.5 0.2];
% 		tiltLim = [1.4 1.2 0.15];

		alphaVal = 0*alphaNom/3;
		[AJP, CoM] = ArmJointFromCoM(CoMLine, alphaVal);%, phiLim, tiltLim);
		alpha(k,1) = -0.5*AJP.elbowPitch;
		pitch(k,1) = AJP.shoulderPitch;
		roll(k,1) = AJP.shoulderRoll;

% 		[fig, AIP, elbowPos, comPos] = PlotArmPose(AJP,10,3,true);
% 		err = norm([norm(comPos - CoM*RM.armLinkLength) norm(cross(CoMLine,CoM)) norm(cross(CoMLine,comPos))]);
% 		if err > 1e-10
% 			error('Error is large!');
% 		end

		alphaVal = 1*alphaNom/3;
		[AJP, CoM] = ArmJointFromCoM(CoMLine, alphaVal);%, phiLim, tiltLim);
		alpha(k,2) = -0.5*AJP.elbowPitch;
		pitch(k,2) = AJP.shoulderPitch;
		roll(k,2) = AJP.shoulderRoll;

% 		[fig, AIP, elbowPos, comPos] = PlotArmPose(AJP,11,3,true);
% 		err = norm([norm(comPos - CoM*RM.armLinkLength) norm(cross(CoMLine,CoM)) norm(cross(CoMLine,comPos))]);
% 		if err > 1e-10
% 			error('Error is large!');
% 		end

		alphaVal = 2*alphaNom/3;
		[AJP, CoM] = ArmJointFromCoM(CoMLine, alphaVal);%, phiLim, tiltLim);
		alpha(k,3) = -0.5*AJP.elbowPitch;
		pitch(k,3) = AJP.shoulderPitch;
		roll(k,3) = AJP.shoulderRoll;

% 		[fig, AIP, elbowPos, comPos] = PlotArmPose(AJP,12,3,true);
% 		err = norm([norm(comPos - CoM*RM.armLinkLength) norm(cross(CoMLine,CoM)) norm(cross(CoMLine,comPos))]);
% 		if err > 1e-10
% 			error('Error is large!');
% 		end

		alphaVal = 3*alphaNom/3;
		[AJP, CoM] = ArmJointFromCoM(CoMLine, alphaVal);%, phiLim, tiltLim);
		alpha(k,4) = -0.5*AJP.elbowPitch;
		pitch(k,4) = AJP.shoulderPitch;
		roll(k,4) = AJP.shoulderRoll;

		comp(k,1) = atan2(CoMLine(1),CoMLine(3));
		comp(k,2) = AJP.shoulderPitch - comp(k,1);
		comp(k,3) = -atan2(sin(2*alpha(k,4)),-cos(AJP.shoulderRoll)*(3+cos(2*alpha(k,4))));

		[fig, AIP, elbowPos, comPos] = PlotArmPose(AJP,13,3,true);
		hold on
		otherPos = comPos + (comPos - elbowPos);
		plot3([elbowPos(1); otherPos(1)], [elbowPos(2); otherPos(2)], [elbowPos(3); otherPos(3)], 'b-');
		hold off
		view([180 0]);
% 		err = norm([norm(comPos - CoM*RM.armLinkLength) norm(cross(CoMLine,CoM)) norm(cross(CoMLine,comPos))]);
% 		if err > 1e-10
% 			error('Error is large!');
% 		end

		pause(0.01);
		k = k + 1;
	end

	figure(77)
	ax = gca;
	ax.ColorOrderIndex = 1; plot(thrange, alpha);
	hold on
	ax.ColorOrderIndex = 1; plot(thrange, pitch);
	ax.ColorOrderIndex = 1; plot(thrange, roll);
	hold off;
	title(['Position \gamma = ' num2str(gamma)]);
	legend('\alpha = 0','\alpha = 1','\alpha = 2','\alpha = 3');
	xlim([0 max(thrange)]);
	ylim([-pi pi]);
	grid on

	dtheta = 0.5*(thrange(1:(end-1)) + thrange(2:end));
	dalpha = 0.5*(alpha(1:(end-1),:) + alpha(2:end,:));
	dpitch = diff(pitch)/H;
	droll = diff(roll)/H;

	figure(78)
	ax = gca;
	ax.ColorOrderIndex = 1; plot(dtheta, dalpha);
	hold on
	ax.ColorOrderIndex = 1; plot(dtheta, dpitch);
	ax.ColorOrderIndex = 1; plot(dtheta, droll);
	hold off;
	title(['Velocity \gamma = ' num2str(gamma)]);
	legend('\alpha = 0','\alpha = 1','\alpha = 2','\alpha = 3');
	xlim([0 max(thrange)]);
	ylim([-pi pi]);
	grid on

	figure(79);
	ax = gca;
	ax.ColorOrderIndex = 1; plot(thrange, [comp pitch(:,4)]);
	xlim([0 max(thrange)]);
	ylim([-pi pi]);
	grid on

	pause
end
