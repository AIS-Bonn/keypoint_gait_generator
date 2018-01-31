% Test leg kinematics functions
%#ok<*SAGROW>

% Number of test cases
N = 100000;
fprintf('N = %d\n', N);

% General
limbSign = RandLimbSign();
RM = RobotModel;
L = RM.legLinkLength;
Loffset = [0 0 2*L];
Hoffset = [-RM.hipOffsetX -limbSign*RM.hipOffsetY 0];
P = Hoffset + Loffset;

% Test InvFromJoint
disp('TEST: InvFromJoint');
for k = 1:N
	JP = RandJointPose(false);
	[IP, hipPRPos, kneePos] = InvFromJoint(JP, limbSign, RM);
	err = [];
	err(end+1) = P(3) - hipPRPos(3);
	err(end+1) = norm(P - hipPRPos) - norm(Hoffset);
	err(end+1) = dot(P - hipPRPos,Hoffset) - norm(P - hipPRPos)*norm(Hoffset)*cos(JP.hipYaw);
	err(end+1) = norm(kneePos - hipPRPos) - L;
	err(end+1) = norm(IP.anklePos - kneePos) - L;
	err(end+1) = dot(kneePos - hipPRPos, IP.anklePos - kneePos) - L*L*cos(JP.kneePitch);
	if any(abs(err) > 1e-13)
		warning('There is a problem!');
		disp(JP);
		disp(IP);
		disp(err);
		pause
	end
end

% Test InvFromAbs
disp('TEST: InvFromAbs');
for k = 1:N
	AP = RandAbsPose(false);
	[IP, hipPRPos, kneePos] = InvFromAbs(AP, limbSign, RM);
	err = [];
	err(end+1) = P(3) - hipPRPos(3);
	err(end+1) = norm(P - hipPRPos) - norm(Hoffset);
	err(end+1) = dot(P - hipPRPos,Hoffset) - norm(P - hipPRPos)*norm(Hoffset)*cos(AP.angleZ);
	err(end+1) = norm(kneePos - hipPRPos) - L;
	err(end+1) = norm(IP.anklePos - kneePos) - L;
	err(end+1) = norm(hipPRPos - IP.anklePos) - 2*L*(1-AP.retraction);
	if any(abs(err) > 1e-13)
		warning('There is a problem!');
		disp(AP);
		disp(IP);
		disp(err);
		pause
	end
end
% EOF