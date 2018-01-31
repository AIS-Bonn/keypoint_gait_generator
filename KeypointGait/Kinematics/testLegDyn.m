% Test leg dynamics functions
%#ok<*SAGROW>

% Number of test cases
N = 50000;
fprintf('N = %d\n\n', N);

% General
RM = RobotModel;
dt = 3e-8;
Tol = 1e-12;

% Test JointFromAbsVel
disp('TEST: JointFromAbsVel');
tolScale = Tol / (dt*5000);
maxderr = 0;
for k = 1:N
	AP = RandAbsPose(false);
	AP.retraction = max(AP.retraction, 0.005);
	JP = JointFromAbs(AP);
	APVel = RandAbsPose(false);
	[JPVelA, JJAA] = JointFromAbsVel(APVel, AP);
	[JPVelJ, JJAJ] = JointFromAbsVel(APVel, JP);
	dAP = StructFn(@(X,V) X + dt*V, AP, APVel);
	dJP = JointFromAbs(dAP);
	dJPVel = StructFn(@(X,dX) picut(dX - X)/dt, JP, dJP);
	tmpA = JointDiffNorm(dJPVel, JPVelA);
	tmpB = JointDiffNorm(dJPVel, JPVelJ);
	maxderr = max([maxderr tmpA tmpB]);

	err = [];
	err(end+1) = JointDiffNorm(JPVelA, JPVelJ);
	err(end+1) = norm(StructToMat(JPVelA) - JJAA*StructToMat(APVel));
	err(end+1) = norm(StructToMat(JPVelJ) - JJAJ*StructToMat(APVel));
	err(end+1) = norm(JJAA - JJAJ);
	err(end+1) = tolScale*tmpA;
	err(end+1) = tolScale*tmpB;
	if any(abs(err) > Tol)
		warning('There is a problem!');
		disp(err);
		pause;
	end
end
disp(['Maximum dJPVel error multiplier: ' num2str(maxderr/dt)]);
fprintf('\n');

% Test AbsFromJointVel
disp('TEST: AbsFromJointVel');
tolScale = Tol / (dt*500);
maxderr = 0;
for k = 1:N
	JP = RandJointPose(false);
	JP.kneePitch = max(pi*rand, 0.05) + 2*pi*randi([-3 2]);
	AP = AbsFromJoint(JP);
	JPVel = RandJointPose(false);
	[APVelJ, JAJJ] = AbsFromJointVel(JPVel, JP);
	[APVelA, JAJA] = AbsFromJointVel(JPVel, AP);
	dJP = StructFn(@(X,V) X + dt*V, JP, JPVel);
	dAP = AbsFromJoint(dJP);
	dAPVel = StructFn(@(X,dX) picut(dX - X)/dt, AP, dAP);
	tmpA = AbsDiffNorm(dAPVel, APVelA);
	tmpB = AbsDiffNorm(dAPVel, APVelJ);
	maxderr = max([maxderr tmpA tmpB]);

	err = [];
	err(end+1) = AbsDiffNorm(APVelA, APVelJ);
	err(end+1) = norm(StructToMat(APVelJ) - JAJJ*StructToMat(JPVel));
	err(end+1) = norm(StructToMat(APVelA) - JAJA*StructToMat(JPVel));
	err(end+1) = norm(JAJJ - JAJA);
	err(end+1) = tolScale*tmpA;
	err(end+1) = tolScale*tmpB;
	if any(abs(err) > Tol)
		warning('There is a problem!');
		disp(err);
		pause;
	end
end
disp(['Maximum dAPVel error multiplier: ' num2str(maxderr/dt)]);
fprintf('\n');

% Test InvFromFootVel
disp('TEST: InvFromFootVel');
tolScale = Tol / (dt*10);
maxderr = 0;
for k = 1:N
	limbSign = RandLimbSign();
	IP = RandInvPose(false, limbSign, RM);
	FFP = FootFloorPoint(IP, limbSign, RM);
	FootVel = 2*rand*RandUnitVec(1,6);
	[IPVel, JIF] = InvFromFootVel(FootVel, IP.footRot, limbSign, RM);
	dFFP = FFP + dt*FootVel(1:3)';
	dfootRot = QuatMult(QuatFromAxis(FootVel(4:6), dt*norm(FootVel(4:6))), IP.footRot);
	dIP = InvFromFFPRot(dFFP, dfootRot, limbSign, RM);
	dAngVel = 2*QuatMult((dIP.footRot - IP.footRot)/dt, QuatInv(IP.footRot));
	dAngVel = dAngVel(2:4);
	dIPVel = [(dIP.anklePos - IP.anklePos)/dt dAngVel]';
	tmpA = norm(dIPVel - StructToMat(IPVel,0));
	maxderr = max([maxderr tmpA]);

	err = [];
	err(end+1) = norm(StructToMat(IPVel,0) - JIF*FootVel);
	err(end+1) = norm(IPVel.footAngVel - FootVel(4:6)');
	err(end+1) = tolScale*tmpA;
	if any(abs(err) > Tol)
		warning('There is a problem!');
		disp(err);
		pause;
	end
end
disp(['Maximum dFootVel error multiplier: ' num2str(maxderr/dt)]);
fprintf('\n');

% Test FootFromInvVel
disp('TEST: FootFromInvVel');
tolScale = Tol / (dt*10);
maxderr = 0;
for k = 1:N
	limbSign = RandLimbSign();
	IP = RandInvPose(false, limbSign, RM);
	FFP = FootFloorPoint(IP, limbSign, RM);
	IPVel = InversePoseVel(RandVec(1,2), RandVec(1,2));
	[FootVel, JFI] = FootFromInvVel(IPVel, IP.footRot, limbSign, RM);
	dIPanklePos = IP.anklePos + dt*IPVel.ankleVel;
	dIPfootRot = QuatMult(QuatFromAxis(IPVel.footAngVel, dt*norm(IPVel.footAngVel)), IP.footRot);
	dIP = InversePose(dIPanklePos, dIPfootRot);
	dFFP = FootFloorPoint(dIP, limbSign, RM);
	dFootVel = [(dFFP - FFP)/dt IPVel.footAngVel]';
	tmpA = norm(dFootVel - FootVel);
	maxderr = max([maxderr tmpA]);

	err = [];
	err(end+1) = norm(FootVel(:) - JFI*StructToMat(IPVel,0));
	err(end+1) = norm(FootVel(4:6)' - IPVel.footAngVel);
	err(end+1) = tolScale*tmpA;
	if any(abs(err) > Tol)
		warning('There is a problem!');
		disp(err);
		pause;
	end
end
disp(['Maximum dFootVel error multiplier: ' num2str(maxderr/dt)]);
fprintf('\n');

% Test InvFromAbsVel
disp('TEST: InvFromAbsVel');
tolScale = Tol / (dt*5000);
maxderr = 0;
for k = 1:N
	limbSign = RandLimbSign();
	AP = RandAbsPose(false);
	AP.retraction = max(AP.retraction, 0.005);
	APVel = RandAbsPose(false);
	[tIP, thipPRPos] = InvFromAbs(AP, limbSign, RM);
	if tIP.footRot(1) < 0
		tIP.footRot = -tIP.footRot;
	end
	tFFP = FootFloorPoint(tIP, limbSign, RM);
	[IPVel, FootVel, JIA, JFA, FFP, IP, hipPRPos] = InvFromAbsVel(APVel, AP, limbSign, RM);
	if IP.footRot(1) < 0
		IP.footRot = -IP.footRot;
	end
	dAP = StructFn(@(X,V) X + dt*V, AP, APVel);
	dIP = InvFromAbs(dAP, limbSign, RM);
	if dIP.footRot(1) < 0
		dIP.footRot = -dIP.footRot;
	end
	dFFP = FootFloorPoint(dIP, limbSign, RM);
	dAngVel = 2*QuatMult((dIP.footRot - tIP.footRot)/dt, QuatInv(tIP.footRot));
	dAngVel = dAngVel(2:4);
	dIPVel = [(dIP.anklePos - tIP.anklePos)/dt dAngVel]';
	dFootVel = [(dFFP - tFFP)/dt dAngVel]';
	tmpA = norm(dIPVel - StructToMat(IPVel,0));
	tmpB = norm(dFootVel - FootVel);
	maxderr = max([maxderr tmpA tmpB]);
	[cFootVel, JFI] = FootFromInvVel(IPVel, IP.footRot, limbSign, RM);
	[cIPVel, JIF] = InvFromFootVel(FootVel, IP.footRot, limbSign, RM);

	err = [];
	err(end+1) = norm(StructToMat(IPVel,0) - JIA*StructToMat(APVel));
	err(end+1) = norm(FootVel(:) - JFA*StructToMat(APVel));
	err(end+1) = norm(cFootVel - FootVel);
	err(end+1) = StructDiffNorm(cIPVel, IPVel);
	err(end+1) = norm(JFI*JIF - eye(6));
	err(end+1) = norm(JFI*JIA - JFA);
	err(end+1) = norm(JIF*JFA - JIA);
	err(end+1) = norm(FFP - tFFP);
	err(end+1) = InvDiffNorm(IP, tIP);
	err(end+1) = norm(hipPRPos - thipPRPos);
	err(end+1) = tolScale*tmpA;
	err(end+1) = tolScale*tmpB;
	if any(abs(err) > Tol)
		warning('There is a problem!');
		disp(err);
		pause;
	end
end
disp(['Maximum dIPVel error multiplier: ' num2str(maxderr/dt)]);
fprintf('\n');

% Test AbsFromInvVel
disp('TEST: AbsFromInvVel');
tolScale = Tol / (dt*50000);
maxderr = 0;
for k = 1:N
	limbSign = RandLimbSign();
	legRetLim = [0.005 0.8]; % Leg retraction limits
	while true
		AP = RandAbsPose(true);
		AP.retraction = coerce(AP.retraction, legRetLim(1), legRetLim(2)); % Limit the leg retraction
		[~, AP] = CanonPose(AP, limbSign, RM);
		if AP.retraction < legRetLim(1) || AP.retraction > legRetLim(2)
			continue
		end
		JP = JointFromAbs(AP);
		if abs(JP.anklePitch) > pi/2 || abs(JP.ankleRoll) > pi/2
			continue
		end
		[tIP, thipPRPos] = InvFromAbs(AP, limbSign, RM);
		if tIP.anklePos(3) / (2*RM.legLinkLength) >= 0.80 % Limit height of ankle (ensure not too high)
			continue
		end
		legaxis = tIP.anklePos - thipPRPos;
		legaxis = legaxis / norm(legaxis);
		footx = QuatRotVec(tIP.footRot, [1 0 0]);
		footx = footx / norm(footx);
		if acos(abs(dot(legaxis, footx))) < 0.25 % Limit leg axis/foot x-axis parallelism (enforce minimum angle between the two)
			continue
		end
		break
	end
	if tIP.footRot(1) < 0
		tIP.footRot = -tIP.footRot;
	end
	tFFP = FootFloorPoint(tIP, limbSign, RM);
	IPVel = InversePoseVel(RandVec(1,0.5)', RandVec(1,0.5)');
	[FootVel, JFI] = FootFromInvVel(IPVel, tIP.footRot, limbSign, RM);
	[APVelI, JIAI, JFAI, JAII, JAFI, FFPI, IPI, hipPRPosI] = AbsFromInvVel(IPVel, AP, limbSign, RM);
	if IPI.footRot(1) < 0
		IPI.footRot = -IPI.footRot;
	end
	[APVelF, JIAF, JFAF, JAIF, JAFF, FFPF, IPF, hipPRPosF] = AbsFromInvVel(FootVel, AP, limbSign, RM);
	if IPF.footRot(1) < 0
		IPF.footRot = -IPF.footRot;
	end
	dIPanklePos = tIP.anklePos + dt*IPVel.ankleVel;
	dIPfootRot = QuatMult(QuatFromAxis(IPVel.footAngVel, dt*norm(IPVel.footAngVel)), tIP.footRot);
	dIP = InversePose(dIPanklePos, dIPfootRot);
	if dIP.footRot(1) < 0
		dIP.footRot = -dIP.footRot;
	end
	dAP = AbsFromInv(dIP, limbSign, RM);
	dAPVel = StructFn(@(X,dX) picut(dX - X)/dt, AP, dAP);
	tmpA = AbsDiffNorm(dAPVel, APVelI);
	tmpB = AbsDiffNorm(dAPVel, APVelF);
	maxderr = max([maxderr tmpA tmpB]);

	err = [];
	err(end+1) = 1e-2*AbsDiffNorm(APVelI, APVelF); % Scaled to loosen the tolerance as dependent on JIA/JFA condition number
	err(end+1) = norm(StructToMat(APVelI) - JIAI\StructToMat(IPVel,0));
	err(end+1) = norm(StructToMat(APVelF) - JFAF\FootVel);
	err(end+1) = norm(StructToMat(APVelI) - JAII*StructToMat(IPVel,0));
	err(end+1) = norm(StructToMat(APVelF) - JAFF*FootVel);
	err(end+1) = norm(JIAI - JIAF);
	err(end+1) = norm(JFAI - JFAF);
	err(end+1) = norm(JAII - JAIF);
	err(end+1) = norm(JAFI - JAFF);
	err(end+1) = norm(JFI*JIAI - JFAF);
	err(end+1) = norm(JFI\JFAF - JIAI);
	err(end+1) = norm(JAFF*JFAF - eye(6));
	err(end+1) = norm(JAII*JIAI - eye(6));
	err(end+1) = norm(FFPI - tFFP);
	err(end+1) = norm(FFPF - tFFP);
	err(end+1) = InvDiffNorm(IPI, tIP);
	err(end+1) = InvDiffNorm(IPF, tIP);
	err(end+1) = norm(hipPRPosI - thipPRPos);
	err(end+1) = norm(hipPRPosF - thipPRPos);
	err(end+1) = tolScale*tmpA;
	err(end+1) = tolScale*tmpB;
	if any(abs(err) > Tol)
		warning('There is a problem!');
		disp(err);
		pause;
	end
end
disp(['Maximum dAPVel error multiplier: ' num2str(maxderr/dt)]);
fprintf('\n');

% Test InvFromJointVel
disp('TEST: InvFromJointVel');
tolScale = Tol / (dt*5000);
maxderr = 0;
for k = 1:N
	limbSign = RandLimbSign();
	JP = RandJointPose(false);
	JP.kneePitch = max(pi*rand, 0.05) + 2*pi*randi([-3 2]);
	JPVel = RandJointPose(false);
	[tIP, thipPRPos, tkneePos] = InvFromJoint(JP, limbSign, RM);
	if tIP.footRot(1) < 0
		tIP.footRot = -tIP.footRot;
	end
	tFFP = FootFloorPoint(tIP, limbSign, RM);
	[IPVel, FootVel, JIJ, JFJ, FFP, IP, hipPRPos, kneePos] = InvFromJointVel(JPVel, JP, limbSign, RM);
	if IP.footRot(1) < 0
		IP.footRot = -IP.footRot;
	end
	dJP = StructFn(@(X,V) X + dt*V, JP, JPVel);
	dIP = InvFromJoint(dJP, limbSign, RM);
	if dIP.footRot(1) < 0
		dIP.footRot = -dIP.footRot;
	end
	dFFP = FootFloorPoint(dIP, limbSign, RM);
	dAngVel = 2*QuatMult((dIP.footRot - tIP.footRot)/dt, QuatInv(tIP.footRot));
	dAngVel = dAngVel(2:4);
	dIPVel = [(dIP.anklePos - tIP.anklePos)/dt dAngVel]';
	dFootVel = [(dFFP - tFFP)/dt dAngVel]';
	tmpA = norm(dIPVel - StructToMat(IPVel,0));
	tmpB = norm(dFootVel - FootVel);
	maxderr = max([maxderr tmpA tmpB]);
	[cFootVel, JFI] = FootFromInvVel(IPVel, IP.footRot, limbSign, RM);
	[cIPVel, JIF] = InvFromFootVel(FootVel, IP.footRot, limbSign, RM);

	err = [];
	err(end+1) = norm(StructToMat(IPVel,0) - JIJ*StructToMat(JPVel));
	err(end+1) = norm(FootVel(:) - JFJ*StructToMat(JPVel));
	err(end+1) = norm(cFootVel - FootVel);
	err(end+1) = StructDiffNorm(cIPVel, IPVel);
	err(end+1) = norm(JFI*JIF - eye(6));
	err(end+1) = norm(JFI*JIJ - JFJ);
	err(end+1) = norm(JIF*JFJ - JIJ);
	err(end+1) = norm(FFP - tFFP);
	err(end+1) = InvDiffNorm(IP, tIP);
	err(end+1) = norm(hipPRPos - thipPRPos);
	err(end+1) = norm(kneePos - tkneePos);
	err(end+1) = tolScale*tmpA;
	err(end+1) = tolScale*tmpB;
	if any(abs(err) > Tol)
		warning('There is a problem!');
		disp(err);
		pause;
	end
end
disp(['Maximum dIPVel error multiplier: ' num2str(maxderr/dt)]);
fprintf('\n');

% Test JointFromInvVel
disp('TEST: JointFromInvVel');
tolScale = Tol / (dt*50000);
maxderr = 0;
for k = 1:N
	limbSign = RandLimbSign();
	legRetLim = [0.005 0.8]; % Leg retraction limits
	kneePitchLim = 2*acos(1 - legRetLim);
	while true
		JP = RandJointPose(true);
		JP.kneePitch = coerce(JP.kneePitch, kneePitchLim(1), kneePitchLim(2)); % Limit knee pitch (i.e. leg retraction)
		JP = CanonPose(JP, limbSign, RM);
		if JP.kneePitch < kneePitchLim(1) || JP.kneePitch > kneePitchLim(2)
			continue
		end
		[tIP, thipPRPos, tkneePos] = InvFromJoint(JP, limbSign, RM);
		if tIP.anklePos(3) / (2*RM.legLinkLength) >= 0.80 % Limit height of ankle (ensure not too high)
			continue
		end
		legaxis = tIP.anklePos - thipPRPos;
		legaxis = legaxis / norm(legaxis);
		footx = QuatRotVec(tIP.footRot, [1 0 0]);
		footx = footx / norm(footx);
		if acos(abs(dot(legaxis, footx))) < 0.25 % Limit leg axis/foot x-axis parallelism (enforce minimum angle between the two)
			continue
		end
		break
	end
	if tIP.footRot(1) < 0
		tIP.footRot = -tIP.footRot;
	end
	tFFP = FootFloorPoint(tIP, limbSign, RM);
	IPVel = InversePoseVel(RandVec(1,0.5)', RandVec(1,0.5)');
	[FootVel, JFI] = FootFromInvVel(IPVel, tIP.footRot, limbSign, RM);
	[JPVelI, JIJI, JFJI, JJII, JJFI, FFPI, IPI, hipPRPosI, kneePosI] = JointFromInvVel(IPVel, JP, limbSign, RM);
	if IPI.footRot(1) < 0
		IPI.footRot = -IPI.footRot;
	end
	[JPVelF, JIJF, JFJF, JJIF, JJFF, FFPF, IPF, hipPRPosF, kneePosF] = JointFromInvVel(FootVel, JP, limbSign, RM);
	if IPF.footRot(1) < 0
		IPF.footRot = -IPF.footRot;
	end
	dIPanklePos = tIP.anklePos + dt*IPVel.ankleVel;
	dIPfootRot = QuatMult(QuatFromAxis(IPVel.footAngVel, dt*norm(IPVel.footAngVel)), tIP.footRot);
	dIP = InversePose(dIPanklePos, dIPfootRot);
	if dIP.footRot(1) < 0
		dIP.footRot = -dIP.footRot;
	end
	dJP = JointFromInv(dIP, limbSign, RM);
	dJPVel = StructFn(@(X,dX) picut(dX - X)/dt, JP, dJP);
	tmpA = JointDiffNorm(dJPVel, JPVelI);
	tmpB = JointDiffNorm(dJPVel, JPVelF);
	maxderr = max([maxderr tmpA tmpB]);

	err = [];
	err(end+1) = 1e-2*JointDiffNorm(JPVelI, JPVelF); % Scaled to loosen the tolerance as dependent on JIJ/JFJ condition number
	err(end+1) = norm(StructToMat(JPVelI) - JIJI\StructToMat(IPVel,0));
	err(end+1) = norm(StructToMat(JPVelF) - JFJF\FootVel);
	err(end+1) = norm(StructToMat(JPVelI) - JJII*StructToMat(IPVel,0));
	err(end+1) = norm(StructToMat(JPVelF) - JJFF*FootVel);
	err(end+1) = norm(JIJI - JIJF);
	err(end+1) = norm(JFJI - JFJF);
	err(end+1) = norm(JJII - JJIF);
	err(end+1) = norm(JJFI - JJFF);
	err(end+1) = norm(JFI*JIJI - JFJF);
	err(end+1) = norm(JFI\JFJF - JIJI);
	err(end+1) = norm(JJFF*JFJF - eye(6));
	err(end+1) = norm(JJII*JIJI - eye(6));
	err(end+1) = norm(FFPI - tFFP);
	err(end+1) = norm(FFPF - tFFP);
	err(end+1) = InvDiffNorm(IPI, tIP);
	err(end+1) = InvDiffNorm(IPF, tIP);
	err(end+1) = norm(hipPRPosI - thipPRPos);
	err(end+1) = norm(hipPRPosF - thipPRPos);
	err(end+1) = norm(kneePosI - tkneePos);
	err(end+1) = norm(kneePosF - tkneePos);
	err(end+1) = tolScale*tmpA;
	err(end+1) = tolScale*tmpB;
	if any(abs(err) > Tol)
		warning('There is a problem!');
		disp(err);
		pause;
	end
end
disp(['Maximum dJPVel error multiplier: ' num2str(maxderr/dt)]);
fprintf('\n');
% EOF