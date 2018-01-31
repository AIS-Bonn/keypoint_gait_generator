% Test kinematic conversions between joint, abstract and inverse space

N = 100000;
fprintf('N = %d\n', N);

RM = RobotModel;

tic
disp('TEST: Inv -> CalcHipYaw');
maxerr = -Inf;
for k = 1:N
	limbSign = RandLimbSign();
	JP = RandJointPose(false);
	IP = InvFromJoint(JP, limbSign, RM);
	[hipYaw, hipYawAlt] = CalcHipYaw(IP, limbSign, RM);
	err = min(abs(picut(hipYaw - JP.hipYaw)), abs(picut(hipYawAlt - JP.hipYaw)));
	if abs(err) > 1e-8
		disp(err);
		warning('We have a problem!');
		pause
	end
	if abs(err) > maxerr
		maxerr = abs(err);
	end
end
toc
fprintf('Max error = %g\n\n', maxerr);

tic
disp('TEST: Joint -> CanonPose');
canonInexactCount = 0;
inexactCount = 0;
maxerr = -Inf;
for k = 1:N
	limbSign = RandLimbSign();
	P = RandJointPose(false);
	IP = InvFromJoint(P, limbSign, RM);
	[JP, AP, JPL, APL, I, canonExact, exact] = CanonPose(P, limbSign, RM);
	if ~canonExact
		canonInexactCount = canonInexactCount + 1;
	end
	if ~exact
		inexactCount = inexactCount + 1;
	end
	maxCostL = inf(1,4);
	errvec = inf(1, 38);
	errvec(1) = JointDiffNorm(JP, JointFromAbs(AP));
	errvec(2) = AbsDiffNorm(AbsFromJoint(JP), AP);
	[IPTest, hipPRPosTest] = InvFromJoint(JP, limbSign, RM);
	errvec(3) = InvDiffNormWeak(IP, IPTest, hipPRPosTest, canonExact);
	[IPTest, hipPRPosTest] = InvFromAbs(AP, limbSign, RM);
	errvec(4) = InvDiffNormWeak(IP, IPTest, hipPRPosTest, canonExact);
	errvec(5) = StructDiffNorm(JP, PicutJoint(JP));
	errvec(6) = StructDiffNorm(AP, PicutAbs(AP));
	for m = 1:4
		errvec(7*m + 0) = JointDiffNorm(JPL(m), JointFromAbs(APL(m)));
		errvec(7*m + 1) = AbsDiffNorm(AbsFromJoint(JPL(m)), APL(m));
		[IPTest, hipPRPosTest] = InvFromJoint(JPL(m), limbSign, RM);
		errvec(7*m + 2) = InvDiffNormWeak(IP, IPTest, hipPRPosTest, (exact || m < 3));
		[IPTest, hipPRPosTest] = InvFromAbs(APL(m), limbSign, RM);
		errvec(7*m + 3) = InvDiffNormWeak(IP, IPTest, hipPRPosTest, (exact || m < 3));
		errvec(7*m + 4) = StructDiffNorm(JPL(m), PicutJoint(JPL(m)));
		errvec(7*m + 5) = StructDiffNorm(APL(m), PicutAbs(APL(m)));
		maxCostL(m) = CalcJointCost(JPL(m));
		if exact || m < 3
			JPtmp = CanonPose(JPL(m), limbSign, RM);
			errvec(7*m + 6) = JointDiffNorm(JPtmp, JP);
		else
			errvec(7*m + 6) = 0;
		end
	end
	errvec(35) = JointDiffNorm(JP, JPL(I));
	errvec(36) = AbsDiffNorm(AP, APL(I));
	errvec(37) = JointDiffNorm(P, JPL(1));
	JPtmp = CanonPose(JP, limbSign, RM);
	if canonExact
		errvec(38) = JointDiffNorm(JPtmp, JP);
	else
		JPtmptmp = CanonPose(JPtmp, limbSign, RM);
		errvec(38) = JointDiffNorm(JPtmptmp, JPtmp);
	end
	maxCost = CalcJointCost(JP);
	err = max(abs(errvec));
	if ~all(maxCost <= maxCostL + 64*eps)
		err = Inf;
	end
	if abs(err) > 1e-8
		disp(err);
		warning('We have a problem!');
		pause
	end
	if abs(err) > maxerr
		maxerr = abs(err);
	end
end
toc
fprintf('%.1f%% of canonical poses could have been inexact\n', 100*inexactCount/N);
fprintf('%.1f%% of canonical poses were inexact\n', 100*canonInexactCount/N);
fprintf('Max error = %g\n\n', maxerr);

tic
disp('TEST: Abs -> CanonPose');
canonInexactCount = 0;
inexactCount = 0;
maxerr = -Inf;
for k = 1:N
	limbSign = RandLimbSign();
	P = RandAbsPose(false);
	IP = InvFromAbs(P, limbSign, RM);
	[JP, AP, JPL, APL, I, canonExact, exact] = CanonPose(P, limbSign, RM);
	if ~canonExact
		canonInexactCount = canonInexactCount + 1;
	end
	if ~exact
		inexactCount = inexactCount + 1;
	end
	maxCostL = inf(1,4);
	errvec = inf(1, 38);
	errvec(1) = JointDiffNorm(JP, JointFromAbs(AP));
	errvec(2) = AbsDiffNorm(AbsFromJoint(JP), AP);
	[IPTest, hipPRPosTest] = InvFromJoint(JP, limbSign, RM);
	errvec(3) = InvDiffNormWeak(IP, IPTest, hipPRPosTest, canonExact);
	[IPTest, hipPRPosTest] = InvFromAbs(AP, limbSign, RM);
	errvec(4) = InvDiffNormWeak(IP, IPTest, hipPRPosTest, canonExact);
	errvec(5) = StructDiffNorm(JP, PicutJoint(JP));
	errvec(6) = StructDiffNorm(AP, PicutAbs(AP));
	for m = 1:4
		errvec(7*m + 0) = JointDiffNorm(JPL(m), JointFromAbs(APL(m)));
		errvec(7*m + 1) = AbsDiffNorm(AbsFromJoint(JPL(m)), APL(m));
		[IPTest, hipPRPosTest] = InvFromJoint(JPL(m), limbSign, RM);
		errvec(7*m + 2) = InvDiffNormWeak(IP, IPTest, hipPRPosTest, (exact || m < 3));
		[IPTest, hipPRPosTest] = InvFromAbs(APL(m), limbSign, RM);
		errvec(7*m + 3) = InvDiffNormWeak(IP, IPTest, hipPRPosTest, (exact || m < 3));
		errvec(7*m + 4) = StructDiffNorm(JPL(m), PicutJoint(JPL(m)));
		errvec(7*m + 5) = StructDiffNorm(APL(m), PicutAbs(APL(m)));
		maxCostL(m) = CalcJointCost(JPL(m));
		if exact || m < 3
			[~, APtmp] = CanonPose(APL(m), limbSign, RM);
			errvec(7*m + 6) = AbsDiffNorm(APtmp, AP);
		else
			errvec(7*m + 6) = 0;
		end
	end
	errvec(35) = JointDiffNorm(JP, JPL(I));
	errvec(36) = AbsDiffNorm(AP, APL(I));
	errvec(37) = AbsDiffNorm(P, APL(1));
	[~, APtmp] = CanonPose(AP, limbSign, RM);
	if canonExact
		errvec(38) = AbsDiffNorm(APtmp, AP);
	else
		[~, APtmptmp] = CanonPose(APtmp, limbSign, RM);
		errvec(38) = AbsDiffNorm(APtmptmp, APtmp);
	end
	maxCost = CalcJointCost(JP);
	err = max(abs(errvec));
	if ~all(maxCost <= maxCostL + 64*eps)
		err = Inf;
	end
	if abs(err) > 1e-8
		disp(err);
		warning('We have a problem!');
		pause
	end
	if abs(err) > maxerr
		maxerr = abs(err);
	end
end
toc
fprintf('%.1f%% of canonical poses could have been inexact\n', 100*inexactCount/N);
fprintf('%.1f%% of canonical poses were inexact\n', 100*canonInexactCount/N);
fprintf('Max error = %g\n\n', maxerr);

tic
disp('TEST: Abs -> Joint -> Abs');
maxerr = -Inf;
for k = 1:N
	AP = RandAbsPose(false);
	JPconv = JointFromAbs(AP);
	APconv = AbsFromJoint(JPconv);
	err = AbsDiffNorm(APconv, AP);
	if abs(err) > 1e-13
		disp(err);
		warning('We have a problem!');
		pause
	end
	if abs(err) > maxerr
		maxerr = abs(err);
	end
end
toc
fprintf('Max error = %g\n\n', maxerr);

tic
disp('TEST: Joint -> Abs -> Joint');
maxerr = -Inf;
for k = 1:N
	JP = RandJointPose(false);
	APconv = AbsFromJoint(JP);
	JPconv = JointFromAbs(APconv);
	err = JointDiffNorm(JPconv, JP);
	if abs(err) > 1e-8
		disp(err);
		warning('We have a problem!');
		pause
	end
	if abs(err) > maxerr
		maxerr = abs(err);
	end
end
toc
fprintf('Max error = %g\n\n', maxerr);

tic
disp('TEST: Inv -> Joint -> Inv');
maxerr = -Inf;
for k = 1:N
	limbSign = RandLimbSign();
	tmp = RandJointPose(false);
	IP = InvFromJoint(tmp, limbSign, RM);
	[JPconv, ~, exact] = JointFromInv(IP, limbSign, RM);
	[IPconv, hipPRPosconv] = InvFromJoint(JPconv, limbSign, RM);
	err = InvDiffNormWeak(IP, IPconv, hipPRPosconv, exact);
	if abs(err) > 1e-8
		disp(err);
		warning('We have a problem!');
		pause
	end
	if abs(err) > maxerr
		maxerr = abs(err);
	end
end
toc
fprintf('Max error = %g\n\n', maxerr);

tic
disp('TEST: Joint -> Inv -> Joint');
inexactCount = 0;
maxerr = -Inf;
for k = 1:N
	while true
		limbSign = RandLimbSign();
		tmp = RandJointPose(false);
		[JP, ~, ~, ~, ~, canonExact] = CanonPose(tmp, limbSign, RM);
		if ~canonExact
			JP = CanonPose(JP, limbSign, RM);
		end
		[IPconv, hipPRPosconv] = InvFromJoint(JP, limbSign, RM);
		if abs(IPconv.anklePos(3)/(2*RM.legLinkLength) - 1) <= 0.05
			continue
		end
		legaxis = IPconv.anklePos - hipPRPosconv;
		legaxis = legaxis / norm(legaxis);
		footx = QuatRotVec(IPconv.footRot, [1 0 0]);
		footx = footx / norm(footx);
		if acos(abs(dot(legaxis, footx))) < 0.2
			continue
		end
		break
	end
	[JPconv, ~, exact] = JointFromInv(IPconv, limbSign, RM);
	if ~exact
		inexactCount = inexactCount + 1;
	end
	err = JointDiffNorm(JPconv, JP);
	if abs(err) > 1e-6 % Knee pitch can be quite sensitive at the stretched leg position (the JP above often has a kneePitch of exactly zero)
		disp(err);
		warning('We have a problem!'); % For poses very close to singularities we can expect that this warning will fire occasionally...
		pause
	end
	if abs(err) > maxerr
		maxerr = abs(err);
	end
end
toc
fprintf('%.1f%% of poses were inexact\n', 100*inexactCount/N);
fprintf('Max error = %g\n\n', maxerr);

tic
disp('TEST: Inv -> Abs -> Inv');
maxerr = -Inf;
for k = 1:N
	limbSign = RandLimbSign();
	tmp = RandAbsPose(false);
	IP = InvFromAbs(tmp, limbSign, RM);
	[APconv, ~, exact] = AbsFromInv(IP, limbSign, RM);
	[IPconv, hipPRPosconv] = InvFromAbs(APconv, limbSign, RM);
	err = InvDiffNormWeak(IP, IPconv, hipPRPosconv, exact);
	if abs(err) > 1e-8
		disp(err);
		warning('We have a problem!');
		pause
	end
	if abs(err) > maxerr
		maxerr = abs(err);
	end
end
toc
fprintf('Max error = %g\n\n', maxerr);

tic
disp('TEST: Abs -> Inv -> Abs');
inexactCount = 0;
maxerr = -Inf;
for k = 1:N
	while true
		limbSign = RandLimbSign();
		tmp = RandAbsPose(false);
		[~, AP, ~, ~, ~, canonExact] = CanonPose(tmp, limbSign, RM);
		if ~canonExact
			[~, AP] = CanonPose(AP, limbSign, RM);
		end
		[IPconv, hipPRPosconv] = InvFromAbs(AP, limbSign, RM);
		if abs(IPconv.anklePos(3)/(2*RM.legLinkLength) - 1) <= 0.05
			continue
		end
		legaxis = IPconv.anklePos - hipPRPosconv;
		legaxis = legaxis / norm(legaxis);
		footx = QuatRotVec(IPconv.footRot, [1 0 0]);
		footx = footx / norm(footx);
		if acos(abs(dot(legaxis, footx))) < 0.2
			continue
		end
		break
	end
	[APconv, ~, exact] = AbsFromInv(IPconv, limbSign, RM);
	if ~exact
		inexactCount = inexactCount + 1;
	end
	err = AbsDiffNorm(APconv, AP);
	if abs(err) > 1e-8
		disp(err);
		warning('We have a problem!'); % For poses very close to singularities we can expect that this warning will fire occasionally...
		pause
	end
	if abs(err) > maxerr
		maxerr = abs(err);
	end
end
toc
fprintf('%.1f%% of poses were inexact\n', 100*inexactCount/N);
fprintf('Max error = %g\n\n', maxerr);
% EOF