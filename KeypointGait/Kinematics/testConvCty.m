% Test the continuity of the kinematic conversions

N = 10;
fprintf('N = %d\n', N);

RM = RobotModel;
limbSign = RandLimbSign();
stepSize = 0.002;

Ldbl = 2*RM.legLinkLength;
fx = RM.footOffsetX;
fy = limbSign*RM.footOffsetY;
fz = RM.footOffsetZ;
reach = Ldbl + norm([fx fy fz]);

Xrange = [-reach reach];
Yrange = [-reach reach];
Zrange = Ldbl + [-reach -norm([fx fy fz])-0.4*Ldbl];
Trange = [pi/3 pi 0.6];

FFP = FootFloorPoint(InversePose, limbSign, RM);
Rot = QuatIdentity;

clear IPL JPL test JPVelA JPVelB

IP = InvFromFFPRot(FFP, Rot, limbSign, RM);
[JP, AP, exact] = JointFromInv(IP, limbSign, RM);
PlotPose(JP, 67, limbSign, false, true, RM);
IPL(1) = IP;
JPL(1) = JP;
JPVelA(1) = JointPose;
JPVelB(1) = JointPose;
test(1,:) = [0 0];

pause

for k = 1:N
	FFPtarget = [Xrange(1) + rand*(Xrange(2) - Xrange(1)), Yrange(1) + rand*(Yrange(2) - Yrange(1)), Zrange(1) + rand*(Zrange(2) - Zrange(1))];
	tiltTarget = (2*rand(1,3) - 1).*Trange;
	quatTarget = QuatFromTilt(tiltTarget);

	while true
		done = false;
		oldFFP = FFP;
		oldRot = Rot;

		distSteps = norm(FFPtarget - FFP) / stepSize;
		if distSteps <= 1
			FFP = FFPtarget;
			Rot = quatTarget;
			done = true;
		else
			FFP = FFP + (FFPtarget - FFP) / distSteps;
			Rot = QuatSlerp(Rot, quatTarget, 1/distSteps);
		end

		IP = InvFromFFPRot(FFP, Rot, limbSign, RM);
		[JP, AP, exact] = JointFromInv(IP, limbSign, RM);
		PlotPose(JP, 67, limbSign, true, true, RM);
		IPL(end+1) = IP;
		JPL(end+1) = JP;

		if exact
			title('exact');
		else
			title('NOT exact');
		end

		hipPRPos = CalcHipPose(JP, limbSign, RM);
		legaxis = IP.anklePos - hipPRPos;
		legaxis = legaxis / norm(legaxis);
		footx = QuatRotVec(IP.footRot, [1 0 0]);
		footx = footx / norm(footx);
		test(end+1,:) = [IP.anklePos(3)/Ldbl acos(abs(dot(legaxis, footx)))];

		if JP.kneePitch > 0.2
			footVel = FFP - oldFFP; % Change in ankle pos in this step
			RotRel = EnsureQuat(QuatMult(Rot, QuatInv(oldRot)), 0, true);
			footAngVel = 2*acos(RotRel(1))*(RotRel(2:4)/norm(RotRel(2:4))); % Angular velocity (per step)
			FFPVel = [footVel(:); footAngVel(:)];
			JPVelA(end+1) = JointFromInvVel(FFPVel, StructFn(@(a,b) 0.5*(a+b), JPL(end-1), JPL(end)), limbSign, RM);
			JPVelB(end+1) = StructFn(@(a,b) (b-a), JPL(end-1), JPL(end));
		end

		pause(0.02);

		if done
			break
		end
	end
end

figure(68);
plot([JPL.hipYaw; JPL.hipRoll; JPL.hipPitch]');
legend('Hip Yaw', 'Hip Roll', 'Hip Pitch');
grid on;

clear cJP cJPL JPLA JPLB JPLC JPLD
M = length(JPL);
for k = 1:M
	[~, ~, cJPL{k}] = CanonPose(JPL(k), limbSign, RM);
	JPLA(k) = cJPL{k}(1);
	JPLB(k) = cJPL{k}(2);
	JPLC(k) = cJPL{k}(3);
	JPLD(k) = cJPL{k}(4);
end
figure(70);plot([JPLA.hipYaw; JPLA.hipRoll; JPLA.hipPitch]');ylim([-pi pi]);grid on;
figure(71);plot([JPLB.hipYaw; JPLB.hipRoll; JPLB.hipPitch]');ylim([-pi pi]);grid on;
figure(72);plot([JPLC.hipYaw; JPLC.hipRoll; JPLC.hipPitch]');ylim([-pi pi]);grid on;
figure(73);plot([JPLD.hipYaw; JPLD.hipRoll; JPLD.hipPitch]');ylim([-pi pi]);grid on;

figure(74);plot(test);grid on;

figure(75);
plot([JPVelA.hipYaw; JPVelA.hipRoll; JPVelA.hipPitch; JPVelA.kneePitch; JPVelA.anklePitch; JPVelA.ankleRoll]','-');
hold on
ax = get(75,'CurrentAxes');
ax.ColorOrderIndex = 1;
plot([JPVelB.hipYaw; JPVelB.hipRoll; JPVelB.hipPitch; JPVelB.kneePitch; JPVelB.anklePitch; JPVelB.ankleRoll]','--','LineWidth',1.3);
hold off
grid on

% You can use the following to replay and check a segment of the test
% AnimPose(JPL(1080:1430), 0, 0, 69, limbSign, RM)
