% Test the condition number of the Jacobians for velocity space conversions
%
% Observations:
% - For a fully extended leg, the JAI condition number stays stable, while the JJI condition number explodes,
%   but otherwise the condition numbers are always at least in the same order of magnitude.
% - Both condition numbers can explode for a fully retracted leg.
% - Both condition numbers can explode for an ankle point at the same height as the hip.
% - Both condition numbers can explode when the leg axis is almost parallel to the foot x-axis

M = 1000000; % Maximum attempts to get N data points
N = 500;

minCond = 150;

RM = RobotModel;

nullJP = JointPose;
nullAP = AbstractPose;

limbSign = zeros(N,1);
JP(N) = JointPose;
AP(N) = AbstractPose;
IP(N) = InversePose;

test = zeros(N,2);

JIJ = zeros(6,6,N);
JFJ = zeros(6,6,N);
JIA = zeros(6,6,N);
JFA = zeros(6,6,N);

condJIJ = zeros(N,1);
condJFJ = zeros(N,1);
condJIA = zeros(N,1);
condJFA = zeros(N,1);

k = 1;
for m = 1:M
	limbSign(k) = RandLimbSign;
	JP(k) = RandJointPose;

	legRetLim = [0.005 0.8]; % Leg retraction limits
	kneePitchLim = 2*acos(1 - legRetLim);
	JP(k).kneePitch = coerce(JP(k).kneePitch, kneePitchLim(1), kneePitchLim(2)); % Limit knee pitch (i.e. leg retraction)
	AP(k) = AbsFromJoint(JP(k));

	[IP(k), hipPRPos] = InvFromJoint(JP(k), limbSign(k), RM);
	test(k,1) = IP(k).anklePos(3) / (2*RM.legLinkLength);
	if test(k,1) >= 0.85 % Limit height of ankle (ensure not too high)
		continue
	end
	legaxis = IP(k).anklePos - hipPRPos;
	legaxis = legaxis / norm(legaxis);
	footx = QuatRotVec(IP(k).footRot, [1 0 0]);
	footx = footx / norm(footx);
	test(k,2) = acos(abs(dot(legaxis, footx)));
	if test(k,2) < 0.25 % Limit leg axis/foot x-axis parallelism (enforce minimum angle between the two)
		continue
	end

	[~, ~, JIJ(:,:,k), JFJ(:,:,k)] = InvFromJointVel(nullJP, JP(k), limbSign(k), RM);
	[~, ~, JIA(:,:,k), JFA(:,:,k)] = InvFromAbsVel(nullAP, AP(k), limbSign(k), RM);

	condJIJ(k) = cond(JIJ(:,:,k));
	condJFJ(k) = cond(JFJ(:,:,k));
	condJIA(k) = cond(JIA(:,:,k));
	condJFA(k) = cond(JFA(:,:,k));

	if all([condJIJ(k) condJFJ(k) condJIA(k) condJFA(k)] < minCond)
		continue
	end

	k = k + 1;

	if k > N
		break;
	end
end
if m == M
	condJIJ(k) = 0;
	condJFJ(k) = 0;
	condJIA(k) = 0;
	condJFA(k) = 0;
end
fprintf('Took %d tries to get %d samples\n', m, k-1);

figure(45);
plot([condJIJ condJFJ condJIA condJFA]); % Check test(k,:) for any k that has high condition numbers...
legend('Joint to Inverse Vel', 'Joint to Foot Vel', 'Abstract to Inverse Vel', 'Abstract to Foot Vel');
grid on;

figure(46);
scatter3(test(:,1), test(:,2), condJIJ, 'o');
hold on;
scatter3(test(:,1), test(:,2), condJFJ, 'x');
hold off;
a = get(46, 'CurrentAxes');
a.ZScale = 'log';
xlim([0 1]);
ylim([0.2 pi/2]);
grid on;

figure(47);
scatter3(test(:,1), test(:,2), condJIA, 'o');
hold on;
scatter3(test(:,1), test(:,2), condJFA, 'x');
hold off;
a = get(47, 'CurrentAxes');
a.ZScale = 'log';
xlim([0 1]);
ylim([0.2 pi/2]);
grid on;
% EOF