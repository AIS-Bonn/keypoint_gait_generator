% Initialisation
N = 1000000;
RM = RobotModel;

% Test AP -> FFP -> AP
disp('AP -> FFP -> AP');
maxerr = -1;
maxAP = AbstractPose;
for k = 1:N
	ls = sgn(rand-0.5);
	AP = RandAbsPose(true);
	IP = InvFromAbs(AP, ls, RM);
	FFP = FootFloorPoint(IP, ls, RM);
	APtmp = AbsFromFloorPoint(FFP, [AP.footAngleX AP.footAngleY], AP.angleZ, ls);
	errvec = [APtmp.angleX-AP.angleX APtmp.angleY-AP.angleY APtmp.retraction-AP.retraction];
	err = norm(errvec);
	if err > maxerr
		maxerr = err;
		maxerrvec = errvec;
		maxAP = AP;
	end
end
disp('Maximum error =');
disp(maxerr);
disp('Error in (AngleX, AngleY, Retraction) =');
disp(maxerrvec);
disp('For AP =');
disp(maxAP);
disp(' ');

% Test FFP -> AP -> FFP
disp('FFP -> AP -> FFP');
maxerrv = -1;
maxv.FFP = [0 0 0];
maxv.footAngle = [0 0];
maxv.legAngleZ = 0;
maxv.ls = 1;
for k = 1:N
	v.FFP = [0.8*rand-0.4 0.8*rand-0.4 0.4*rand];
	v.footAngle = pi/2*(2*rand(1,2)-1);
	v.legAngleZ = pi/2*(2*rand-1);
	v.ls = sgn(rand-0.5);
	AP = AbsFromFloorPoint(v.FFP, v.footAngle, v.legAngleZ, v.ls, RM);
	IP = InvFromAbs(AP, v.ls, RM);
	FFPtmp = FootFloorPoint(IP, v.ls, RM);
	if AP.retraction == 0
		ray = IP.anklePos - [0 0 2*RM.legLinkLength];
		ray = ray / norm(ray);
		errray = v.FFP - FFPtmp;
		errray = errray - dot(errray,ray)*ray;
		err = norm(errray);
	else
		err = norm(v.FFP - FFPtmp);
	end
	if err > maxerrv
		maxerrv = err;
		maxv = v;
	end
end
disp('Maximum error =');
disp(maxerrv);
disp('For v =');
disp(maxv);
disp(' ');
% EOF