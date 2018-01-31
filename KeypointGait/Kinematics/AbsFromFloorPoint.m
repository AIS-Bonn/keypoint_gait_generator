% AbsFromFloorPoint.m - Philipp Allgeuer - 14/09/16
% Calculate an abstract space pose from a floor point, foot angles and leg angle Z.
%
% function [AP] = AbsFromFloorPoint(FFP, footAngle, legAngleZ, limbSign, RM)
%
function [AP] = AbsFromFloorPoint(FFP, footAngle, legAngleZ, limbSign, RM)

	% Input arguments
	if nargin < 3
		legAngleZ = 0;
	end
	if nargin < 4
		limbSign = 1; % 1 = Left leg, -1 = Right leg
	end
	limbSign = sgn(limbSign);
	if nargin < 5
		RM = RobotModel;
	end

	% Variable aliases
	L = 2.0*RM.legLinkLength;
	ex = footAngle(1);
	ey = footAngle(2);
	fx = RM.footOffsetX;
	fy = limbSign*RM.footOffsetY;
	fz = RM.footOffsetZ;

	% Unrotate the foot floor point by the leg angle Z, and make it relative to the hip point
	claz = cos(legAngleZ);
	slaz = sin(legAngleZ);
	p = [claz*FFP(1)+slaz*FFP(2); claz*FFP(2)-slaz*FFP(1); FFP(3)-L]; % In yaw frame coordinates relative to the hip point

	% Calculate the leg angle X
	sex = sin(ex);
	cex = cos(ex);
	A = fy*cex + fz*sex - p(2);
	B = fy*sex - fz*cex - p(3);
	legAngleX = atan2(-A,B);
	if legAngleX > pi/2
		legAngleX = legAngleX - pi;
	elseif legAngleX < -pi/2
		legAngleX = legAngleX + pi;
	end

	% Further unrotate the foot floor point by the leg angle X
	clax = cos(legAngleX);
	slax = sin(legAngleX);
	q = [p(1); clax*p(3)-slax*p(2)]; % In roll plane xz-coords relative to the hip point

	% Calculate the ankle point
	sey = sin(ey);
	cey = cos(ey);
	ankleRoll = ex - legAngleX;
	fztilde = fz*cos(ankleRoll) - fy*sin(ankleRoll);
	a = [q(1)-fx*cey+fztilde*sey; q(2)+fx*sey+fztilde*cey]; % In roll plane xz-coords relative to the hip point

	% Calculate the leg retraction and leg angle Y
	legRetraction = coerce(1.0 - norm(a)/L, 0.0, 1.0);
	legAngleY = atan2(-a(1), -a(2));

	% Construct the output abstract pose
	AP = AbstractPose([legAngleX legAngleY legAngleZ], [ex ey], legRetraction);

end
% EOF