% GenLegMotion.m - Philipp Allgeuer - 01/09/16
%
% function [u, JPA, APA, JPP, APP, IPF] = GenLegMotion(u, gcv, RM, config)
%
function [u, JPA, APA, JPP, APP, IPF] = GenLegMotion(u, gcv, RM, config)

	% Input variables
	if isscalar(u)
		utmp = 0:0.03:u;
		if utmp(end) ~= u
			u = [utmp u];
		else
			u = utmp;
		end
	end
	N = numel(u);
	if nargin < 2
		gcv = [0 0 0];
	end
	if nargin < 3
		RM = RobotModel;
	end
	if nargin < 4
		config = ConfigVars;
	end

	% Initialise joint pose array
	APAL(N) = AbstractLegMotion(AbstractPose, gcv, 0.0, 1, config);
	APAR(N) = AbstractLegMotion(AbstractPose, gcv, 0.0, -1, config);
	IPFL = zeros(N,3);
	IPFR = zeros(N,3);
	IPFD = zeros(N,3);
	JPAL(N) = JointPose;
	JPAR(N) = JointPose;

	% Generate the required data
	for k = 1:N
		gaitPhase = picut(u(k));
		APL = AbstractHaltPose(1, config);
		APR = AbstractHaltPose(-1, config);
		APL = AbstractLegMotion(APL, gcv, gaitPhase, 1, config);
		APR = AbstractLegMotion(APR, gcv, gaitPhase, -1, config);
		APAL(k) = APL;
		APAR(k) = APR;
		IPL = InvFromAbs(APL, 1, RM);
		IPR = InvFromAbs(APR, -1, RM);
		IPL = InverseLegMotion(IPL, gcv, gaitPhase, 1, config);
		IPR = InverseLegMotion(IPR, gcv, gaitPhase, -1, config);
		IPFL(k,:) = FootFloorPoint(IPL, 1, RM);
		IPFR(k,:) = FootFloorPoint(IPR, -1, RM);
		IPFD(k,:) = IPFL(k,:) - IPFR(k,:);
		JPL = JointFromInv(IPL, 1, RM);
		JPR = JointFromInv(IPR, -1, RM);
		JPAL(k) = JPL;
		JPAR(k) = JPR;
	end

	% Populate the required outputs
	JPA = {JPAL, JPAR};
	APA = {APAL, APAR};
	JPP = {SAFromAS(JPAL), SAFromAS(JPAR)};
	APP = {SAFromAS(APAL), SAFromAS(APAR)};
	IPF = {IPFL, IPFR, IPFD};

end
% EOF