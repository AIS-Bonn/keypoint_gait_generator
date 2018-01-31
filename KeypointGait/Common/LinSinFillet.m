% LinSinFillet.m - Philipp Allgeuer - 27/05/15
% Fillet a linear-sinusoid transition
% It is assumed the sine wave is A*sin(B*t). The fillet is added from constant zero to the sine wave,
% with constant acceleration, continuous positions and velocities, and merging into the sine wave at
% time t = T.

% Main function
function [Delta, Success, Coeff] = LinSinFillet(A, B, T, t)

	% Calculate stuff
	[Success, Coeff] = update(A, B, T);

	% Evaluate stuff
	for r = 1:size(t,1)
		for c = 1:size(t,2)
			Delta(r,c) = evaluate(Coeff, t(r,c));
		end
	end

end

% Update function
function [Success, Coeff] = update(A, B, T)

	% Initialise stuff
	Success = false;
	Coeff = [A B 0 0 0 T T]; % A B a b c tPre tPost

	% Update as required
	BT = B*T;
	if T < 1e-6 || abs(B) < 1e-6 || abs(BT) > pi/2 - 1e-6
		Success = false;
		return
	end
	m_A = A;
	m_B = B;
	if abs(A) < 1e-6
		Success = true;
		return
	end
	m_c = m_A*sin(BT);
	m_b = m_A*m_B*cos(BT);
	m_a = m_b*m_b/(4*m_c);
	m_tPre = T - 2*m_c/m_b;
	m_tPost = T;

	% Return results
	Success = true;
	Coeff = [m_A m_B m_a m_b m_c m_tPre m_tPost];

end

% Evaluate function
function [Delta] = evaluate(Coeff, t)

	% Evaluate the delta to the lin-sin
	if ~(t > Coeff(6) && t < Coeff(7))
		Delta = 0.0;
		return
	end
	dt = t - Coeff(7);
	parabola = Coeff(5) + dt*(Coeff(4) + dt*Coeff(3));
	if t <= 0.0
		Delta = parabola;
	else
		Delta = parabola - Coeff(1)*sin(Coeff(2)*t);
	end

end
% EOF