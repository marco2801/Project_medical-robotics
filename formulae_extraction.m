clear all
clc

syms alpha1 alpha2 ein eout lio gamma s0 l0 dc
syms IdIa_x IdIa_y OdOa_x OdOa_y CoIa_x CoIa_y CoOa_x CoOa_y

% Vettori unitari
IdIa = [IdIa_x; IdIa_y];
OdOa = [OdOa_x; OdOa_y];
CoIa = [CoIa_x; CoIa_y];
CoOa = [CoOa_x; CoOa_y];

% equations (9)
eq1 = lio/2 - s0 + ein * IdIa(1) == (dc/2) * CoIa(1);
eq2 = -l0 + ein * IdIa(2) == (dc/2) * CoIa(2);
% equations (10)
eq3 = -lio/2 - s0 + eout * OdOa(1) == (dc/2) * CoOa(1);
eq4 = -l0 + eout * OdOa(2) == (dc/2) * CoOa(2);

% equations (11) - collinearity
eq5 = det([IdIa, [cos(pi - (pi - gamma)/2); sin(pi - (pi - gamma)/2)]]) == 0;
eq6 = det([OdOa, [cos((pi - gamma)/2); sin((pi - gamma)/2)]]) == 0;

% equations (12)
eq7 = dot(CoIa, [cos(pi - (pi - gamma)/2); sin(pi - (pi - gamma)/2)]) == -cos(alpha2);
eq8 = dot(CoOa, [cos((pi - gamma)/2); sin((pi - gamma)/2)]) == -cos(alpha1);

% solve
vars = [alpha1, alpha2, ein, eout, IdIa_x, IdIa_y, OdOa_x, OdOa_y, CoIa_x, CoIa_y, CoOa_x, CoOa_y];
sol = solve([eq1; eq2; eq3; eq4; eq5; eq6; eq7; eq8], vars, 'ReturnConditions', true);

disp('Soluzioni simboliche:')
disp(sol)
