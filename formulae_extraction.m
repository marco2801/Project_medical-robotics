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

%% Trials

gamma = 4*pi/5;
lio = 16;
ww = 5.5;
t = (lio-ww)/(2*cos(pi/2-gamma/2));
s0 = 0;
l0 = 10;
ein = -4.92;

an = [1/4; 3/8; 1/2; 5/8];
CoIa = [ww/2 + (t-ein)*cos(pi/2-gamma/2)-s0;
        ein*sin(pi/2-gamma/2)-l0];
radius = sqrt(CoIa(1)^2 + CoIa(2)^2);
theta = 0:pi/100:pi*2;

Q_mat = zeros(2,2,4);
for i = 1:4
    Q_mat(:,:,i) = [cos(2*pi*an(i)), -sin(2*pi*an(i));
                    sin(2*pi*an(i)), cos(2*pi*an(i))];
    Q_coord(:,i) = Q_mat(:,:,i)*CoIa + [s0;l0];
end

figure()
plot(s0,l0,'ro','LineWidth',2)
hold on
for i=1:4
    plot(Q_coord(1,i),Q_coord(2,i),'*','linewidth',2)
    grid on; hold on;
    legend('center','1','2','3','4')
end
plot(s0 + CoIa(1),l0 + CoIa(2),'bo')
plot(radius*cos(theta)+s0,radius*sin(theta)+l0,'g','LineWidth',2);
axis equal
