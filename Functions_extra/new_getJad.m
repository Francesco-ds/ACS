function Jad = new_getJad(q,xe,xTilde,Td)

Re = eul2rotm(xe(4:6)', 'ZYZ');
Rd = eul2rotm(xd(4:6)', 'ZYZ');


Te = zeros(4,4);
Te(1:3, 1:3) = Re;
Te(1:3, 4) = xe(1:3);
Te(4,4) = 1;

Td = zeros(4,4);
Td(1:3, 1:3) = Rd;
Td(1:3, 4) = xd(1:3);
Td(4,4) = 1;

Tde = inv(Td)*Te;
phide  = rotm2eul(Tde(1:3, 1:3), 'ZYZ');

Rd = eul2rotm(xd(4:6)', 'ZYZ');

% Compute the analytical jacobian respect to the desired frame 
Jad = pinv(getTransformAnalytical(phide))*[Rd', zeros(3,3); zeros(3,3), Rd']*getJacobian(q);
Jad = pinv(getTransformAnalytical(-xtilde(4:6)))*[Rd', zeros(3,3); zeros(3,3), Rd']*getJacobian(q);


