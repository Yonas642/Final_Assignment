function G = acceleration1(mbs, q, qd, t)
% Return LHS of acceleration equation
G = zeros(mbs.nc, 1);
G_idx = 0;

for rj = mbs.joints.revolute
    q1 = q(body_idx(rj.body1));
    q2 = q(body_idx(rj.body2));
    qd1 = qd(body_idx(rj.body1));
    qd2 = qd(body_idx(rj.body2));
    qd1 = qd1(3);
    qd2 = qd2(3);
    A1 = rot(q1(3));
    A2 = rot(q2(3));
    G(G_idx + (1:2)) = A1 * rj.s1 * qd1 .* qd1  -  A2 * rj.s2 * qd2 .* qd2;
    G_idx = G_idx + 2;
end

for tj = mbs.joints.translational
    q1 = q(body_idx(tj.body1));
    q2 = q(body_idx(tj.body2));
    qd1 = qd(body_idx(tj.body1));
    qd2 = qd(body_idx(tj.body2));
    
    q0_s2 = tj.s2;
    phi1 = q1(3);
    A1 = rot(phi1);
    r1 = q1(1:2);
    q_s2 = r1 + A1 * q0_s2;
    
    G(G_idx + 1) = - 2 * ((q1(1) - q_s2(1)) * (qd1(1) - qd2(1))... % elements are took from table 4.3 in Parviz E. Nikravesh book
                   + (q1(2) - q_s2(2)) * (qd1(2) - qd2(2))) * qd1(3)...
                   - ((q1(1) - q_s2(1)) * (q1(2) - q2(2))...
                   - (q1(2) - q_s2(2)) * (q1(1) - q2(1))) * qd2(3) * qd2(3); 
    G(G_idx + 2) = 0;
    G_idx = G_idx + 2;
end

for sj = mbs.joints.simple
    qb = qd(body_idx(sj.body));
    G(G_idx + 1) = qb(sj.coord) - sj.c0;
    G_idx = G_idx + 1;
end

for dj = mbs.joints.driving
    G(G_idx + 1) = - dj.cfun_dtt(t);
    G_idx = G_idx + 1;
end

