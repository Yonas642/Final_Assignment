function C = constraints(mbs, q, t)
%CONSTRAINTS Return constraints vector for the system
C = zeros(mbs.nc, 1);
c_idx = 0;

for rj = mbs.joints.revolute
    q1 = q(body_idx(rj.body1));
    q2 = q(body_idx(rj.body2));
    r1 = q1(1:2);
    r2 = q2(1:2);
    A1 = rot(q1(3));
    A2 = rot(q2(3));
    C(c_idx + (1:2)) = r1 + A1 * rj.s1 - r2 - A2 * rj.s2;
    c_idx = c_idx + 2;
end

for tj = mbs.joints.translational
    q1 = q(body_idx(tj.body1));
    q2 = q(body_idx(tj.body2));
    phi1 = q1(3);
    phi2 = q2(3);
    A1 = rot(q1(3));
    r1 = q1(1:2);
    q_s2 = tj.s2;
    q_s3 = r1 + A1 * q_s2;

    n_i = [-(q1(2) - q_s3(2)); q1(1) - q_s3(1)];
    d = [q2(1) - q1(1); q2(2) - q1(2)];

    C(c_idx + 1) = n_i' * d;
    C(c_idx + 2) = phi1 - phi2;
    c_idx = c_idx + 2;
end

for sj = mbs.joints.simple
    qb = q(body_idx(sj.body));
    C(c_idx + 1) = qb(sj.coord) - sj.c0;
    c_idx = c_idx + 1;
end

for dj = mbs.joints.driving
    qb = q(body_idx(dj.body));
    C(c_idx + 1) = qb(dj.coord) - dj.cfun(t);
    c_idx = c_idx + 1;
end

