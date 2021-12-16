 function [t, u] = ode45(odefun, tspan, U_0)
    dt = 0.01;
    T = 1;
    N_t = floor(T/dt);
    u = zeros(N_t+1, length(U_0));
    t = linspace(0, N_t*dt, length(u));
    u(1, :) = U_0';
    for n = 1:N_t
        u(n+1, :) = u(n, :) + dt.*odefun(t(n), u(n, :)')';
    end
end
    





