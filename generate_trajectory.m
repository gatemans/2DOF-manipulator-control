function [q, dq, ddq, t] = generate_trajectory(q0, qf, T, N)
    t = linspace(0, T, N);
    [q, dq, ddq] = jtraj(q0, qf, t);
end
