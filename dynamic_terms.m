
function [D, C, G] = dynamic_terms(q, dq, robot)
    % Uses Peter Corke Robotics Toolbox model to get dynamics

    D = robot.inertia(q);
    C = robot.coriolis(q, dq);
    G = robot.gravload(q)';
end
