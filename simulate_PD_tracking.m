% ------------------------------------------------------------------------------
% Closed-Loop PD Control Simulation for a 2-DOF Planar Manipulator
% ------------------------------------------------------------------------------
% This script simulates a 2-link planar elbow manipulator controlled by a 
% proportional-derivative (PD) controller with gravity compensation.
%
% The manipulator is modeled using Peter Corke's Robotics Toolbox 
% with dynamic parameters and link definitions consistent with standard DH models.
%
% Workflow:
%   - Define the robot using DH parameters with mass, inertia, and CoM.
%   - Generate a joint-space trajectory using jtraj() from initial to final pose.
%   - Apply a PD controller: u = -Kp*e - Kd*edot + G(q)
%   - Simulate dynamics using Euler integration: q̈ = D⁻¹(u - C*dq - G)
%   - Log joint states and control torques over time.
%   - Animate the resulting motion and plot tracking/error performance.
%
%
% Next Step:Use PSO to find an optimal path while considering:
%
%   - Physical joint limits of the robot
%   - Obstacle avoidance in the workspace
%   - Minimum energy or torque usage
%   - Shortest or fastest path to the goal
%
%% Mohammad Mahdi Khaligh
%% ------------------------------------------------------------------------------

function simulate_PD_tracking()
    % Load robot
    robot = define_planar_robot();

    % Desired joint trajectory
    q0 = [pi/4, 0];
    qf = [pi/3, pi/3];
    [q_d, dq_d, ddq_d, t] = generate_trajectory(q0, qf, 10, 200);

    % Controller gains
    kp = 10;
    kd = 5;
    Kp = kp * eye(2);
    Kd = kd * eye(2);

    % Initial state
    q = q0;
    dq = [0 0];

    % Logging
    Q = []; DQ = []; U = [];

    for i = 1:length(t)
        e = q - q_d(i, :);
        edot = dq - dq_d(i, :);
        
        % PD + gravity compensation
        tau = -Kp * e' - Kd * edot' + robot.gravload(q)';
        
        % Simulate forward dynamics: qdd = D⁻¹(tau - C*dq - G)
        D = robot.inertia(q);
        C = robot.coriolis(q, dq);
        G = robot.gravload(q);
        qdd = D \ (tau - C*dq' - G');

        % Integrate (Euler)
        dt = t(2) - t(1);
        dq = dq + qdd' * dt;
        q = q + dq * dt;

        % Log
        Q(i,:) = q;
        DQ(i,:) = dq;
        U(i,:) = tau';
    end
    define_planar_robot()

    % Plot results
    figure;
    subplot(2,1,1);
    plot(t, Q(:,1), 'b', t, q_d(:,1), 'r--'); hold on;
    plot(t, Q(:,2), 'g', t, q_d(:,2), 'k--');
    title('Joint Positions');
    legend('q1', 'q1\_desired', 'q2', 'q2\_desired');

    subplot(2,1,2);
    plot(t, U(:,1), 'b', t, U(:,2), 'r');
    title('Control Inputs');
    legend('u1', 'u2');
    % Animate simulation
    figure;
    robot.plot(Q, 'fps', 1/(t(2)-t(1)));



end
