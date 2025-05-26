function robot = define_planar_robot()
    % Define the 2-DOF planar manipulator using DH parameters
    L1 = Link('d', 0, 'a', 0.5, 'alpha', 0, 'm', 1, 'r', [0.25 0 0], 'I', [0 0 0.25], 'G', 0, 'Jm', 0);
    L2 = Link('d', 0, 'a', 0.5, 'alpha', 0, 'm', 1, 'r', [0.25 0 0], 'I', [0 0 0.25], 'G', 0, 'Jm', 0);
    robot = SerialLink([L1 L2], 'name', 'Planar2DOF');
end
