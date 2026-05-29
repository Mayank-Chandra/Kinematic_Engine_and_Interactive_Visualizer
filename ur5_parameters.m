function robot = ur5_parameters()
    robot.name = 'UR5';
    % Matrix rows: [alpha, a, d, theta_offset]
    robot.dh_table = [
         1.570796327,  0.0,        0.089159,  0.0;
         0.0,         -0.425,      0.0,       0.0;
         0.0,         -0.39225,    0.0,       0.0;
         1.570796327,  0.0,        0.10915,   0.0;
        -1.570796327,  0.0,        0.09465,   0.0;
         0.0,          0.0,        0.0823,    0.0
    ];
end