function angles = inverseKinematicsCalculator(H)

    % DH-parameters for the robot
    alpha = [pi/2  0  pi/2 -pi/2 pi/2 0];  
    a     = [0  0.32 0.0735  0  0 0];
    d     = [0.23682  0 0  0.2507 0 0.057];

   % alpha = [-pi/2  -pi -pi/2 pi/2 -pi/2 pi];  
   % a     = [0  0.32 0.0735  0  0 0];
   % d     = [0.2357  0 0  -0.25 0 -0.176];

    % Extract end-effector position
    x = H(1,4);
    y = H(2,4);
    z = H(3,4);

    R06 = H(1:3, 1:3); % Rotation matrix of the end-effector

    % Compute wrist center position
    x_w = x - d(6) * R06(1,3);
    y_w = y - d(6) * R06(2,3);
    z_w = z - d(6) * R06(3,3);

    % Compute theta1
    theta1 = atan2(y_w, x_w);

    % Compute r and s for theta2 and theta3
    r = sqrt(x_w^2 + y_w^2);
    s = z_w - d(1);

    % Adjust a3 due to offset
    a3 = sqrt(a(3)^2 + d(4)^2);
    theta3_offset = atan2(-d(4), a(3));

    % Compute theta3
    D = (r^2 + s^2 - a(2)^2 - a3^2) / (2 * a(2) * a3);
    theta3_no_offset = atan2(-sqrt(1 - D^2), D);
    theta3 = theta3_no_offset - theta3_offset;

    % Compute theta2
    theta2 = atan2(s, r) - atan2(a3 * sin(theta3_no_offset), a(2) + a3 * cos(theta3_no_offset));

    % Compute rotation matrix R_03
    A1 = [cos(theta1), -sin(theta1)*cos(alpha(1)), sin(theta1)*sin(alpha(1)), 0;
          sin(theta1), cos(theta1)*cos(alpha(1)), -cos(theta1)*sin(alpha(1)), 0;
          0, sin(alpha(1)), cos(alpha(1)), d(1);
          0, 0, 0, 1];

    A2 = [cos(theta2), -sin(theta2)*cos(alpha(2)), sin(theta2)*sin(alpha(2)), a(2)*cos(theta2);
          sin(theta2), cos(theta2)*cos(alpha(2)), -cos(theta2)*sin(alpha(2)), a(2)*sin(theta2);
          0, sin(alpha(2)), cos(alpha(2)), d(2);
          0, 0, 0, 1];

    A3 = [cos(theta3), -sin(theta3)*cos(alpha(3)), sin(theta3)*sin(alpha(3)), a(3)*cos(theta3);
          sin(theta3), cos(theta3)*cos(alpha(3)), -cos(theta3)*sin(alpha(3)), a(3)*sin(theta3);
          0, sin(alpha(3)), cos(alpha(3)), d(3);
          0, 0, 0, 1];

    T_03 = A1 * A2 * A3;
    R_03 = T_03(1:3, 1:3);

    % Compute R36
    R36 = R_03' * R06;

    % Compute theta4, theta5, theta6
    theta4 = atan2(R36(2,3), R36(1,3));
    theta5 = atan2(sqrt(R36(1,3)^2 + R36(2,3)^2), R36(3,3));
    theta6 = atan2(R36(3,2), -R36(3,1));

    % Return joint angles
    angles = [theta1, theta2, theta3, theta4, theta5, theta6];
end