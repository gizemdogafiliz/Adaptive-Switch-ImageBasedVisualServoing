function AdaptiveSwitchIBVS()
    % AdaptiveSwitchIBVS
    % Implementation of Adaptive Switch Image-Based Visual Servoing (IBVS)
    % for a simulated 6-DOF robot in MATLAB.
    %
    % This script includes:
    % - Simplified robot dynamics
    % - Image formation (pinhole camera model)
    % - Adaptive switch control law with camera parameter estimation
    % - Simulation loop
    % - Plotting of results

    clc; clear; close all;

    %% ================== Simulation Parameters ==================
    % Simulation time
    tStart = 0;          % Start time (seconds)
    tEnd   = 30;         % End time (seconds)
    dt     = 0.001;      % Time step (seconds)
    tArray = tStart:dt:tEnd;
    N      = length(tArray);

    % Gains for Adaptive Switch IBVS
    Kv1 = 3*eye(6);        % Damping gain stage 1
    Kv2 = 3*eye(6);        % Damping gain stage 2
    Kv3 = 3*eye(6);        % Damping gain stage 3
    Kp1 = 9e-5 * eye(8);   % Feature gain stage 1 (Corrected to 8x8)
    Kp2 = 9e-4 * eye(8);   % Feature gain stage 2 (Corrected to 8x8)
    Kp3 = 9e-3 * eye(8);   % Feature gain stage 3 (Corrected to 8x8)

    % Adaptive camera parameter estimation gain
    Ka = 0.1 * eye(10);    % Camera-adaptation gain for stage 1 & 2

    % Switch thresholds for angle alpha (in radians)
    alpha0 = deg2rad(10.3);   % Stage 1 to 2
    alpha1 = deg2rad(8.5);    % Stage 2 to 3

    % Robot Parameters (Simplified)
    robotParam.Mdiag = [3; 3; 2; 2; 1; 1];  % Inertia matrix diagonal elements
    robotParam.C     = zeros(6);             % Coriolis matrix (simplified as zero)
    robotParam.G     = zeros(6,1);           % Gravity vector (simplified as zero)

    % Camera Intrinsic Parameters (True Values)
    trueCam.f    = 0.004;       % Focal length (meters)
    trueCam.beta = 110000;      % Scale factor (pixel/m)
    trueCam.cu   = 120;         % Principal point x (pixels)
    trueCam.cv   = 187;         % Principal point y (pixels)
    trueCam.Z    = 0.5;         % Depth (meters)

    % Initial Estimated Camera Parameters
    estCam.f    = 0.002;       % Initial estimate of focal length
    estCam.beta = 100000;      % Initial estimate of scale factor
    estCam.cu   = 110;         % Initial estimate of principal point x
    estCam.cv   = 180;         % Initial estimate of principal point y
    estCam.Z    = trueCam.Z;   % Assume known depth

    % Initialize Camera Parameter Vector (theta)
    theta0 = computeThetaFromCamParams(estCam);

    % Desired and Initial Feature Points (Test 1 as example)
    % Feature points are in [x; y] format for each of the 4 features
    sD = [232  82;
           272  82;
           272 119;
           233 119]';
    sDvec = sD(:);    % 8x1 vector

    s0 = [146 107;
          172  76;
          200  99;
          175 131]';
    s0vec = s0(:);    % 8x1 vector

    % Initial Angle alpha (between desired and actual features)
    alpha_init = computeAngleBetweenFeatures(s0vec, sDvec);

    % Initial Robot State
    q0    = zeros(6,1);         % Initial joint positions
    qdot0 = zeros(6,1);         % Initial joint velocities

    % State Vector: [q; qdot; s; theta]
    x0 = [q0; qdot0; s0vec; theta0];

    % Pre-allocate for logging
    xAll = zeros(length(x0), N);
    xAll(:,1) = x0;

    % Norm of Feature Error (NFE)
    NFE = zeros(1, N);

    %% ================== Simulation Loop ==================
    for i = 1:N-1
        t = tArray(i);
        x = xAll(:,i);

        % Compute Control Input (Torque) and State Derivative
        [xdot, currentNFE] = adaptiveSwitchControl(t, x, sDvec, robotParam, trueCam, ...
                                                  alpha0, alpha1, Kp1, Kp2, Kp3, ...
                                                  Kv1, Kv2, Kv3, Ka);
        % Update State using Euler Integration
        xAll(:,i+1) = x + xdot * dt;

        % Log NFE
        NFE(i) = currentNFE;

        % Check Stopping Condition
        if NFE(i) < 0.005
            xAll(:,i+1:end) = xAll(:,i);
            NFE(i+1:end) = NFE(i);
            break;
        end
    end

    % Trim unused data
    xAll = xAll(:,1:i+1);
    tArray = tArray(1:i+1);
    N = length(tArray);
    NFE = NFE(1:i+1);

    %% ================== Plotting Results ==================
    % Extract States
    qData    = xAll(1:6, :);        % Joint positions
    qdotData = xAll(7:12, :);       % Joint velocities
    sData    = xAll(13:20, :);      % Feature points
    thetaData= xAll(21:end, :);     % Camera parameter estimates

    % Plot Feature Trajectories in Image Plane
    figure;
    hold on; grid on; box on;
    colors = ['r','g','b','m'];
    for f = 1:4
        plot(sData(2*f-1, :), sData(2*f, :), colors(f), 'LineWidth', 2);
    end
    plot(sD(1,:), sD(2,:), 'ko','MarkerSize',10,'LineWidth',2);
    xlabel('x (pixels)'); ylabel('y (pixels)');
    title('Adaptive Switch IBVS - Feature Trajectories');
    legend('Feature1','Feature2','Feature3','Feature4','Desired','Location','Best');

    % Plot Norm of Feature Error Over Time
    figure;
    plot(tArray, NFE, 'b', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Norm of Feature Error (pixels)');
    title('Adaptive Switch IBVS - Norm of Feature Error');
    grid on; box on;

    % Plot Joint Angles Over Time
    figure;
    hold on; grid on; box on;
    for f = 1:6
        plot(tArray, qData(f,:), 'LineWidth', 1.5);
    end
    xlabel('Time (s)'); ylabel('Joint Angles (rad)');
    title('Adaptive Switch IBVS - Joint Angles');
    legend('q1','q2','q3','q4','q5','q6','Location','Best');

    %% ================== Helper Functions ==================

    function theta = computeThetaFromCamParams(cam)
        % Compute the theta vector from camera parameters (eq. 24)
        % theta = [1; 1/(f^2*beta^2); cu/(f^2*beta^2); cv/(f^2*beta^2);
        %          (cu*cv)/(f^2*beta^2); (cu^2)/(f^2*beta^2);
        %          1/(f*beta); cv/(f*beta); cu/(f*beta);
        %          (cv^2)/(f^2*beta^2)]
        f    = cam.f;
        b    = cam.beta;
        cu   = cam.cu;
        cv   = cam.cv;
        theta = [
            1;
            1/(f^2 * b^2);
            cu/(f^2 * b^2);
            cv/(f^2 * b^2);
            (cu*cv)/(f^2 * b^2);
            (cu^2)/(f^2 * b^2);
            1/(f*b);
            cv/(f*b);
            cu/(f*b);
            (cv^2)/(f^2 * b^2)
        ];
    end

    function alpha = computeAngleBetweenFeatures(sActual, sDesired)
        % Compute the average angle between actual and desired feature vectors
        % sActual and sDesired are 8x1 vectors [x1; y1; x2; y2; ... ; x4; y4]
        nF = 4; % Number of features
        sumAngle = 0;
        for i = 1:nF
            vA = [sActual(2*i-1); sActual(2*i)];
            vD = [sDesired(2*i-1); sDesired(2*i)];
            normA = norm(vA);
            normD = norm(vD);
            if normA < 1e-6 || normD < 1e-6
                angle_i = 0;
            else
                angle_i = acos( dot(vA, vD)/(normA*normD) );
            end
            sumAngle = sumAngle + angle_i;
        end
        alpha = sumAngle / nF;
    end

    function [xdot, NFE] = adaptiveSwitchControl(t, x, sDvec, robotParam, trueCam, ...
                                                alpha0, alpha1, Kp1, Kp2, Kp3, ...
                                                Kv1, Kv2, Kv3, Ka)
        % Adaptive Switch Control Law
        %
        % Inputs:
        %   t        - Current time
        %   x        - Current state vector [q; qdot; s; theta]
        %   sDvec    - Desired feature vector (8x1)
        %   robotParam - Struct containing robot dynamics parameters
        %   trueCam  - Struct containing true camera parameters
        %   alpha0, alpha1 - Threshold angles for switching
        %   Kp1, Kp2, Kp3 - Feature gains for stages 1, 2, 3
        %   Kv1, Kv2, Kv3 - Damping gains for stages 1, 2, 3
        %   Ka       - Adaptation gain matrix
        %
        % Outputs:
        %   xdot     - State derivative
        %   NFE      - Current norm of feature error

        % Unpack State
        q       = x(1:6);
        qdot    = x(7:12);
        s       = x(13:20);
        theta   = x(21:end);  % Camera parameter estimates

        % Compute Feature Error
        sErr = s - sDvec;

        % Compute Current Angle alpha
        alpha = computeAngleBetweenFeatures(s, sDvec);

        % Determine Control Stage
        if abs(alpha) >= alpha0
            stage = 1; % Rotation-only
        elseif abs(alpha) >= alpha1
            stage = 2; % Translation-only
        else
            stage = 3; % Full IBVS
        end

        % Compute Image Jacobian based on stage
        if stage == 1
            [Jimg, JR] = computeImgJacobianRotationOnly(q, s, theta, trueCam);
            Kp = Kp1;
            Kv = Kv1;
        elseif stage == 2
            [Jimg, JR] = computeImgJacobianTranslationOnly(q, s, theta, trueCam);
            Kp = Kp2;
            Kv = Kv2;
        else
            [Jimg, JR] = computeImgJacobianFull(q, s, theta, trueCam);
            Kp = Kp3;
            Kv = Kv3;
        end

        % Compute Control Velocity Vc using Pseudo-Inverse of Jimg
        % Vc = - pinv(Jimg) * (Kp * sErr);
        % Ensure dimensions: Jimg (8x6), pinv(Jimg) (6x8), Kp (8x8), sErr (8x1)
        % So Kp * sErr is (8x1), pinv(Jimg) * (8x1) = (6x1)
        Vc = - pinv(Jimg) * (Kp * sErr);  % Vc is 6x1

        % Compute Control Torque tau
        % Here, we simplify tau as Vc (Replace with actual dynamics-based torque if needed)
        tau = Vc;

        % Compute State Derivative (Simplified Dynamics)
        % qddot = M^-1 * (tau - C*qdot - G)
        M = diag(robotParam.Mdiag);
        C = robotParam.C;
        G = robotParam.G;
        qddot = M \ (tau - C * qdot - G);

        % Compute sdot = Jimg * qdot
        sdot = Jimg * qdot;

        % Update Camera Parameters if in Stage 1 or 2
        if stage == 1
            Y = computeRegressionMatrixRotation(q, s, theta, trueCam);
            theta_dot = Ka \ (Y' * (Kp * sErr));
        elseif stage == 2
            Y = computeRegressionMatrixTranslation(q, s, theta, trueCam);
            theta_dot = Ka \ (Y' * (Kp * sErr));
        else
            theta_dot = zeros(10,1); % No adaptation in Stage 3
        end

        % Compute Norm of Feature Error
        NFE = sum(sqrt(sErr(1:2:end).^2 + sErr(2:2:end).^2));

        % Assemble State Derivative
        xdot = [qdot;
                qddot;
                sdot;
                theta_dot];
    end

    function [Jimg, JR] = computeImgJacobianRotationOnly(q, s, theta, cam)
        % Compute the image Jacobian for rotation-only control (Stage 1)
        % Placeholder implementation: needs to be replaced with actual computation
        % based on eq. (17) from the paper.

        % For demonstration, assume JR (robot rotational Jacobian) is identity
        JR = eye(6);

        % Compute partial image Jacobian Jr(t)
        % Placeholder: Assign scaled identity to simulate rotation Jacobian
        Jimg = 0.01 * eye(8,6); % 8x6 Jacobian for 4 features (2 per feature)
    end

    function [Jimg, JR] = computeImgJacobianTranslationOnly(q, s, theta, cam)
        % Compute the image Jacobian for translation-only control (Stage 2)
        % Placeholder implementation: needs to be replaced with actual computation
        % based on eq. (18) from the paper.

        % For demonstration, assume JR (robot translational Jacobian) is identity
        JR = eye(6);

        % Compute partial image Jacobian Jt(t)
        % Placeholder: Assign scaled identity to simulate translation Jacobian
        Jimg = 0.02 * eye(8,6); % 8x6 Jacobian for 4 features (2 per feature)
    end

    function [Jimg, JR] = computeImgJacobianFull(q, s, theta, cam)
        % Compute the full image Jacobian for full IBVS control (Stage 3)
        % Placeholder implementation: needs to be replaced with actual computation
        % based on eq. (19) from the paper.

        % For demonstration, assume JR (robot Jacobian) is identity
        JR = eye(6);

        % Compute full image Jacobian Jimg(t)
        % Placeholder: Assign scaled identity to simulate full Jacobian
        Jimg = 0.03 * eye(8,6); % 8x6 Jacobian for 4 features (2 per feature)
    end

    function Y = computeRegressionMatrixRotation(q, s, theta, cam)
        % Compute the regression matrix Y1 for rotation-only control (Stage 1)
        % Based on eq. (21) and (23) from the paper.
        % Placeholder implementation: needs to be replaced with actual computation.

        Y = 0.01 * rand(8,10); % 8x10 regression matrix (Placeholder)
    end

    function Y = computeRegressionMatrixTranslation(q, s, theta, cam)
        % Compute the regression matrix Y2 for translation-only control (Stage 2)
        % Based on eq. (21) and (23) from the paper.
        % Placeholder implementation: needs to be replaced with actual computation.

        Y = 0.02 * rand(8,10); % 8x10 regression matrix (Placeholder)
    end

end
