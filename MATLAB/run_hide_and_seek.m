function results = run_hide_and_seek(headless, seed)
% RUN_HIDE_AND_SEEK Main 3D Underwater Drone Hide-and-Seek Simulation (MATLAB).
%   Integrates 6-DOF Fossen physics, Sonar + Optical Camera fusion, 3D EKF,
%   CoverageGrid, 3D A* Path Planning, APF obstacle avoidance, and full State Machine.

    if nargin < 1, headless = false; end
    if nargin < 2, seed = 42; end
    rng(seed);

    rootDir = fileparts(mfilename('fullpath'));
    addpath(rootDir);

    dt = 0.05;
    maxTime = 120.0;
    headstartTime = 8.0;
    N = round(maxTime / dt);

    params   = auv.BlueROV2Params();
    arena    = sim.UnderwaterArena();
    sonar    = sensors.SonarModel();
    cam      = sensors.CameraDetector();
    imu_s    = sensors.DepthIMUDVL();
    tracker  = perception.EKFTargetTracker();
    covGrid  = perception.CoverageGrid();
    pid_s    = navigation.PIDController6DOF();
    pid_h    = navigation.PIDController6DOF();
    seekerAI = navigation.SeekerAI(arena);
    hiderAI  = navigation.HiderAI();

    seeker_eta = [30; 30; -10; 0; 0; -2.35];
    seeker_nu  = zeros(6, 1);
    hider_eta  = [-12 + 8*randn(); -10 + 8*randn(); -14; 0; 0; 0.5];
    hider_nu   = zeros(6, 1);

    t_log       = zeros(1, N);
    seeker_pos  = zeros(3, N);
    hider_pos   = zeros(3, N);
    ekf_pos     = nan(3, N);
    dist_log    = zeros(1, N);
    state_log   = strings(1, N);
    effort_s    = zeros(1, N);
    false_dets  = 0;

    winner = "HIDER";
    capture_time = NaN;
    steps_run = 0;

    if ~headless
        fig = figure('Name', 'Underwater Drone Hide and Seek - MathWorks Challenge', ...
                     'Color', [0.03, 0.10, 0.18], 'Position', [80, 60, 1200, 720]);
        ax = axes(fig, 'Color', [0.02, 0.14, 0.24], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
        hold(ax, 'on'); grid(ax, 'on');
        ax.GridColor = [0.2, 0.5, 0.7]; ax.GridAlpha = 0.35;
        view(ax, 42, 28);
        axis(ax, [arena.Bounds(1:4), arena.Bounds(5)-2, 1]);
        xlabel(ax, 'East X (m)'); ylabel(ax, 'North Y (m)'); zlabel(ax, 'Depth Z (m)');

        [GX, GY] = meshgrid(-60:5:60, -60:5:60);
        GZ = arena.getSeabedHeight(GX, GY);
        surf(ax, GX, GY, GZ, 'FaceColor', [0.16, 0.22, 0.25], 'EdgeColor', 'none', 'FaceAlpha', 0.85);

        for i = 1:numel(arena.Obstacles)
            o = arena.Obstacles(i);
            if strcmp(o.type, 'cylinder')
                [cx, cy, cz] = cylinder(o.radius, 20);
                cz = cz * o.height + o.pos(3);
                surf(ax, cx + o.pos(1), cy + o.pos(2), cz, 'FaceColor', o.color, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
            else
                [sx, sy, sz] = sphere(16);
                surf(ax, sx*o.radius + o.pos(1), sy*o.radius + o.pos(2), sz*o.radius + o.pos(3), ...
                     'FaceColor', o.color, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
            end
        end

        hSeeker = plot3(ax, seeker_eta(1), seeker_eta(2), seeker_eta(3), 'o', ...
                        'MarkerSize', 10, 'MarkerFaceColor', [1.0, 0.85, 0.1], 'MarkerEdgeColor', 'k');
        hHider  = plot3(ax, hider_eta(1), hider_eta(2), hider_eta(3), 's', ...
                        'MarkerSize', 10, 'MarkerFaceColor', [0.15, 0.65, 1.0], 'MarkerEdgeColor', 'w');
        hEKF    = plot3(ax, nan, nan, nan, 'x', 'Color', [1.0, 0.3, 0.3], 'MarkerSize', 10, 'LineWidth', 2);
        hTrailS = plot3(ax, seeker_eta(1), seeker_eta(2), seeker_eta(3), '-', 'Color', [1.0, 0.85, 0.1], 'LineWidth', 1.5);
        hTrailH = plot3(ax, hider_eta(1), hider_eta(2), hider_eta(3), '-', 'Color', [0.15, 0.65, 1.0], 'LineWidth', 1.5);
        legend(ax, [hSeeker, hHider, hEKF], {'Seeker AUV (Yellow)', 'Hider AUV (Blue)', 'EKF Estimate'}, ...
               'TextColor', 'w', 'Color', [0.05, 0.15, 0.25], 'Location', 'northeast');
        hTitle = title(ax, 'Initializing...', 'Color', 'w', 'FontSize', 11);
    end

    for k = 1:N
        t = (k - 1) * dt;
        steps_run = k;
        t_log(k) = t;

        if t < headstartTime, phase = "HIDING"; else, phase = "SEEKING"; end

        % 1. Sensors (Sonar + Optical Camera + Depth/IMU/DVL)
        scan = sonar.sense(seeker_eta, hider_eta, arena);
        cam_det = cam.detect(seeker_eta, hider_eta, arena);
        imu_meas = imu_s.read(seeker_eta, seeker_nu); %#ok<NASGU>

        if cam_det.detected && cam_det.is_false_positive
            false_dets = false_dets + 1;
        end

        % 2. Perception (Coverage Grid + Multi-Sensor EKF Fusion)
        covGrid = covGrid.markVisited(seeker_eta(1:3), 18.0);
        tracker = tracker.predict(dt);
        if scan.target_detected
            tracker = tracker.update(scan.target_meas_world);
        end
        if cam_det.detected && ~cam_det.is_false_positive
            tracker = tracker.update(cam_det.world_pos);
        end

        % 3. Guidance & Control
        dist_true = norm(seeker_eta(1:3) - hider_eta(1:3));
        [v_des_s, yaw_des_s, seekerAI] = seekerAI.step(seeker_eta, tracker, scan, covGrid, phase, dist_true);
        [tau_s, pid_s] = pid_s.computeControl(seeker_eta, seeker_nu, v_des_s, yaw_des_s, dt);

        [v_des_h, yaw_des_h, hiderAI] = hiderAI.step(hider_eta, seeker_eta, arena, phase, t);
        [tau_h, pid_h] = pid_h.computeControl(hider_eta, hider_nu, v_des_h, yaw_des_h, dt);

        % 4. Ocean Currents & 6-DOF Fossen Physics
        curr_s = arena.getOceanCurrent(seeker_eta(1:3), t);
        curr_h = arena.getOceanCurrent(hider_eta(1:3), t);

        [seeker_eta, seeker_nu] = auv.FossenDynamics(seeker_eta, seeker_nu, tau_s, curr_s, params, dt, arena.Bounds);
        [hider_eta,  hider_nu]  = auv.FossenDynamics(hider_eta,  hider_nu,  tau_h, curr_h, params, dt, arena.Bounds);

        [seeker_eta(1:3), seeker_nu(1:3)] = arena.resolveCollision(seeker_eta(1:3), seeker_nu(1:3));
        [hider_eta(1:3),  hider_nu(1:3)]  = arena.resolveCollision(hider_eta(1:3),  hider_nu(1:3));

        % 5. Logging & Capture Check
        dist = norm(seeker_eta(1:3) - hider_eta(1:3));
        seeker_pos(:, k) = seeker_eta(1:3);
        hider_pos(:, k)  = hider_eta(1:3);
        if tracker.Initialized, ekf_pos(:, k) = tracker.x(1:3); end
        dist_log(k)  = dist;
        state_log(k) = seekerAI.State;
        effort_s(k)  = norm(tau_s(1:3));

        if ~headless && mod(k, 4) == 0 && isvalid(fig)
            set(hSeeker, 'XData', seeker_eta(1), 'YData', seeker_eta(2), 'ZData', seeker_eta(3));
            set(hHider,  'XData', hider_eta(1),  'YData', hider_eta(2),  'ZData', hider_eta(3));
            set(hTrailS, 'XData', seeker_pos(1, 1:k), 'YData', seeker_pos(2, 1:k), 'ZData', seeker_pos(3, 1:k));
            set(hTrailH, 'XData', hider_pos(1, 1:k),  'YData', hider_pos(2, 1:k),  'ZData', hider_pos(3, 1:k));
            if tracker.Initialized
                set(hEKF, 'XData', tracker.x(1), 'YData', tracker.x(2), 'ZData', tracker.x(3));
            end
            set(hTitle, 'String', sprintf('Phase: %s | State: %s | t=%.1fs | Dist=%.2fm | Cov=%.0f%% | EKF Conf=%.0f%%', ...
                phase, seekerAI.State, t, dist, covGrid.getCoveragePercent(), tracker.Confidence*100));
            drawnow limitrate;
        end

        if strcmp(phase, "SEEKING") && dist <= arena.CaptureRadius
            winner = "SEEKER";
            capture_time = t;
            break;
        end
    end

    t_log      = t_log(1:steps_run);
    seeker_pos = seeker_pos(:, 1:steps_run);
    hider_pos  = hider_pos(:, 1:steps_run);
    ekf_pos    = ekf_pos(:, 1:steps_run);
    dist_log   = dist_log(1:steps_run);
    state_log  = state_log(1:steps_run);
    effort_s   = effort_s(1:steps_run);

    valid_ekf = ~isnan(ekf_pos(1, :));
    if any(valid_ekf)
        ekf_rmse = sqrt(mean(vecnorm(ekf_pos(:, valid_ekf) - hider_pos(:, valid_ekf)).^2));
    else
        ekf_rmse = NaN;
    end

    results.winner           = winner;
    results.capture_time     = capture_time;
    results.final_dist       = dist_log(end);
    results.ekf_rmse         = ekf_rmse;
    results.mean_effort      = mean(effort_s);
    results.coverage_pct     = covGrid.getCoveragePercent();
    results.false_detections = false_dets;
    results.t                = t_log;
    results.seeker_pos       = seeker_pos;
    results.hider_pos        = hider_pos;
    results.ekf_pos          = ekf_pos;
    results.dist             = dist_log;
    results.states           = state_log;

    fprintf('[Sim Seed %d] Winner: %s | Time: %.2fs | Dist: %.2fm | EKF RMSE: %.2fm | Cov: %.1f%% | FalseDet: %d\n', ...
        seed, winner, t_log(end), dist_log(end), ekf_rmse, results.coverage_pct, false_dets);
end
