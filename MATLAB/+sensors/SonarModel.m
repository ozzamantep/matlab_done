classdef SonarModel
% SONARMODEL Forward-looking 3D acoustic sonar simulation with occlusion and noise.
%   Generates SonarPing detections (range, azimuth, elevation, intensity, label).

    properties
        MaxRange = 45.0;          % meters
        HorizontalFOV = 110.0;    % degrees (+/- 55 deg)
        VerticalFOV = 50.0;       % degrees (+/- 25 deg)
        RangeNoiseStd = 0.35;     % meters
        AngleNoiseStd = 1.2;      % degrees
        DetectionProb = 0.92;     % probability of detecting target in FOV
        NumRaysAz = 15;           % horizontal obstacle sweep rays
    end

    methods
        function scan = sense(obj, seeker_eta, hider_eta, arena)
            p_s = seeker_eta(1:3);
            p_h = hider_eta(1:3);
            psi = seeker_eta(6);
            theta = seeker_eta(5);

            % Rotation from world to seeker body (approx yaw+pitch)
            R_wb = eul2rotm_yaw_pitch(psi, theta);

            scan.target_detected = false;
            scan.target_meas_world = [NaN; NaN; NaN];
            scan.target_range = NaN;
            scan.target_az = NaN;
            scan.target_el = NaN;
            scan.target_intensity = 0.0;
            scan.obstacle_points = zeros(3, 0);

            % 1) Check Hider detection
            dp_world = p_h - p_s;
            dist = norm(dp_world);
            if dist <= obj.MaxRange && dist > 0.2
                dp_body = R_wb' * dp_world;
                az = rad2deg(atan2(dp_body(2), dp_body(1)));
                el = rad2deg(atan2(dp_body(3), hypot(dp_body(1), dp_body(2))));

                in_fov = abs(az) <= obj.HorizontalFOV/2 && abs(el) <= obj.VerticalFOV/2;
                if in_fov && ~arena.isLineOfSightOccluded(p_s, p_h)
                    if rand() <= obj.DetectionProb
                        r_noisy  = max(0.3, dist + obj.RangeNoiseStd * randn());
                        az_noisy = az + obj.AngleNoiseStd * randn();
                        el_noisy = el + obj.AngleNoiseStd * randn();

                        % Convert noisy spherical back to body then world
                        az_r = deg2rad(az_noisy);
                        el_r = deg2rad(el_noisy);
                        dp_noisy_body = r_noisy * [cos(el_r)*cos(az_r);
                                                   cos(el_r)*sin(az_r);
                                                   sin(el_r)];
                        scan.target_detected = true;
                        scan.target_meas_world = p_s + R_wb * dp_noisy_body;
                        scan.target_range = r_noisy;
                        scan.target_az = az_noisy;
                        scan.target_el = el_noisy;
                        scan.target_intensity = max(0.1, min(1.0, 1.0 - dist / (obj.MaxRange * 1.1)));
                    end
                end
            end

            % 2) Obstacle sweep rays (for reactive obstacle avoidance)
            az_angles = linspace(-obj.HorizontalFOV/2, obj.HorizontalFOV/2, obj.NumRaysAz);
            obs_pts = [];
            for a = az_angles
                dir_body = [cosd(a); sind(a); 0];
                dir_world = R_wb * dir_body;
                for r = 1.5:1.5:18.0
                    pt = p_s + r * dir_world;
                    [hit, ~, ~] = arena.checkCollision(pt, 0.3);
                    if hit
                        obs_pts(:, end+1) = pt; %#ok<AGROW>
                        break;
                    end
                end
            end
            scan.obstacle_points = obs_pts;
        end
    end
end

function R = eul2rotm_yaw_pitch(psi, theta)
    cy = cos(psi); sy = sin(psi);
    ct = cos(theta); st = sin(theta);
    R = [cy*ct, -sy, cy*st;
         sy*ct,  cy, sy*st;
         -st,    0,  ct];
end
