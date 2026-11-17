classdef CameraDetector
% CAMERADETECTOR Simulates underwater optical camera + color/contour target detection.
%   Models turbidity attenuation, FOV limits, occlusion, and occasional false positives.

    properties
        MaxVisualRange = 22.0     % meters (turbidity-limited underwater visibility)
        HorizontalFOV = 80.0      % degrees
        VerticalFOV = 60.0        % degrees
        FalsePositiveProb = 0.02  % 2% chance of false-positive clutter
    end

    methods
        function det = detect(obj, seeker_eta, hider_eta, arena)
            p_s = seeker_eta(1:3);
            p_h = hider_eta(1:3);
            psi = seeker_eta(6);
            R_wb = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];

            det.detected = false;
            det.is_false_positive = false;
            det.world_pos = [NaN; NaN; NaN];
            det.confidence = 0.0;
            det.pixel_uv = [NaN, NaN];

            dp = p_h - p_s;
            dist = norm(dp);
            if dist <= obj.MaxVisualRange && dist > 0.3
                dp_body = R_wb' * dp;
                az = rad2deg(atan2(dp_body(2), dp_body(1)));
                el = rad2deg(atan2(dp_body(3), hypot(dp_body(1), dp_body(2))));

                if abs(az) <= obj.HorizontalFOV/2 && abs(el) <= obj.VerticalFOV/2
                    if ~arena.isLineOfSightOccluded(p_s, p_h)
                        % Light attenuation with distance & depth
                        vis_conf = exp(-0.07 * dist) * max(0.3, 1.0 - abs(p_s(3))/45.0);
                        if vis_conf > 0.18
                            det.detected = true;
                            det.confidence = min(0.98, vis_conf + 0.15);
                            det.world_pos = p_h + 0.25 * randn(3, 1);
                            det.pixel_uv = [320 - 320*(az/(obj.HorizontalFOV/2)), ...
                                            240 - 240*(el/(obj.VerticalFOV/2))];
                            return;
                        end
                    end
                end
            end

            % Occasional optical false positive (e.g. coral/fish reflection)
            if rand() < obj.FalsePositiveProb
                det.detected = true;
                det.is_false_positive = true;
                det.confidence = 0.28;
                det.world_pos = p_s + R_wb * [8 + 6*rand(); 4*randn(); randn()];
            end
        end
    end
end
