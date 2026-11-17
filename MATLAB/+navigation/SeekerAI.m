classdef SeekerAI
% SEEKERAI Autonomous Seeker Decision State Machine (SEARCH -> TRACK -> INTERCEPT).
%   Includes 3D Lawnmower/Spiral search waypoints and Artificial Potential Field (APF)
%   reactive obstacle avoidance.

    properties
        State string = "SEARCH"
        Waypoints (3,:) double
        WpIdx double = 1
        CruiseSpeed = 1.65        % m/s
        InterceptSpeed = 2.05     % m/s
        RepulsiveGain = 18.0
        RepulsiveRange = 6.5
    end

    methods
        function obj = SeekerAI()
            % 3D Lawnmower + Spiral patrol covering the 120x120m arena
            pts = [
                -35, -35, -10;
                 35, -35, -12;
                 35, -10, -14;
                -35, -10, -12;
                -35,  15, -14;
                 35,  15, -12;
                 35,  38, -10;
                -35,  38, -12;
                  0,   0, -15
            ]';
            obj.Waypoints = pts;
        end

        function [v_des, yaw_des, obj] = step(obj, seeker_eta, tracker, scan, phase)
            p_s = seeker_eta(1:3);

            % During HidingPhase headstart, Seeker holds position / scans slowly
            if strcmp(phase, "HIDING")
                obj.State = "WAIT_HEADSTART";
                v_des = [0; 0; 0];
                yaw_des = seeker_eta(6) + 0.35;
                return;
            end

            % State transitions based on EKF confidence and time since last measurement
            if tracker.Initialized && tracker.TimeSinceLastMeas < 6.0
                dist_est = norm(tracker.x(1:3) - p_s);
                if dist_est < 16.0 && tracker.Confidence > 0.45
                    obj.State = "INTERCEPT";
                else
                    obj.State = "TRACK";
                end
            else
                obj.State = "SEARCH";
            end

            % Desired velocity vector before obstacle avoidance
            if obj.State == "SEARCH"
                wp = obj.Waypoints(:, obj.WpIdx);
                diff_wp = wp - p_s;
                if norm(diff_wp) < 4.5
                    obj.WpIdx = mod(obj.WpIdx, size(obj.Waypoints, 2)) + 1;
                    wp = obj.Waypoints(:, obj.WpIdx);
                    diff_wp = wp - p_s;
                end
                dir = diff_wp / max(1e-4, norm(diff_wp));
                v_att = dir * obj.CruiseSpeed;
                yaw_des = atan2(dir(2), dir(1));

            elseif obj.State == "TRACK"
                % Pursue EKF estimated position
                target_pos = tracker.x(1:3);
                diff_t = target_pos - p_s;
                dir = diff_t / max(1e-4, norm(diff_t));
                v_att = dir * (obj.CruiseSpeed * 1.1);
                yaw_des = atan2(dir(2), dir(1));

            else % INTERCEPT (Lead Pursuit)
                target_pos = tracker.x(1:3);
                target_vel = tracker.x(4:6);
                dist = norm(target_pos - p_s);
                t_lead = min(2.5, dist / obj.InterceptSpeed);
                intercept_pt = target_pos + 0.7 * target_vel * t_lead;
                intercept_pt(3) = max(min(intercept_pt(3), -1.0), -26.0);

                diff_i = intercept_pt - p_s;
                dir = diff_i / max(1e-4, norm(diff_i));
                v_att = dir * obj.InterceptSpeed;
                yaw_des = atan2(dir(2), dir(1));
            end

            % Artificial Potential Field (APF) reactive obstacle avoidance
            v_rep = [0; 0; 0];
            if ~isempty(scan.obstacle_points)
                for k = 1:size(scan.obstacle_points, 2)
                    obs_pt = scan.obstacle_points(:, k);
                    dv = p_s - obs_pt;
                    d = norm(dv);
                    if d < obj.RepulsiveRange && d > 0.1
                        mag = obj.RepulsiveGain * (1.0/d - 1.0/obj.RepulsiveRange) / (d^2);
                        v_rep = v_rep + min(mag, 2.2) * (dv / d);
                    end
                end
            end

            v_des = v_att + v_rep;
            spd = norm(v_des);
            max_spd = obj.InterceptSpeed;
            if spd > max_spd
                v_des = (v_des / spd) * max_spd;
            end
        end
    end
end
