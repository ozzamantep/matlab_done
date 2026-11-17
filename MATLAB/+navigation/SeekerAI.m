classdef SeekerAI
% SEEKERAI Autonomous Seeker State Machine (IDLE, SEARCHING, DETECTED, APPROACHING, FOUND, LOST).
%   Integrates 3D A* global path planning, CoverageGrid search, and APF obstacle avoidance.

    properties
        State string = "IDLE"
        Planner
        CurrentPath (3,:) double = zeros(3,0)
        PathIdx double = 1
        ReplanCounter double = 0
        CruiseSpeed = 1.65        % m/s
        ApproachSpeed = 2.05      % m/s
        RepulsiveGain = 18.0
        RepulsiveRange = 6.5
    end

    methods
        function obj = SeekerAI(arena)
            if nargin >= 1
                obj.Planner = navigation.AStarPlanner3D(arena, 6.0);
            end
        end

        function [v_des, yaw_des, obj] = step(obj, seeker_eta, tracker, scan, covGrid, phase, dist_true)
            p_s = seeker_eta(1:3);

            if strcmp(phase, "HIDING")
                obj.State = "IDLE";
                v_des = [0; 0; 0];
                yaw_des = seeker_eta(6) + 0.35;
                return;
            end

            % Full Hide-and-Seek State Machine: IDLE, SEARCHING, DETECTED, APPROACHING, FOUND, LOST
            if nargin >= 7 && dist_true <= 3.0
                obj.State = "FOUND";
            elseif tracker.Initialized && tracker.TimeSinceLastMeas < 1.5
                if norm(tracker.x(1:3) - p_s) < 18.0
                    obj.State = "APPROACHING";
                else
                    obj.State = "DETECTED";
                end
            elseif tracker.Initialized && tracker.TimeSinceLastMeas < 6.0
                obj.State = "APPROACHING";
            elseif tracker.Initialized && tracker.TimeSinceLastMeas >= 6.0 && tracker.TimeSinceLastMeas < 11.0
                obj.State = "LOST";
            else
                obj.State = "SEARCHING";
            end

            % Determine goal & desired velocity
            if obj.State == "SEARCHING" || obj.State == "LOST"
                goal = covGrid.getNearestUnvisited(p_s);
                obj.ReplanCounter = obj.ReplanCounter + 1;
                needReplan = isempty(obj.CurrentPath) || ...
                             norm(obj.CurrentPath(:, end) - goal) > 8.0 || ...
                             (obj.PathIdx >= size(obj.CurrentPath, 2) && norm(obj.CurrentPath(:, end) - p_s) < 5.0);
                if ~isempty(obj.Planner) && needReplan
                    obj.CurrentPath = obj.Planner.plan(p_s, goal);
                    obj.PathIdx = min(2, size(obj.CurrentPath, 2));
                end

                if ~isempty(obj.CurrentPath)
                    wp = obj.CurrentPath(:, obj.PathIdx);
                    if norm(wp - p_s) < 4.5 && obj.PathIdx < size(obj.CurrentPath, 2)
                        obj.PathIdx = obj.PathIdx + 1;
                        wp = obj.CurrentPath(:, obj.PathIdx);
                    end
                    target_pt = wp;
                else
                    target_pt = goal;
                end

                diff_t = target_pt - p_s;
                dir = diff_t / max(1e-4, norm(diff_t));
                v_att = dir * obj.CruiseSpeed;
                yaw_des = atan2(dir(2), dir(1));

            else % DETECTED, APPROACHING, FOUND (Lead Pursuit)
                target_pos = tracker.x(1:3);
                target_vel = tracker.x(4:6);
                dist = norm(target_pos - p_s);
                t_lead = min(2.5, dist / obj.ApproachSpeed);
                intercept_pt = target_pos + 0.7 * target_vel * t_lead;
                intercept_pt(3) = max(min(intercept_pt(3), -1.0), -26.0);

                diff_i = intercept_pt - p_s;
                dir = diff_i / max(1e-4, norm(diff_i));
                spd = obj.ApproachSpeed;
                if obj.State == "DETECTED", spd = obj.CruiseSpeed * 1.1; end
                v_att = dir * spd;
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
            if spd > obj.ApproachSpeed
                v_des = (v_des / spd) * obj.ApproachSpeed;
            end
        end
    end
end
