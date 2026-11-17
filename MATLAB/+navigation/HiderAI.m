classdef HiderAI
% HIDERAI Autonomous Hider evasion strategy using obstacle cover & boundary avoidance.

    properties
        MaxSpeed = 1.55           % m/s (slightly slower than Seeker so capture is possible)
        SafeMargin = 10.0         % m from arena walls
        HideAnchor = [-22; -18; -16] % Initial cover near shipwreck/obstacles
    end

    methods
        function [v_des, yaw_des, obj] = step(obj, hider_eta, seeker_eta, arena, phase, t)
            p_h = hider_eta(1:3);
            p_s = seeker_eta(1:3);

            if strcmp(phase, "HIDING")
                % Headstart: move toward obstacle cover away from Seeker
                away = p_h - p_s;
                away(3) = 0;
                if norm(away) > 1e-3, away = away / norm(away); else, away = [-1; -1; 0]; end
                goal = p_h + away * 12.0;
                goal(3) = -16.0;
                diff_g = goal - p_h;
                v_att = (diff_g / max(1e-4, norm(diff_g))) * obj.MaxSpeed;
            else
                dist_s = norm(p_h - p_s);
                if dist_s < 32.0
                    % Active evasion: flee away from Seeker + weave tangentially
                    away = (p_h - p_s) / max(1e-4, dist_s);
                    tangent = [-away(2); away(1); 0] * 0.55 * sin(0.35 * t);
                    dir = away + tangent;
                    dir(3) = 0.25 * sin(0.4 * t);
                    v_att = (dir / max(1e-4, norm(dir))) * obj.MaxSpeed;
                else
                    % Patrol / loiter behind obstacles
                    angle = 0.12 * t;
                    orbit_pt = [-18 + 18*cos(angle); -10 + 18*sin(angle); -15 + 3*sin(0.2*t)];
                    diff_o = orbit_pt - p_h;
                    v_att = (diff_o / max(1e-4, norm(diff_o))) * (obj.MaxSpeed * 0.75);
                end
            end

            % Repulsion from obstacles & walls
            v_rep = [0; 0; 0];
            b = arena.Bounds;
            if p_h(1) < b(1) + obj.SafeMargin, v_rep(1) = v_rep(1) + 1.8; end
            if p_h(1) > b(2) - obj.SafeMargin, v_rep(1) = v_rep(1) - 1.8; end
            if p_h(2) < b(3) + obj.SafeMargin, v_rep(2) = v_rep(2) + 1.8; end
            if p_h(2) > b(4) - obj.SafeMargin, v_rep(2) = v_rep(2) - 1.8; end
            if p_h(3) < b(5) + 3.0,            v_rep(3) = v_rep(3) + 1.2; end
            if p_h(3) > b(6) - 2.0,            v_rep(3) = v_rep(3) - 1.2; end

            for i = 1:numel(arena.Obstacles)
                o = arena.Obstacles(i);
                dv = p_h(1:2) - o.pos(1:2)';
                d = norm(dv) - o.radius;
                if d < 5.0 && d > 0.1
                    v_rep(1:2) = v_rep(1:2) + 2.2 * (1.0/d - 1.0/5.0) * (dv / norm(dv));
                end
            end

            v_des = v_att + v_rep;
            spd = norm(v_des);
            if spd > obj.MaxSpeed
                v_des = (v_des / spd) * obj.MaxSpeed;
            end
            yaw_des = atan2(v_des(2), v_des(1));
        end
    end
end
