classdef UnderwaterArena
% UNDERWATERARENA 3D underwater environment with seabed, obstacles, and ocean currents.
%   Coordinate frame: ENU (x: East, y: North, z: Up; water surface z=0, seabed ~z=-28m)

    properties
        Bounds = [-60, 60, -60, 60, -28, -0.5];  % [xmin xmax ymin ymax zmin zmax]
        Obstacles struct                         % Array of cylinder/sphere obstacles
        CaptureRadius = 3.0;                     % meters
    end

    methods
        function obj = UnderwaterArena()
            % Define realistic underwater obstacles matching UE5 UnderwaterArena:
            % Coral Lighthouse, Cruise Shipwreck, Reef Pillars, Rock Formations
            obs(1) = struct('name', 'CoralLighthouse', 'type', 'cylinder', ...
                'pos', [0, 5, -28], 'radius', 4.5, 'height', 22.0, 'color', [0.85, 0.55, 0.35]);
            obs(2) = struct('name', 'ShipwreckHull', 'type', 'cylinder', ...
                'pos', [-22, -18, -28], 'radius', 7.0, 'height', 12.0, 'color', [0.45, 0.38, 0.32]);
            obs(3) = struct('name', 'NorthReefPillar', 'type', 'cylinder', ...
                'pos', [20, 22, -28], 'radius', 5.0, 'height', 16.0, 'color', [0.35, 0.65, 0.55]);
            obs(4) = struct('name', 'EastRockArch', 'type', 'cylinder', ...
                'pos', [28, -15, -28], 'radius', 5.5, 'height', 14.0, 'color', [0.42, 0.45, 0.48]);
            obs(5) = struct('name', 'SouthCoralCluster', 'type', 'cylinder', ...
                'pos', [-15, 25, -28], 'radius', 4.0, 'height', 15.0, 'color', [0.75, 0.42, 0.52]);
            obs(6) = struct('name', 'DeepBoulder', 'type', 'sphere', ...
                'pos', [8, -26, -22], 'radius', 5.0, 'height', 10.0, 'color', [0.38, 0.40, 0.42]);
            obj.Obstacles = obs;
        end

        function z_bed = getSeabedHeight(~, x, y)
            % Gentle undulating seafloor around -28m
            z_bed = -28.0 + 1.4 * sin(0.06 * x) .* cos(0.05 * y) + 0.6 * cos(0.11 * x + 0.08 * y);
        end

        function curr = getOceanCurrent(~, pos, t)
            % Spatially & temporally varying 3D ocean current (m/s)
            x = pos(1); y = pos(2); z = pos(3);
            depth_factor = max(0.2, 1.0 - abs(z) / 35.0);
            cx = (0.28 + 0.12 * sin(0.05*y + 0.15*t)) * depth_factor;
            cy = (0.14 * cos(0.04*x + 0.10*t)) * depth_factor;
            cz = 0.03 * sin(0.08*x + 0.08*y + 0.2*t);
            curr = [cx; cy; cz];
        end

        function [hit, min_dist, hit_normal] = checkCollision(obj, pos, margin)
            if nargin < 3, margin = 0.8; end
            hit = false;
            min_dist = inf;
            hit_normal = [0; 0; 1];

            z_bed = obj.getSeabedHeight(pos(1), pos(2));
            d_bed = pos(3) - z_bed;
            if d_bed < margin
                hit = true;
                min_dist = d_bed;
                hit_normal = [0; 0; 1];
            end

            for i = 1:numel(obj.Obstacles)
                o = obj.Obstacles(i);
                if strcmp(o.type, 'cylinder')
                    dx = pos(1) - o.pos(1);
                    dy = pos(2) - o.pos(2);
                    horiz_dist = hypot(dx, dy) - o.radius;
                    in_z = (pos(3) >= o.pos(3)) && (pos(3) <= o.pos(3) + o.height + margin);
                    if in_z && horiz_dist < min_dist
                        min_dist = horiz_dist;
                        if horiz_dist < margin
                            hit = true;
                            nrm = [dx; dy; 0];
                            if norm(nrm) > 1e-4, hit_normal = nrm / norm(nrm); end
                        end
                    end
                elseif strcmp(o.type, 'sphere')
                    dv = pos(:) - o.pos(:);
                    dist = norm(dv) - o.radius;
                    if dist < min_dist
                        min_dist = dist;
                        if dist < margin
                            hit = true;
                            if norm(dv) > 1e-4, hit_normal = dv / norm(dv); end
                        end
                    end
                end
            end
        end

        function occluded = isLineOfSightOccluded(obj, p1, p2)
            % Ray-march check between p1 and p2 against all obstacles
            occluded = false;
            n_steps = 20;
            for k = 1:n_steps-1
                s = k / n_steps;
                pt = p1(:) * (1 - s) + p2(:) * s;
                [hit, ~, ~] = obj.checkCollision(pt, 0.2);
                if hit
                    occluded = true;
                    return;
                end
            end
        end

        function [pos_resolved, vel_resolved] = resolveCollision(obj, pos, vel)
            [hit, ~, nrm] = obj.checkCollision(pos, 0.9);
            pos_resolved = pos;
            vel_resolved = vel;
            if hit
                pos_resolved = pos + nrm * 0.45;
                v_dot = dot(vel(1:3), nrm);
                if v_dot < 0
                    vel_resolved(1:3) = vel(1:3) - 1.3 * v_dot * nrm;
                end
            end
        end
    end
end
