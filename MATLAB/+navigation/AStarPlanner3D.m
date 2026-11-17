classdef AStarPlanner3D
% ASTARPLANNER3D 3D Grid-based A* Path Planner for obstacle-free AUV navigation.

    properties
        Resolution = 5.0          % meters per voxel
        Bounds = [-60, 60, -60, 60, -28, -2]
        GridSize (1,3) double
        OccGrid logical
    end

    methods
        function obj = AStarPlanner3D(arena, resolution)
            if nargin >= 2, obj.Resolution = resolution; end
            obj.Bounds = [arena.Bounds(1:4), arena.Bounds(5), -2.0];
            nx = floor((obj.Bounds(2) - obj.Bounds(1)) / obj.Resolution) + 1;
            ny = floor((obj.Bounds(4) - obj.Bounds(3)) / obj.Resolution) + 1;
            nz = floor((obj.Bounds(6) - obj.Bounds(5)) / obj.Resolution) + 1;
            obj.GridSize = [nx, ny, nz];
            obj.OccGrid = false(nx, ny, nz);

            % Populate 3D occupancy grid from arena obstacles
            for ix = 1:nx
                for iy = 1:ny
                    for iz = 1:nz
                        pt = obj.idxToWorld([ix, iy, iz]);
                        [hit, ~, ~] = arena.checkCollision(pt, 2.0);
                        obj.OccGrid(ix, iy, iz) = hit;
                    end
                end
            end
        end

        function idx = worldToIdx(obj, pt)
            ix = round((pt(1) - obj.Bounds(1)) / obj.Resolution) + 1;
            iy = round((pt(2) - obj.Bounds(3)) / obj.Resolution) + 1;
            iz = round((pt(3) - obj.Bounds(5)) / obj.Resolution) + 1;
            ix = max(1, min(obj.GridSize(1), ix));
            iy = max(1, min(obj.GridSize(2), iy));
            iz = max(1, min(obj.GridSize(3), iz));
            idx = [ix, iy, iz];
        end

        function pt = idxToWorld(obj, idx)
            x = obj.Bounds(1) + (idx(1) - 1) * obj.Resolution;
            y = obj.Bounds(3) + (idx(2) - 1) * obj.Resolution;
            z = obj.Bounds(5) + (idx(3) - 1) * obj.Resolution;
            pt = [x; y; z];
        end

        function path = plan(obj, startPt, goalPt)
            sIdx = obj.worldToIdx(startPt);
            gIdx = obj.worldToIdx(goalPt);

            % Ensure start/goal voxels are traversable
            obj.OccGrid(sIdx(1), sIdx(2), sIdx(3)) = false;
            obj.OccGrid(gIdx(1), gIdx(2), gIdx(3)) = false;

            nx = obj.GridSize(1); ny = obj.GridSize(2); nz = obj.GridSize(3);
            gScore = inf(nx, ny, nz);
            fScore = inf(nx, ny, nz);
            closed = false(nx, ny, nz);
            cameFrom = zeros(nx, ny, nz, 3, 'int32');

            gScore(sIdx(1), sIdx(2), sIdx(3)) = 0;
            fScore(sIdx(1), sIdx(2), sIdx(3)) = norm(double(gIdx - sIdx));

            % Open list as Nx4 [f, ix, iy, iz]
            openList = [fScore(sIdx(1), sIdx(2), sIdx(3)), sIdx];

            % 6-connected + planar diagonal neighbors (10 directions for speed)
            nbrs = [ 1  0  0; -1  0  0;  0  1  0;  0 -1  0;  0  0  1;  0  0 -1; ...
                     1  1  0;  1 -1  0; -1  1  0; -1 -1  0];

            found = false;
            maxIter = 2500;
            iter = 0;
            while ~isempty(openList) && iter < maxIter
                iter = iter + 1;
                [~, bestRow] = min(openList(:, 1));
                curr = openList(bestRow, 2:4);
                openList(bestRow, :) = [];

                if isequal(curr, gIdx)
                    found = true;
                    break;
                end

                if closed(curr(1), curr(2), curr(3)), continue; end
                closed(curr(1), curr(2), curr(3)) = true;

                for k = 1:size(nbrs, 1)
                    nb = curr + nbrs(k, :);
                    if any(nb < 1) || nb(1) > nx || nb(2) > ny || nb(3) > nz
                        continue;
                    end
                    if closed(nb(1), nb(2), nb(3)) || obj.OccGrid(nb(1), nb(2), nb(3))
                        continue;
                    end

                    stepCost = norm(double(nbrs(k, :)));
                    tentativeG = gScore(curr(1), curr(2), curr(3)) + stepCost;
                    if tentativeG < gScore(nb(1), nb(2), nb(3))
                        cameFrom(nb(1), nb(2), nb(3), :) = int32(curr);
                        gScore(nb(1), nb(2), nb(3)) = tentativeG;
                        fVal = tentativeG + norm(double(gIdx - nb));
                        fScore(nb(1), nb(2), nb(3)) = fVal;
                        openList(end+1, :) = [fVal, nb]; %#ok<AGROW>
                    end
                end
            end

            if ~found
                path = [startPt(:), goalPt(:)];
                return;
            end

            % Reconstruct path
            idxPath = gIdx;
            curr = gIdx;
            while ~isequal(curr, sIdx)
                prev = double(squeeze(cameFrom(curr(1), curr(2), curr(3), :))');
                if all(prev == 0), break; end
                idxPath = [prev; idxPath]; %#ok<AGROW>
                curr = prev;
            end

            path = zeros(3, size(idxPath, 1));
            for i = 1:size(idxPath, 1)
                path(:, i) = obj.idxToWorld(idxPath(i, :));
            end
            path(:, end) = goalPt(:);
        end
    end
end
