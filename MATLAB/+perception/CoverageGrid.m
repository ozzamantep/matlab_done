classdef CoverageGrid
% COVERAGEGRID Tracks explored 2D/3D search cells to avoid redundant searching.

    properties
        CellSize = 15.0           % meters
        Bounds = [-55, 55, -55, 55]
        Visited logical
        GridSize (1,2) double
    end

    methods
        function obj = CoverageGrid()
            nx = floor((obj.Bounds(2) - obj.Bounds(1)) / obj.CellSize) + 1;
            ny = floor((obj.Bounds(4) - obj.Bounds(3)) / obj.CellSize) + 1;
            obj.GridSize = [nx, ny];
            obj.Visited = false(nx, ny);
        end

        function obj = markVisited(obj, pos, sensorRadius)
            if nargin < 3, sensorRadius = 15.0; end
            rCells = ceil(sensorRadius / obj.CellSize);
            cx = round((pos(1) - obj.Bounds(1)) / obj.CellSize) + 1;
            cy = round((pos(2) - obj.Bounds(3)) / obj.CellSize) + 1;
            for ix = max(1, cx - rCells):min(obj.GridSize(1), cx + rCells)
                for iy = max(1, cy - rCells):min(obj.GridSize(2), cy + rCells)
                    pt = [obj.Bounds(1) + (ix-1)*obj.CellSize;
                          obj.Bounds(3) + (iy-1)*obj.CellSize];
                    if norm(pt - pos(1:2)) <= sensorRadius
                        obj.Visited(ix, iy) = true;
                    end
                end
            end
        end

        function pct = getCoveragePercent(obj)
            pct = 100.0 * sum(obj.Visited(:)) / numel(obj.Visited);
        end

        function target = getNearestUnvisited(obj, pos)
            [ixList, iyList] = find(~obj.Visited);
            if isempty(ixList)
                % Reset if 100% covered
                obj.Visited(:) = false;
                [ixList, iyList] = find(~obj.Visited);
            end
            bestDist = inf;
            target = [0; 0; -12];
            for k = 1:numel(ixList)
                pt = [obj.Bounds(1) + (ixList(k)-1)*obj.CellSize;
                      obj.Bounds(3) + (iyList(k)-1)*obj.CellSize];
                d = norm(pt - pos(1:2)) + 0.35 * norm(pt);
                if d < bestDist
                    bestDist = d;
                    target = [pt(1); pt(2); -13.0];
                end
            end
        end
    end
end
