function tests = test_planner_and_pid
    tests = functiontests(localfunctions);
end

function testAStarAvoidsObstacles(testCase)
    arena = sim.UnderwaterArena();
    planner = navigation.AStarPlanner3D(arena, 6.0);
    path = planner.plan([-15; 5; -15], [15; 5; -15]);
    verifyGreaterThanOrEqual(testCase, size(path, 2), 2);
    % Check intermediate waypoints are not inside CoralLighthouse
    for k = 2:size(path, 2)-1
        [hit, ~, ~] = arena.checkCollision(path(:, k), 0.1);
        verifyFalse(testCase, hit);
    end
end

function testPIDControllerOutput(testCase)
    pid = navigation.PIDController6DOF();
    eta = [0; 0; -10; 0; 0; 0];
    nu  = zeros(6, 1);
    [tau, ~] = pid.computeControl(eta, nu, [1.5; 0; 0], 0.0, 0.05);
    verifyEqual(testCase, size(tau), [6, 1]);
    verifyGreaterThan(testCase, tau(1), 0);
end

function testSeekerStateMachine(testCase)
    arena = sim.UnderwaterArena();
    ai = navigation.SeekerAI(arena);
    tracker = perception.EKFTargetTracker();
    covGrid = perception.CoverageGrid();
    scan.obstacle_points = zeros(3, 0);

    [~, ~, ai] = ai.step([0;0;-10;0;0;0], tracker, scan, covGrid, "HIDING", 50.0);
    verifyEqual(testCase, ai.State, "IDLE");

    [~, ~, ai] = ai.step([0;0;-10;0;0;0], tracker, scan, covGrid, "SEEKING", 50.0);
    verifyEqual(testCase, ai.State, "SEARCHING");

    tracker = tracker.update([10; 0; -10]);
    [~, ~, ai] = ai.step([0;0;-10;0;0;0], tracker, scan, covGrid, "SEEKING", 10.0);
    verifyEqual(testCase, ai.State, "APPROACHING");
end
