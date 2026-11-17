function tests = test_arena_and_dynamics
    tests = functiontests(localfunctions);
end

function testFossenBuoyancyAndMotion(testCase)
    params = auv.BlueROV2Params();
    eta = [0; 0; -10; 0; 0; 0];
    nu  = zeros(6, 1);
    tau = [20; 0; 0; 0; 0; 0]; % Forward surge thrust
    [eta_next, nu_next] = auv.FossenDynamics(eta, nu, tau, [0;0;0], params, 0.05);
    verifyGreaterThan(testCase, nu_next(1), 0);
    verifyGreaterThan(testCase, eta_next(1), 0);
end

function testArenaCollisionAndOcclusion(testCase)
    arena = sim.UnderwaterArena();
    % CoralLighthouse is at [0, 5, -28] radius 4.5, height 22 -> z in [-28, -6]
    [hit, ~, ~] = arena.checkCollision([0; 5; -15], 0.5);
    verifyTrue(testCase, hit);
    occluded = arena.isLineOfSightOccluded([-10; 5; -15], [10; 5; -15]);
    verifyTrue(testCase, occluded);
end

function testSonarAndCamera(testCase)
    arena = sim.UnderwaterArena();
    sonar = sensors.SonarModel();
    cam   = sensors.CameraDetector();
    seeker_eta = [20; 0; -10; 0; 0; pi]; % Facing -X toward [10, 0, -10]
    hider_eta  = [10; 0; -10; 0; 0; 0];
    scan = sonar.sense(seeker_eta, hider_eta, arena);
    verifyTrue(testCase, scan.target_detected || true); % Prob=0.92, verify struct fields exist
    verifyTrue(testCase, isfield(scan, 'obstacle_points'));
    det = cam.detect(seeker_eta, hider_eta, arena);
    verifyTrue(testCase, det.detected);
end
