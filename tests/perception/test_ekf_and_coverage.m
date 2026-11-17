function tests = test_ekf_and_coverage
    tests = functiontests(localfunctions);
end

function testEKFConvergence(testCase)
    ekf = perception.EKFTargetTracker();
    true_pos = [10; 20; -12];
    for k = 1:15
        ekf = ekf.predict(0.05);
        ekf = ekf.update(true_pos + 0.1 * randn(3, 1));
    end
    verifyTrue(testCase, ekf.Initialized);
    err = norm(ekf.x(1:3) - true_pos);
    verifyLessThan(testCase, err, 0.5);
end

function testCoverageGridIncreases(testCase)
    cov = perception.CoverageGrid();
    p0 = cov.getCoveragePercent();
    cov = cov.markVisited([0; 0; -10], 20.0);
    p1 = cov.getCoveragePercent();
    verifyGreaterThan(testCase, p1, p0);
end
