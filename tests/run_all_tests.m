function results = run_all_tests()
% RUN_ALL_TESTS Runs all unit tests across simulation, navigation, and perception.
    rootDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(rootDir, 'MATLAB'));

    suite = [ ...
        testsuite(fullfile(rootDir, 'tests', 'simulation', 'test_arena_and_dynamics.m')), ...
        testsuite(fullfile(rootDir, 'tests', 'navigation', 'test_planner_and_pid.m')), ...
        testsuite(fullfile(rootDir, 'tests', 'perception', 'test_ekf_and_coverage.m')) ...
    ];
    results = run(suite);
    disp(table(results));
    assert(all([results.Passed]), 'One or more unit tests failed!');
end
