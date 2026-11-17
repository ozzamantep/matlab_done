function summary = run_benchmark(num_trials)
% RUN_BENCHMARK Runs Monte Carlo evaluation of the Hide-and-Seek AUV system.
%   Generates summary statistics and saves plots to MATLAB/results/.

    if nargin < 1, num_trials = 10; end

    rootDir = fileparts(mfilename('fullpath'));
    addpath(rootDir);
    outDir = fullfile(rootDir, 'results');
    if ~exist(outDir, 'dir'), mkdir(outDir); end

    winners = strings(1, num_trials);
    cap_times = nan(1, num_trials);
    ekf_rmses = nan(1, num_trials);
    efforts = nan(1, num_trials);

    fprintf('=== Running %d Monte Carlo Trials ===\n', num_trials);
    for i = 1:num_trials
        res = run_hide_and_seek(true, 100 + i);
        winners(i) = res.winner;
        cap_times(i) = res.capture_time;
        ekf_rmses(i) = res.ekf_rmse;
        efforts(i) = res.mean_effort;
        if i == 1
            sample_res = res;
        end
    end

    capture_rate = mean(winners == "SEEKER") * 100;
    valid_cap = cap_times(~isnan(cap_times));
    mean_cap_time = mean(valid_cap);
    mean_ekf_rmse = mean(ekf_rmses(~isnan(ekf_rmses)));

    fprintf('\n=== Benchmark Summary (%d Trials) ===\n', num_trials);
    fprintf('Seeker Capture Rate : %.1f%%\n', capture_rate);
    fprintf('Mean Capture Time   : %.2f s\n', mean_cap_time);
    fprintf('Mean EKF Pos RMSE   : %.2f m\n', mean_ekf_rmse);
    fprintf('Mean Thruster Effort: %.2f N\n', mean(efforts));

    % Generate summary figure (headless-safe)
    fig = figure('Visible', 'off', 'Position', [100, 100, 1100, 450]);
    subplot(1, 2, 1);
    plot(sample_res.t, sample_res.dist, 'b-', 'LineWidth', 1.8); hold on;
    yline(3.0, 'r--', 'Capture Radius (3m)', 'LineWidth', 1.2);
    xlabel('Time (s)'); ylabel('Seeker-Hider Distance (m)');
    title('Sample Episode Distance Profile'); grid on;

    subplot(1, 2, 2);
    plot3(sample_res.seeker_pos(1,:), sample_res.seeker_pos(2,:), sample_res.seeker_pos(3,:), 'y-', 'LineWidth', 1.8); hold on;
    plot3(sample_res.hider_pos(1,:), sample_res.hider_pos(2,:), sample_res.hider_pos(3,:), 'c-', 'LineWidth', 1.8);
    legend('Seeker AUV', 'Hider AUV', 'Location', 'best');
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Depth (m)');
    title('Sample 3D Intercept Trajectory'); grid on; view(35, 25);

    saveas(fig, fullfile(outDir, 'benchmark_summary.png'));
    close(fig);

    summary.capture_rate = capture_rate;
    summary.mean_capture_time = mean_cap_time;
    summary.mean_ekf_rmse = mean_ekf_rmse;
    summary.mean_effort = mean(efforts);
end
