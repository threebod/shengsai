function result = analyze_heading_pid(logFile, outputDir)
% Decode real firmware PID logs for identification and later closed-loop tuning.
% This intake step checks the recording and plots it; it does not invent a
% vehicle model or recommend gains before measured responses are available.
arguments
    logFile (1,1) string
    outputDir (1,1) string = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'build', 'heading-pid')
end
lines = strip(readlines(logFile));
names = {'ms','imu_ms','span','step','segment','sub','template','target_cd','actual_cd', ...
    'forward4','right4','turn4','mean_turn4','pid100','integral1000','rate100','scale1000','dt_ms','yaw_sign'};
rows = zeros(0, numel(names));
cfg = zeros(0, 6);
discarded = 0;
for i = 1:numel(lines)
    line = char(lines(i));
    if startsWith(line, 'PIDCFG ')
        values = sscanf(line, 'PIDCFG span=%f kp_m=%f ki_m=%f kd_m=%f limit_m=%f integral_m=%f');
        if numel(values) == 6 && all(isfinite(values)), cfg(end+1,:) = values'; end %#ok<AGROW>
    elseif startsWith(line, 'PID ')
        values = sscanf(line(5:end), '%f,');
        if numel(values) == numel(names) && all(isfinite(values))
            rows(end+1,:) = values'; %#ok<AGROW>
        else
            discarded = discarded + 1;
        end
    end
end
if isempty(rows)
    error('heading_pid:NoSamples', 'No PID rows found. Update firmware and read its latest task log.');
end
t = array2table(rows, 'VariableNames', names);
t.time_s = mod(t.ms - t.ms(1), 2^32) / 1000;
t.age_ms = mod(t.ms - t.imu_ms, 2^32);
t.target_deg = t.target_cd / 100;
t.actual_deg = t.actual_cd / 100;
t.error_deg = mod(t.target_deg - t.actual_deg + 180, 360) - 180;
t.actual_near_target_deg = t.target_deg - t.error_deg;
t.forward_rpm = t.forward4 / 4;
t.right_rpm = t.right4 / 4;
t.turn_rpm = t.turn4 / 4;
t.mean_turn_rpm = t.mean_turn4 / 4;
t.raw_pid_rpm = t.pid100 / 100;
t.integral_deg_s = t.integral1000 / 1000;
t.filtered_rate_dps = t.rate100 / 100;
t.output_scale = t.scale1000 / 1000;
t.valid = t.age_ms <= 250 & t.dt_ms >= 20 & t.dt_ms <= 150 & abs(t.yaw_sign) == 1;
t.predicted_pid_rpm = NaN(height(t), 1);
t.pid_limit_rpm = NaN(height(t), 1);
% I and filtered yaw rate are logged after the real firmware control step.
% Reconstruct its raw PID output, including deadband, clamp and yaw polarity.
for i = 1:size(cfg, 1)
    select = t.span == cfg(i,1);
    errorDeg = t.error_deg(select);
    errorDeg(abs(errorDeg) < 0.3) = 0;
    raw = cfg(i,2)/1000 * errorDeg + cfg(i,3)/1000 * t.integral_deg_s(select) ...
        - cfg(i,4)/1000 * t.filtered_rate_dps(select);
    limit = cfg(i,5)/1000;
    t.predicted_pid_rpm(select) = max(-limit, min(limit, raw)) .* t.yaw_sign(select);
    t.pid_limit_rpm(select) = limit;
end
t.pid_replay_error_rpm = t.raw_pid_rpm - t.predicted_pid_rpm;
t.saturated = abs(t.raw_pid_rpm) >= t.pid_limit_rpm - 0.01;
% RC-1/sub=2 is the final long down leg, excluding the 100mm/50mm departure.
centre = t.valid & t.template == 17 & t.sub == 2;
spans = unique(t.span(centre));
summary = table('Size',[0 6], 'VariableTypes',repmat("double",1,6), ...
    'VariableNames',{'span','samples','duration_s','mean_error_deg','rms_error_deg','max_error_deg'});
for i = 1:numel(spans)
    selected = t(centre & t.span == spans(i),:);
    summary(end+1,:) = {spans(i), height(selected), ...
        mod(selected.ms(end)-selected.ms(1),2^32)/1000, mean(selected.error_deg), ...
        sqrt(mean(selected.error_deg.^2)), max(abs(selected.error_deg))}; %#ok<AGROW>
end
if ~isfolder(outputDir), mkdir(outputDir); end
writetable(t, fullfile(outputDir, 'samples.csv'));
writetable(summary, fullfile(outputDir, 'centre-spans.csv'));
fig = figure('Visible','off','Name','Recorded heading/PID commands');
closeFigure = onCleanup(@() close(fig));
tiledlayout(fig, 4, 1);
nexttile; plot(t.time_s, t.target_deg, '--', t.time_s, t.actual_near_target_deg); ylabel('deg'); legend('target','IMU near target'); grid on;
nexttile; plot(t.time_s, t.error_deg); yline(0.5, '--'); yline(-0.5, '--'); ylabel('error deg'); grid on;
nexttile; plot(t.time_s, t.raw_pid_rpm, t.time_s, t.turn_rpm); ylabel('turn RPM'); legend('raw PID','wheel command mix'); grid on;
nexttile; plot(t.time_s, t.forward_rpm, t.time_s, t.right_rpm); ylabel('command RPM'); xlabel('time s'); legend('forward','right'); grid on;
exportgraphics(fig, fullfile(outputDir, 'recording.png'), 'Resolution',150);
result = struct('Samples',t,'Gains',cfg,'CentreSummary',summary,'DiscardedRows',discarded, ...
    'Status',"recording analysis only; vehicle model and gains require real-data validation");
fprintf('Parsed %d PID samples, %d centre spans; discarded %d malformed rows.\n', height(t), height(summary), discarded);
fprintf('Outputs: %s\n', outputDir);
end
