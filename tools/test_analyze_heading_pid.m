function test_analyze_heading_pid(outputDir)
% Synthetic parser/units regression only; this is not vehicle tuning evidence.
arguments
    outputDir (1,1) string
end
if ~isfolder(outputDir), mkdir(outputDir); end
file = fullfile(outputDir, 'synthetic-parser-fixture.txt');
lines = ["SYNTHETIC PARSER TEST - NOT REAL VEHICLE DATA"; ...
    "PIDCFG span=1 kp_m=2000 ki_m=250 kd_m=120 limit_m=12000 integral_m=12000"; ...
    "PID 1010,1000,1,6,8,2,17,17900,-17900,400,0,-16,-16,-400,0,0,1000,20,1"; ...
    "PID 1110,1100,1,6,8,2,17,17900,-17950,400,0,-12,-14,-300,0,0,1000,20,1"; ...
    "PID incomplete"; ...
    "YAW ms=1110 step=6 seg=8 tpl=17 phase=D target=17900 actual=-17950 error=-150 age=10"];
writelines(lines, file);
result = analyze_heading_pid(file, outputDir);
assert(height(result.Samples) == 2 && result.DiscardedRows == 1);
assert(isequal(result.Samples.error_deg, [-2; -1.5]));
assert(isequal(result.Samples.actual_near_target_deg, [181; 180.5]));
assert(isequal(result.Samples.forward_rpm, [100; 100]));
assert(isequal(result.Samples.turn_rpm, [-4; -3]));
assert(isequal(result.Samples.mean_turn_rpm, [-4; -3.5]));
assert(all(result.Samples.age_ms == 10) && all(result.Samples.valid));
assert(max(abs(result.Samples.pid_replay_error_rpm)) < 1e-10);
assert(height(result.CentreSummary) == 1 && result.CentreSummary.samples == 2);
assert(abs(result.CentreSummary.duration_s - 0.1) < 1e-10);
assert(isfile(fullfile(outputDir, 'samples.csv')) && isfile(fullfile(outputDir, 'recording.png')));
fprintf('PASS synthetic MATLAB parser, unit conversion, signed wrap, PID replay and chart export.\n');
end
