function test_simulate_heading_pid(outputDir)
% Synthetic identification and rejection-gate tests, not real vehicle evidence.
arguments
    outputDir (1,1) string
end
if ~isfolder(outputDir), mkdir(outputDir); end
lines = "SYNTHETIC IDENTIFICATION TEST - NOT REAL VEHICLE DATA";
for span = 1:2
    lines(end+1) = sprintf('PIDCFG span=%d kp_m=2000 ki_m=250 kd_m=120 limit_m=12000 integral_m=12000',span); %#ok<AGROW>
    y=0;dy=0;
    for k=1:40
        forward=round(120+40*sin(k*0.29));
        u=mod(7*k+3*span,9)-4;
        if k>1, dy=0.4*dy+0.08*u-0.1*(forward/240)+0.02;y=y+dy;end
        angle=round(y*100);error=-angle/100;
        raw=0;if abs(error)>=0.3,raw=max(-12,min(12,2*error));end
        ms=10000*span+(k-1)*100;
        lines(end+1) = sprintf('PID %d,%d,%d,%d,%d,2,17,0,%d,%d,0,%d,%d,%d,0,0,1000,20,1', ...
            ms,ms,span,6+7*(span-1),4+5*(span-1),angle,forward*4,u*4,u*4,round(raw*100)); %#ok<AGROW>
    end
end
file=fullfile(outputDir,'synthetic-identification.txt');writelines(lines,file);
r=simulate_heading_pid(file,outputDir);
m=r.Summary.model;
assert(m.delay_samples==0 && abs(m.a-0.4)<0.03 && abs(m.b-0.08)<0.005);
assert(m.validation_fit_percent>90);
% The injected inputs are not outputs of the recorded baseline controller.
% An accurate open-loop fit alone must not release a tuned PID.
assert(r.Summary.baseline_closed_loop_mismatch_deg>0.3);
assert(~r.Summary.model_accepted && isempty(r.Summary.recommended_pid));
assert(height(r.Candidates)==72 && all(r.Candidates.model_accepted==0));
single=fullfile(outputDir,'synthetic-single-run.txt');writelines(lines(1:42),single);
rejected=false;
try
    simulate_heading_pid(single,fullfile(outputDir,'single-run'));
catch e
    rejected=strcmp(e.identifier,'heading_pid:NeedValidationRun');
end
assert(rejected);
fprintf('PASS synthetic known-model recovery, independent validation, baseline mismatch rejection and missing-run rejection.\n');
end
