function test_turn_pid(fixtureDir, outputDir)
% Fixtures come from test_turn_test.exe: production C PID plus a known ARX
% plant. They verify software contracts, not real vehicle performance.
arguments
    fixtureDir (1,1) string = fullfile(fileparts(fileparts(mfilename('fullpath'))),'build','turn-test-fixtures')
    outputDir (1,1) string = fullfile(fileparts(fileparts(mfilename('fullpath'))),'build','turn-matlab-selftest')
end
files=fullfile(fixtureDir,compose('run%d.txt',1:4));
r=simulate_turn_pid(files,fullfile(outputDir,'valid'));
assert(abs(r.model.a-0.4)<0.04 && abs(r.model.b-0.07)<0.003);
assert(r.validation_rmse_deg<0.1 && r.baseline_closed_loop_rmse_deg<0.1 && r.model_accepted);
assert(~isempty(r.recommended_pid));
image=imread(fullfile(outputDir,'valid','turn-recording.png'));
assert(numel(unique(image(:)))>32,'Recorded-data plot is blank.');
changed=fullfile(fixtureDir,compose('run%d.txt',[1,2,5,6]));
rejected=simulate_turn_pid(changed,fullfile(outputDir,'changed-plant'));
assert(~rejected.model_accepted && isempty(rejected.recommended_pid));
original=readlines(files(1));
if ~isfolder(outputDir),mkdir(outputDir);end
bad=fullfile(outputDir,'incomplete.txt');writelines(original(~startsWith(original,'TURN_END ')),bad);
expectError(@()analyze_turn_pid(bad,fullfile(outputDir,'incomplete')),'turn_pid:Incomplete');
bad=fullfile(outputDir,'aborted.txt');writelines(replace(original,'result=complete','result=aborted'),bad);
expectError(@()simulate_turn_pid([bad,files(2:4)],fullfile(outputDir,'aborted')),'turn_pid:RecordingRejected');
expectError(@()simulate_turn_pid(files(1:2),fullfile(outputDir,'too-few')),'turn_pid:NeedValidationRuns');
% Shift both clocks across uint32 wrap, leaving angles and command values intact.
shifted=original;offset=2^32-2000;
for i=1:numel(shifted)
    line=char(shifted(i));
    if startsWith(line,'TC ') || startsWith(line,'TY ')
        values=sscanf(line(4:end),'%f,')';count=1+startsWith(line,'TY ');
        values(1:count)=mod(values(1:count)+offset,2^32);
        shifted(i)=string(line(1:3))+strjoin(compose('%.0f',values),',');
    elseif startsWith(line,'TURN_TEST ') || startsWith(line,'TURN_END ')
        [a,b,tokens]=regexp(line,'\<(?:ms|imu_ms)=(\d+)','start','end','tokens');
        for k=numel(a):-1:1
            token=line(a(k):b(k));equal=find(token=='=',1);
            replacement=[token(1:equal),sprintf('%.0f',mod(str2double(tokens{k}{1})+offset,2^32))];
            line=[line(1:a(k)-1),replacement,line(b(k)+1:end)];
        end
        shifted(i)=string(line);
    end
end
wrapped=fullfile(outputDir,'wrapped.txt');writelines(shifted,wrapped);
a=analyze_turn_pid(files(1),fullfile(outputDir,'original'));
b=analyze_turn_pid(wrapped,fullfile(outputDir,'wrapped'));
assert(max(abs(a.Samples.sensor_s-b.Samples.sensor_s))<1e-9);
assert(max(abs(a.Commands.time_s-b.Commands.time_s))<1e-9);
assert(all(a.Samples.angle_deg==b.Samples.angle_deg));
fprintf('PASS turn MATLAB: C PID replay, known model/closed-loop recovery, held-out validation, candidates, rejection and uint32 wrap.\n');
end

function expectError(action,id)
try
    action();
catch exception
    assert(strcmp(exception.identifier,id),exception.message);return;
end
error('turn_pid:ExpectedFailure','Expected %s.',id);
end
