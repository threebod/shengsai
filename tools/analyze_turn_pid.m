function result = analyze_turn_pid(logFiles, outputDir)
% Decode independent 90-degree tests. RPM values are commands, not feedback.
arguments
    logFiles string
    outputDir (1,1) string = fullfile(fileparts(fileparts(mfilename('fullpath'))),'build','turn-pid')
end
if isempty(logFiles), error('turn_pid:NoFiles','Provide one or more turn-test log files.'); end
if ~isfolder(outputDir), mkdir(outputDir); end
runs = cell(numel(logFiles),1);
allSamples = table; allCommands = table; summary = table;
for fileIndex = 1:numel(logFiles)
    lines = strip(readlines(logFiles(fileIndex)));
    start = []; cfg = []; samples = zeros(0,9); commands = zeros(0,5); ending = [];
    state = '';
    for i = 1:numel(lines)
        line = char(lines(i));
        if startsWith(line,'TURN_TEST ')
            v = sscanf(line,'TURN_TEST ms=%f direction=%f yaw_sign=%f start_cd=%f target_cd=%f');
            if ~isempty(start) || numel(v)~=5, error('turn_pid:BadStart','Each file must contain exactly one test.'); end
            start = v';
        elseif startsWith(line,'TURN_CFG ')
            v = sscanf(line,['TURN_CFG kp_m=%f ki_m=%f kd_far_m=%f kd_near_m=%f switch_cd=%f ' ...
                'integral_m=%f max_rpm=%f accel_m=%f control_ms=%f stable_ms=%f tolerance_cd=%f trim1=%f trim2=%f trim3=%f trim4=%f']);
            if ~isempty(cfg) || numel(v)~=15, error('turn_pid:BadConfig','Invalid or repeated TURN_CFG.'); end
            cfg = v';
        elseif startsWith(line,'TY ')
            v = sscanf(line(4:end),'%f,');
            if numel(v)~=9 || any(~isfinite(v)), error('turn_pid:BadSample','Malformed TY row.'); end
            samples(end+1,:) = v'; %#ok<AGROW>
        elseif startsWith(line,'TC ')
            v = sscanf(line(4:end),'%f,');
            if numel(v)~=5 || any(~isfinite(v)), error('turn_pid:BadCommand','Malformed TC row.'); end
            commands(end+1,:) = v'; %#ok<AGROW>
        elseif startsWith(line,'TURN_END ')
            token = regexp(line,['^TURN_END result=(complete|aborted) ms=(\d+) imu_ms=(\d+) ' ...
                'actual_cd=(-?\d+) error_cd=(-?\d+)$'],'tokens','once');
            if isempty(token) || ~isempty(ending), error('turn_pid:BadEnd','Invalid or repeated TURN_END.'); end
            state = token{1}; ending = str2double(token(2:end));
        end
    end
    if isempty(start) || isempty(cfg) || isempty(ending) || isempty(commands) || size(samples,1)<2
        error('turn_pid:Incomplete','Missing header, samples, commands or end marker in %s.',logFiles(fileIndex));
    end
    if any(~isfinite([start,cfg,ending])) || ~ismember(start(2),[-1,1]) || ~ismember(start(3),[-1,1]) || ...
            any(cfg(1:4)<0) || any(cfg(5:end)<=0) || cfg(9)~=20 || cfg(10)~=200 || cfg(11)~=50
        error('turn_pid:BadConfig','Invalid test metadata or unsupported completion settings.');
    end
    s = array2table(samples,'VariableNames',{'ms','imu_ms','target_cd','actual_cd','error_cd','raw100','ramp100','integral1000','band'});
    c = array2table(commands,'VariableNames',{'ms','wheel1','wheel2','wheel3','wheel4'});
    s.time_s = clockSeconds(s.ms,start(1)); s.sensor_s = clockSeconds(s.imu_ms,start(1));
    c.time_s = clockSeconds(c.ms,start(1));
    if any(diff(s.sensor_s)<=0) || any(diff(s.time_s)<0) || any(diff(c.time_s)<0) || ...
            any(c.time_s<0) || any(~ismember(s.band,[0,1,2])) || ...
            any(abs(angleWrap(s.target_cd/100-start(5)/100))>0.01)
        error('turn_pid:BadTimeline','Nonmonotonic timestamps or inconsistent targets.');
    end
    s.angle_deg = rad2deg(unwrap(deg2rad(s.actual_cd/100)))-start(4)/100;
    s.error_deg = s.error_cd/100; s.raw_rpm = s.raw100/100; s.ramp_rpm = s.ramp100/100;
    s.integral_deg_s = s.integral1000/1000;
    s.age_ms = mod(s.ms-s.imu_ms,2^32);
    s.valid = s.age_ms<=250 & s.time_s>=0 & s.sensor_s<=s.time_s;
    c.turn_rpm = (c.wheel1-c.wheel2-c.wheel3+c.wheel4)/4;
    c.plant_rpm = c.turn_rpm*start(3);
    if any(c{end,{'wheel1','wheel2','wheel3','wheel4'}}~=0)
        error('turn_pid:MissingStop','The final wheel command must be zero.');
    end
    target = 90*start(2)*start(3);
    if abs(angleWrap(start(5)/100-start(4)/100)-target)>0.03
        error('turn_pid:BadTarget','The log is not a signed 90-degree test.');
    end
    kd = repmat(cfg(4)/1000,height(s),1); kd(abs(s.error_deg)>cfg(5)/100)=cfg(3)/1000;
    derivative = [0;angleWrap(diff(s.error_deg))./(diff(s.sensor_s)/0.05)];
    predicted = cfg(1)/1000*s.error_deg + cfg(2)/1000*s.integral_deg_s + kd.*derivative;
    s.replayed_raw_rpm = max(-cfg(7),min(cfg(7),predicted))*start(3);
    s.replay_error_rpm = s.raw_rpm-s.replayed_raw_rpm;
    signOfTravel = start(2)*start(3);
    overshoot = max(0,max(signOfTravel*(s.angle_deg-target)));
    rates = diff(s.angle_deg)./diff(s.sensor_s);
    nz = sign(c.turn_rpm(c.turn_rpm~=0)); reversals = sum(diff(nz)~=0);
    duration = clockSeconds(ending(1),start(1));
    item = table(fileIndex,string(logFiles(fileIndex)),start(2),string(state),height(s),duration, ...
        overshoot,ending(4)/100,max(abs(rates)),max(abs(c.turn_rpm)),reversals, ...
        max(abs(s.replay_error_rpm)),all(s.valid), ...
        'VariableNames',{'run','file','direction','result','samples','duration_s','overshoot_deg', ...
        'final_error_deg','peak_rate_dps','peak_command_rpm','reversals','pid_replay_max_error_rpm','all_samples_valid'});
    summary = [summary;item]; %#ok<AGROW>
    s.run = repmat(fileIndex,height(s),1); c.run = repmat(fileIndex,height(c),1);
    allSamples = [allSamples;s]; allCommands = [allCommands;c]; %#ok<AGROW>
    runs{fileIndex} = struct('Samples',s,'Commands',c,'Config',cfg,'Start',start, ...
        'End',ending,'Target',target,'Complete',strcmp(state,'complete'),'File',logFiles(fileIndex));
end
writetable(allSamples,fullfile(outputDir,'turn-samples.csv'));
writetable(allCommands,fullfile(outputDir,'turn-commands.csv'));
writetable(summary,fullfile(outputDir,'turn-summary.csv'));
fig=figure('Visible','off','Position',[100,100,1100,800]);cleanup=onCleanup(@()close(fig));
layout=tiledlayout(fig,3,1);
for axisIndex=1:3
    ax=nexttile(layout);hold(ax,'on');grid(ax,'on');
    for i=1:numel(runs)
        r=runs{i};polarity=r.Start(2)*r.Start(3);
        if axisIndex==1, plot(ax,r.Samples.sensor_s,polarity*r.Samples.angle_deg,'DisplayName',sprintf('run %d',i));
        elseif axisIndex==2, plot(ax,r.Samples.sensor_s,polarity*r.Samples.error_deg,'DisplayName',sprintf('run %d',i));
        else, stairs(ax,r.Commands.time_s,polarity*r.Commands.plant_rpm,'DisplayName',sprintf('run %d',i)); end
    end
    if axisIndex==1, yline(ax,90,'--');ylabel(ax,'angle toward target deg');legend(ax,'Location','best');
    elseif axisIndex==2, yline(ax,0.5,'--');yline(ax,-0.5,'--');ylabel(ax,'error deg');
    else, ylabel(ax,'turn command RPM');xlabel(ax,'seconds since test start'); end
end
drawnow;
print(fig,fullfile(outputDir,'turn-recording.png'),'-dpng','-r150','-painters');
result=struct('Runs',{runs},'Summary',summary,'Samples',allSamples,'Commands',allCommands);
fprintf('Decoded %d turn tests; %d fresh IMU records, %d command changes.\n',numel(runs),height(allSamples),height(allCommands));
end

function seconds = clockSeconds(value,origin)
seconds=(mod(value-origin+2^31,2^32)-2^31)/1000;
end
function value = angleWrap(value)
value=mod(value+180,360)-180;
end
