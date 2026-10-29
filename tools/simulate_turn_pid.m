function result = simulate_turn_pid(logFiles, outputDir)
% Identify yaw dynamics using actual command timestamps, validate on other
% tests, and simulate the firmware's discrete PID/ramp/IMU completion rules.
arguments
    logFiles string
    outputDir (1,1) string = fullfile(fileparts(fileparts(mfilename('fullpath'))),'build','turn-pid-model')
end
decoded=analyze_turn_pid(logFiles,outputDir);runs=decoded.Runs;
directions=cellfun(@(r)r.Start(2),runs);
if sum(directions==1)<2 || sum(directions==-1)<2
    error('turn_pid:NeedValidationRuns','Provide at least two complete tests in each direction; three each is preferred.');
end
cfg=runs{1}.Config;
for i=1:numel(runs)
    r=runs{i};
    if ~r.Complete || ~all(r.Samples.valid) || height(r.Samples)<15 || ...
            any(r.Config~=cfg) || max(abs(r.Samples.replay_error_rpm))>0.15 || ...
            abs(r.End(4)/100)>cfg(11)/100+0.01
        error('turn_pid:RecordingRejected','Use complete, fresh, matching-configuration tests with valid PID replay.');
    end
end
Ts=median(cellfun(@(r)median(diff(r.Samples.sensor_s)),runs));
if Ts<0.02 || Ts>0.25, error('turn_pid:SamplePeriod','Unsupported measured IMU sample period.'); end
train=[find(directions==1,1),find(directions==-1,1)];
validate=setdiff(1:numel(runs),train,'stable');
data=cellfun(@(r)regularize(r,Ts),runs,'UniformOutput',false);
models=zeros(0,7);
for delay=0:4
    X=[];Y=[];
    for i=train
        r=data{i};u=intervalInputs(r,Ts,delay);dy=diff(r.y);
        X=[X;dy(1:end-1),u(3:end),ones(numel(dy)-1,1)];Y=[Y;dy(2:end)]; %#ok<AGROW>
    end
    if rank(X)<3 || cond(X)>1e5, continue; end
    p=X\Y;
    if p(1)<0 || p(1)>=1 || p(2)<=0, continue; end
    model=struct('a',p(1),'b',p(2),'bias',p(3),'delay',delay,'period_s',Ts);
    [trainRmse,~]=predictionError(data,train,model);
    [validationRmse,fit]=predictionError(data,validate,model);
    models(end+1,:)=[delay,p',trainRmse,validationRmse,fit]; %#ok<AGROW>
end
if isempty(models), error('turn_pid:NoModel','No stable positive-gain model could be identified.'); end
modelTable=array2table(models,'VariableNames',{'delay_samples','a','b','bias','training_rmse_deg','validation_rmse_deg','validation_fit_percent'});
[~,best]=min(modelTable.training_rmse_deg);chosen=modelTable(best,:);
model=struct('a',chosen.a,'b',chosen.b,'bias',chosen.bias,'delay',chosen.delay_samples,'period_s',Ts);
baseGains=cfg(1:4)/1000;baseAccel=cfg(8)/1000;
baseSimulation=cell(size(runs));residual=[];
for i=1:numel(runs)
    baseSimulation{i}=closedLoop(runs{i},model,baseGains,baseAccel);
    s=baseSimulation{i};d=data{i};
    predicted=interp1(s.sensor_s,s.angle_deg,d.t,'linear','extrap');
    residual=[residual;predicted-d.y]; %#ok<AGROW>
end
baselineRmse=sqrt(mean(residual.^2));
accepted=chosen.validation_rmse_deg<=0.5 && chosen.validation_fit_percent>=80 && baselineRmse<=0.5;
candidates=zeros(0,11);bestSimulation={};recommended=[];
observedPeak=max(decoded.Summary.peak_command_rpm);
observedRate=max(decoded.Summary.peak_rate_dps);
if accepted
    for kp=baseGains(1)*[0.8,1,1.2]
        for ki=unique(baseGains(2)*[0,0.5,1])
            for dFactor=[0.8,1,1.2]
                for accel=baseAccel*[1,1.2,1.4]
                    gains=[kp,ki,baseGains(3:4)*dFactor];
                    simulations=cellfun(@(r)closedLoop(r,model,gains,accel),runs,'UniformOutput',false);
                    duration=cellfun(@(s)s.completion_s,simulations);
                    overshoot=max(cellfun(@(s)s.overshoot_deg,simulations));
                    finalError=max(cellfun(@(s)abs(s.final_error_deg),simulations));
                    peak=max(cellfun(@(s)s.peak_command_rpm,simulations));
                    rate=max(cellfun(@(s)s.peak_rate_dps,simulations));
                    inDomain=peak<=observedPeak*1.25 && rate<=observedRate*1.25;
                    eligible=all(isfinite(duration)) && overshoot<=1 && finalError<=0.5 && inDomain;
                    candidates(end+1,:)=[gains,accel,mean(duration),overshoot,finalError,peak,inDomain,eligible]; %#ok<AGROW>
                end
            end
        end
    end
end
candidateTable=array2table(candidates,'VariableNames',{'kp','ki','kd_far','kd_near','accel_rpm_s', ...
    'mean_completion_s','max_overshoot_deg','max_final_error_deg','peak_command_rpm','within_observed_domain','eligible'});
safe=find(candidateTable.eligible==1);
if ~isempty(safe)
    [~,k]=min(candidateTable.mean_completion_s(safe));row=candidateTable(safe(k),:);
    recommended=table2struct(row);
    bestSimulation=cellfun(@(r)closedLoop(r,model,row{1,1:4},row.accel_rpm_s),runs,'UniformOutput',false);
end
writetable(modelTable,fullfile(outputDir,'turn-model-delays.csv'));
writetable(candidateTable,fullfile(outputDir,'turn-candidates.csv'));
result=struct('model',model,'training_runs',train,'validation_runs',validate, ...
    'validation_rmse_deg',chosen.validation_rmse_deg,'validation_fit_percent',chosen.validation_fit_percent, ...
    'baseline_closed_loop_rmse_deg',baselineRmse,'model_accepted',accepted,'recommended_pid',recommended, ...
    'note','Offline candidates only. Validate on the vehicle before applying; no firmware parameters were written.');
fid=fopen(fullfile(outputDir,'turn-model-summary.json'),'w');assert(fid>=0);
fwrite(fid,jsonencode(result,PrettyPrint=true),'char');fclose(fid);
fig=figure('Visible','off','Position',[100,100,1100,700]);cleanup=onCleanup(@()close(fig));
tiledlayout(fig,2,1);nexttile;hold on;grid on;
for i=validate
    r=data{i};prediction=freeRun(r,model);polarity=runs{i}.Start(2)*runs{i}.Start(3);
    plot(r.t,r.y*polarity,'DisplayName',sprintf('run %d measured',i));
    plot(r.t,prediction*polarity,'--','DisplayName',sprintf('run %d held-out prediction',i));
end
ylabel('angle toward target deg');legend('Location','best');
nexttile;hold on;grid on;
for i=1:numel(runs)
    r=runs{i};s=baseSimulation{i};polarity=r.Start(2)*r.Start(3);
    plot(r.Samples.sensor_s,r.Samples.angle_deg*polarity,'DisplayName',sprintf('run %d measured',i));
    plot(s.sensor_s,s.angle_deg*polarity,'--','DisplayName',sprintf('run %d baseline simulation',i));
    if ~isempty(bestSimulation)
        b=bestSimulation{i};plot(b.sensor_s,b.angle_deg*polarity,':','DisplayName',sprintf('run %d candidate',i));
    end
end
ylabel('angle toward target deg');xlabel('seconds since test start');legend('Location','best');
print(fig,fullfile(outputDir,'turn-model-validation.png'),'-dpng','-r150');
fprintf('Held-out RMSE %.3f deg, fit %.1f%%; baseline closed-loop RMSE %.3f deg; accepted=%d.\n', ...
    chosen.validation_rmse_deg,chosen.validation_fit_percent,baselineRmse,accepted);
end

function d=regularize(r,Ts)
s=r.Samples;
if max(abs(diff(s.sensor_s)-Ts))>0.3*Ts
    error('turn_pid:IrregularSamples','IMU gaps exceed 30%% of the measured sample period.');
end
t=s.sensor_s(1)+(0:floor((s.sensor_s(end)-s.sensor_s(1)+1e-9)/Ts))'*Ts;
t(end)=min(t(end),s.sensor_s(end));
d=struct('t',t,'y',interp1(s.sensor_s,s.angle_deg,t),'commands',r.Commands);
end

function u=intervalInputs(d,Ts,delay)
u=zeros(size(d.t));
for k=2:numel(u),u(k)=commandMean(d.commands,d.t(k-1)-delay*Ts,d.t(k)-delay*Ts);end
end

function value=commandMean(c,from,to)
t=c.time_s;u=c.plant_rpm;
area=[0;cumsum(diff(t).*u(1:end-1))];
value=(at(to)-at(from))/(to-from);
    function total=at(time)
        k=find(t<=time,1,'last');
        if isempty(k),total=0;else,total=area(k)+u(k)*(time-t(k));end
    end
end

function predicted=freeRun(d,m)
u=intervalInputs(d,m.period_s,m.delay);predicted=d.y;
for k=3:numel(predicted)
    predicted(k)=predicted(k-1)+m.a*(predicted(k-1)-predicted(k-2))+m.b*u(k)+m.bias;
end
end

function [rmse,fit]=predictionError(data,ids,m)
error=[];reference=[];
for i=ids
    d=data{i};p=freeRun(d,m);error=[error;p(3:end)-d.y(3:end)]; %#ok<AGROW>
    reference=[reference;d.y(3:end)-mean(d.y(3:end))]; %#ok<AGROW>
end
rmse=sqrt(mean(error.^2));fit=100*(1-norm(error)/max(norm(reference),eps));
end

function s=closedLoop(r,m,gains,accel)
cfg=r.Config;Ts=m.period_s;control=cfg(9)/1000;age=median(r.Samples.age_ms)/1000;
sensor=r.Samples.sensor_s(1);angle=r.Samples.angle_deg(1);dy=0;
c=struct('time_s',0,'plant_rpm',0);lastCommand=0;
integral=0;previousError=0;previousSample=0;pid=0;pidReady=false;ramp=0;
band=0;stable=0;completed=Inf;
maximumTime=max(18,r.Samples.sensor_s(end)+1);
for now=control:control:maximumTime
    while sensor(end)+Ts<=now+1e-9
        t=sensor(end)+Ts;
        u=commandMean(c,t-Ts-m.delay*Ts,t-m.delay*Ts);
        dy=m.a*dy+m.b*u+m.bias;
        sensor(end+1,1)=t;angle(end+1,1)=angle(end)+dy; %#ok<AGROW>
    end
    k=find(sensor+age<=now+1e-9,1,'last');if isempty(k),k=1;end
    error=mod(r.Target-angle(k)+180,360)-180;
    if isinf(completed)
        if ~band && abs(error)<=cfg(11)/100,band=1;end
        if band && abs(error)>cfg(11)/100,band=0;end
        if ~pidReady || sensor(k)~=previousSample
            scale=1;change=0;
            if pidReady
                scale=(sensor(k)-previousSample)/0.05;
                change=mod(error-previousError+180,360)-180;
            end
            if abs(error)<=cfg(5)/100
                integral=max(-cfg(6)/1000,min(cfg(6)/1000,integral+error*0.05*scale));
            else,integral=0;end
            kd=gains(4);if abs(error)>cfg(5)/100,kd=gains(3);end
            pid=max(-cfg(7),min(cfg(7),gains(1)*error+gains(2)*integral+kd*change/scale))*r.Start(3);
            previousError=error;previousSample=sensor(k);pidReady=true;
        end
        target=pid;if band,target=0;end
        if ramp*target<=0 || abs(target)<abs(ramp)
            if ramp*target<0,ramp=0;
            elseif abs(target)<abs(ramp),ramp=target;end
        end
        ramp=ramp+max(-accel*control,min(accel*control,target-ramp));command=round(ramp);
        if band && command==0
            if band==1,stable=now;band=2;end
            if now-stable>=cfg(10)/1000-1e-9 && sensor(k)-stable>=cfg(10)/1000-1e-9,completed=now;end
        end
    else,command=0;end
    if command~=lastCommand
        c.time_s(end+1,1)=now;c.plant_rpm(end+1,1)=mean(fix(command*cfg(12:15)/1000))*r.Start(3);lastCommand=command;
    end
    if isfinite(completed) && now>=max(completed+2*Ts,r.Samples.sensor_s(end)+Ts),break;end
end
polarity=r.Start(2)*r.Start(3);
finalIndex=find(sensor+age<=completed+1e-9,1,'last');if isempty(finalIndex),finalIndex=numel(angle);end
s=struct('sensor_s',sensor,'angle_deg',angle,'completion_s',completed, ...
    'overshoot_deg',max(0,max(polarity*(angle-r.Target))), ...
    'final_error_deg',r.Target-angle(finalIndex),'peak_command_rpm',max(abs(c.plant_rpm)), ...
    'peak_rate_dps',max(abs(diff(angle)/Ts)));
end
