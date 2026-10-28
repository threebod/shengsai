function result = simulate_heading_pid(logFile, outputDir)
% Fit on the first RC-1 run, validate on the second, then compare PID candidates.
% Rejected models must not produce a recommended controller or firmware write.
arguments
    logFile (1,1) string
    outputDir (1,1) string = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'build', 'heading-pid-model')
end
decoded = analyze_heading_pid(logFile, outputDir);
t = decoded.Samples;
ids = unique(t.span(t.valid & t.template == 17 & t.sub == 2), 'stable');
if numel(ids) < 2
    error('heading_pid:NeedValidationRun', 'At least two complete RC-1 long runs are needed for separate fitting and validation.');
end
replayError = max(abs(t.pid_replay_error_rpm(t.valid)), [], 'omitnan');
if isempty(replayError) || ~isfinite(replayError) || replayError > 0.05
    error('heading_pid:ReplayMismatch', 'Logged gains/state do not reconstruct the firmware output within 0.05 RPM.');
end
traces = cell(1,numel(ids));
for i = 1:numel(ids), traces{i} = trace(t,ids(i)); end
train = traces{1}; validation = traces{2};
models = table('Size',[0 10], 'VariableTypes',repmat("double",1,10), ...
    'VariableNames',{'delay_samples','a','b','speed_term','bias','condition','train_rmse_deg','validation_rmse_deg','validation_fit_percent','stable_positive_gain'});
for delay = 0:4
    [coeff,condition] = identify(train, delay, true);
    predTrain = predict(train, coeff, delay, true);
    predValidation = predict(validation, coeff, delay, true);
    trainError = rms(predTrain(3:end)-train.y(3:end));
    valError = rms(predValidation(3:end)-validation.y(3:end));
    fit = 100*(1-norm(predValidation(3:end)-validation.y(3:end)) / norm(validation.y(3:end)-mean(validation.y(3:end))));
    admissible = coeff(1)>=0 && coeff(1)<1 && coeff(2)>0;
    models(end+1,:) = {delay,coeff(1),coeff(2),coeff(3),coeff(4),condition,trainError,valError,fit,admissible}; %#ok<AGROW>
end
eligible = find(models.stable_positive_gain == 1);
if isempty(eligible)
    error('heading_pid:NoPhysicalModel','No stable model with the expected yaw-command sign was identified.');
end
% Select using training error only. The second run is not used for selection.
[~,best] = min(models.train_rmse_deg(eligible));
model = models(eligible(best),:);
coeff = [model.a;model.b;model.speed_term;model.bias];
delay = model.delay_samples;
openLoop = predict(validation, coeff, delay, true);
[speedOnly,~] = identify(train, 0, false);
speedPrediction = predict(validation,speedOnly,0,false);
speedOnlyRmse = rms(speedPrediction(3:end)-validation.y(3:end));
baseCfg = decoded.Gains(decoded.Gains(:,1)==ids(1),:);
if isempty(baseCfg), error('heading_pid:MissingConfiguration','The fitting span has no PIDCFG record.'); end
baseline = baseCfg(end,2:4)/1000;
baselineSimulation = simulate(validation,coeff,delay,baseline,baseCfg(end,5:6)/1000);
closedLoopMismatch = rms(baselineSimulation.theta(3:end)-validation.y(3:end));
% Validation thresholds are explicit analysis gates, not physical tolerances.
accepted = model.validation_fit_percent >= 70 && model.validation_rmse_deg <= 0.3 ...
    && closedLoopMismatch <= 0.3 && model.validation_rmse_deg <= 0.8*speedOnlyRmse;
candidates = table('Size',[0 7], 'VariableTypes',repmat("double",1,7), ...
    'VariableNames',{'kp','ki','kd','predicted_rms_deg','predicted_peak_deg','turn_rms_rpm','model_accepted'});
for kp = unique([baseline(1),1,3,4,6,8])
    for ki = unique([baseline(2),0.125,0.5,1])
        for kd = unique([baseline(3),0.06,0.24])
            errors = []; turns = [];
            for i = 1:numel(traces)
                s = simulate(traces{i},coeff,delay,[kp,ki,kd],baseCfg(end,5:6)/1000);
                errors = [errors;s.theta]; turns = [turns;s.u]; %#ok<AGROW>
            end
            candidates(end+1,:) = {kp,ki,kd,rms(errors),max(abs(errors)),rms(turns),accepted}; %#ok<AGROW>
        end
    end
end
candidates = sortrows(candidates,{'predicted_rms_deg','predicted_peak_deg'});
recommended = [];
if accepted, recommended = candidates(1,:); end
bestExploratory = simulate(validation,coeff,delay,candidates{1,1:3},baseCfg(end,5:6)/1000);
fig = figure('Visible','off','Position',[100,100,1100,800]);
closeFigure = onCleanup(@()close(fig));
tiledlayout(fig,3,1);
nexttile;
plot(validation.t,-validation.y,'k-o',validation.t,-openLoop,'--',validation.t,-speedPrediction,':');
ylabel('heading error deg'); grid on; legend('second measured run','model + measured commands','speed-only comparison');
title(sprintf('Held-out run: fit %.1f%%, RMSE %.3f deg',model.validation_fit_percent,model.validation_rmse_deg));
nexttile;
plot(validation.t,-validation.y,'k-o',validation.t,-baselineSimulation.theta,'--',validation.t,-bestExploratory.theta,':');
yline(0.5,'--');yline(-0.5,'--');ylabel('heading error deg');grid on;
legend('measured baseline','simulated baseline','best exploratory candidate');
if accepted, title('Model passed analysis gates; hardware validation still required');
else, title('MODEL REJECTED: candidate curves are exploratory only, not tuning evidence'); end
nexttile;
stairs(validation.t,validation.u);hold on;stairs(validation.t,baselineSimulation.u);stairs(validation.t,bestExploratory.u);
xlabel('time s');ylabel('turn command RPM');grid on;legend('recorded mean command','simulated baseline','exploratory candidate');
exportgraphics(fig,fullfile(outputDir,'model-validation.png'),'Resolution',150);
writetable(models,fullfile(outputDir,'model-delays.csv'));
writetable(candidates,fullfile(outputDir,'exploratory-candidates.csv'));
summary = struct('training_span',ids(1),'validation_span',ids(2), ...
    'model',table2struct(model),'sample_period_s',0.1,'pid_control_period_s',0.02, ...
    'speed_only_validation_rmse_deg',speedOnlyRmse,'baseline_closed_loop_mismatch_deg',closedLoopMismatch, ...
    'firmware_pid_replay_max_error_rpm',replayError,'model_accepted',logical(accepted), ...
    'recommended_pid',recommended,'baseline_pid',baseline, ...
    'note','The model uses 100ms averaged commands; PID simulation uses 20ms integration and 100ms held IMU samples. No parameters were applied.');
writelines(jsonencode(summary,'PrettyPrint',true),fullfile(outputDir,'model-summary.json'));
result = struct('Decoded',decoded,'Models',models,'Candidates',candidates,'Summary',summary);
fprintf('Training span %d, held-out span %d; fit %.2f%%, RMSE %.4f deg.\n',ids(1),ids(2),model.validation_fit_percent,model.validation_rmse_deg);
fprintf('Speed-only RMSE %.4f deg; baseline closed-loop mismatch %.4f deg; accepted=%d.\n',speedOnlyRmse,closedLoopMismatch,accepted);
if ~accepted, fprintf('No PID recommendation: exploratory candidate ranking must not be applied to firmware.\n'); end
end

function s = trace(t,id)
x = t(t.span==id & t.valid,:);
if height(x)<15 || numel(unique(x.yaw_sign))~=1
    error('heading_pid:ShortRun','Each fitting/validation run needs at least 15 valid samples and fixed yaw polarity.');
end
time = mod(x.ms-x.ms(1),2^32)/1000;
if any(diff(time)<=0) || any(diff(time)>0.25)
    error('heading_pid:RecordingGap','Run has nonmonotonic time or gaps over 250ms.');
end
s.t = (0:floor((time(end)+1e-9)/0.1))'*0.1;
s.t(end) = min(s.t(end),time(end));
s.y = interp1(time,-x.error_deg,s.t);
s.u = interp1(time,x.mean_turn_rpm,s.t);
s.v = interp1(time,x.forward_rpm,s.t)/240;
s.scale = interp1(time,x.output_scale,s.t);
s.integral = x.integral_deg_s(1);
s.rate = x.filtered_rate_dps(1);
s.sign = x.yaw_sign(1);
end

function [coeff,condition] = identify(s,delay,withCommand)
dy = diff(s.y);
k = (max(2,delay+1):numel(dy))';
x = [dy(k-1),s.v(k+1),ones(numel(k),1)];
if withCommand, x = [x(:,1),s.u(k+1-delay),x(:,2:end)]; end
condition = cond(x);
if rank(x)<size(x,2), error('heading_pid:InsufficientExcitation','Inputs cannot identify independent model terms.'); end
coeff = x\dy(k);
end

function y = predict(s,coeff,delay,withCommand)
% Only the first two validation positions initialize the state; no refitting.
y = s.y(1:2);
dy = y(2)-y(1);
for k = 2:numel(s.y)-1
    if withCommand
        dy = coeff(1)*dy + coeff(2)*s.u(max(1,k+1-delay)) + coeff(3)*s.v(k+1) + coeff(4);
    else
        dy = coeff(1)*dy + coeff(2)*s.v(k+1) + coeff(3);
    end
    y(k+1,1) = y(k)+dy; %#ok<AGROW>
end
end

function s = simulate(input,coeff,delay,gains,limits)
n = numel(input.t);
s.theta = zeros(n,1);s.theta(1)=input.y(1);s.u=zeros(n,1);
integral=input.integral;rate=input.rate;correction=0;previousYaw=s.theta(1);dy=0;
for k = 2:n
    error = -s.theta(k-1);
    rate = rate + 0.5*((s.theta(k-1)-previousYaw)/0.1-rate);
    previousYaw = s.theta(k-1);
    commands = zeros(5,1);
    for tick = 1:5
        proportional = error;
        if abs(error)<0.3, proportional=0; end
        if proportional~=0 && abs(error)<=8
            candidate=max(-limits(2),min(limits(2),integral+error*0.02));
            raw=gains(1)*proportional+gains(2)*candidate-gains(3)*rate;
            if abs(raw)<=limits(1) || (raw>limits(1) && error<0) || (raw< -limits(1) && error>0)
                integral=candidate;
            end
        end
        raw=gains(1)*proportional+gains(2)*integral-gains(3)*rate;
        raw=max(-limits(1),min(limits(1),raw))*input.sign;
        scale=input.scale(k-1)+(input.scale(k)-input.scale(k-1))*tick/5;
        target=raw*scale;
        correction=correction+max(-1.8,min(1.8,target-correction));
        commands(tick)=sign(correction)*floor(abs(correction)+0.5);
    end
    s.u(k)=mean(commands);
    dy=coeff(1)*dy+coeff(2)*s.u(max(1,k-delay))+coeff(3)*input.v(k)+coeff(4);
    s.theta(k)=s.theta(k-1)+dy;
end
end
