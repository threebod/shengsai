export const clamp = (value, low, high) => Math.max(low, Math.min(high, value));
const quintic = t => t * t * t * (10 + t * (-15 + 6 * t));
const linear = t => t;

function axisCurve(duration, rpm, accel, margin = 0) {
  const movement = Math.max(1, duration - margin);
  const ramp = Math.min(0.5, rpm * (256 - accel) * 0.05 / movement);
  return fraction => {
    const t = clamp(fraction * duration / movement, 0, 1);
    if (ramp < 0.00001) return t;
    if (t < ramp) return t * t / (2 * ramp * (1 - ramp));
    if (t > 1 - ramp) return 1 - (1 - t) ** 2 / (2 * ramp * (1 - ramp));
    return (t - ramp / 2) / (1 - ramp);
  };
}

export function describeAction(action, group) {
  if (action.approach) return { title: '先水平接近物料', kind: 'move', detail: '下层物料互锁：先平移，再下降' };
  if (action.type === 'pose') {
    const p = action.pose;
    return { title: p.lift > 0 ? '移动至工作姿态' : '抬升 / 转向 / 平移', kind: 'move',
      detail: `水平 ${p.h.toFixed(1)} mm · 向下 ${p.lift.toFixed(1)} mm · 云台 ${p.turret.toFixed(1)}°` };
  }
  if (action.type === 'gripper') return { title: action.value ? '松开夹爪' : '闭合夹爪',
    kind: action.value ? 'release' : 'grip', detail: `${action.value ? group.servo.open : group.servo.close}°${action.waitMs ? ` · 稳定等待 ${action.waitMs} ms` : ''}` };
  if (action.type === 'platform') return { title: `转盘切换至 ${action.value} 号槽`, kind: 'platform',
    detail: `指令角度 ${group.servo.platformAngles[action.value - 1]}°` };
  if (action.type === 'wait') return { title: '稳定等待', kind: 'wait', detail: `${action.waitMs} ms` };
  return { title: `舵机 ${action.channel} 动作`, kind: 'platform', detail: `${action.value}°` };
}

export function buildTimeline(group) {
  const initial = { h: group.initial.pose.h, lift: group.initial.pose.lift,
    turret: group.initial.pose.turret, gripper: group.initial.gripper, platform: group.initial.platform };
  const tracks = Object.fromEntries(Object.keys(initial).map(key => [key, []]));
  const steps = [];
  const events = [{ time: 0, type: 'slot', slot: group.initial.slot }];
  let now = 0;
  let platformEnd = 0;
  let previousAction = null;

  function valueAt(key, time) {
    let value = initial[key];
    for (const track of tracks[key]) {
      if (time < track.start) break;
      if (time >= track.end) value = track.to;
      else return track.from + (track.to - track.from) * track.curve((time - track.start) / (track.end - track.start));
    }
    return value;
  }
  function stateAt(time) {
    const t = clamp(time, 0, duration);
    const result = Object.fromEntries(Object.keys(initial).map(key => [key, valueAt(key, t)]));
    result.slot = group.initial.slot;
    for (const event of events) if (event.type === 'slot' && event.time <= t) result.slot = event.slot;
    return result;
  }
  function add(key, to, start, ms, curve = linear) {
    const from = valueAt(key, start);
    tracks[key].push({ start, end: start + ms, from, to, curve });
    return start + ms;
  }
  for (const action of group.actions) {
    const before = Object.fromEntries(Object.keys(initial).map(key => [key, valueAt(key, now)]));
    if (action.type === 'pose' && [-50, -45, -47].includes(action.pose.h) && action.pose.turret === 120 && action.pose.lift > before.lift)
      now = Math.max(now, platformEnd);
    if (action.type === 'gripper' && action.value && before.turret === 120)
      now = Math.max(now, platformEnd);
    const start = now;
    const semantic = describeAction(action, group);
    if (action.type === 'pose') {
      const p = action.pose;
      now = Math.max(add('h', p.h, start, action.motionMs.h,
        axisCurve(action.motionMs.h, p.horizontalRpm, p.horizontalAccel, action.motionMs.h ? 50 : 0)),
      add('lift', p.lift, start, action.motionMs.lift, axisCurve(action.motionMs.lift, p.liftRpm, p.liftAccel)),
      add('turret', p.turret, start, action.motionMs.turret, quintic));
    } else if (action.type === 'gripper') {
      const angle = action.value ? group.servo.open : group.servo.close;
      const ms = Math.ceil(Math.abs(angle - before.gripper) / group.servo.gripperDps * 50) * 20;
      now = add('gripper', angle, start, ms);
      events.push({ time: now, type: action.value ? 'open' : 'close', pose: { ...before }, slot: 0 });
    } else if (action.type === 'platform') {
      const delayed = group.family === 'raw' && previousAction?.type === 'gripper' && previousAction.value;
      const turnAt = Math.max(start + (delayed ? 500 : 0), platformEnd);
      const from = valueAt('platform', turnAt);
      const angle = group.servo.platformAngles[action.value - 1];
      const ms = Math.ceil(Math.abs(angle - from) / group.servo.platformDps * 50) * 20;
      platformEnd = add('platform', angle, turnAt, ms);
      events.push({ time: turnAt, type: 'slot', slot: action.value });
      if (action.waitMs) now = platformEnd;
    } else if (action.type === 'servo') {
      const key = { 2: 'gripper', 3: 'platform', 4: 'turret' }[action.channel];
      const speed = action.channel === 2 ? group.servo.gripperDps : action.channel === 3 ? group.servo.platformDps : group.initial.pose.turretDps;
      now = add(key, action.value, start, Math.ceil(Math.abs(action.value - before[key]) / speed * 50) * 20, key === 'turret' ? quintic : linear);
    }
    now += action.waitMs;
    steps.push({ ...semantic, start, end: now, action, index: steps.length });
    previousAction = action;
  }
  const duration = Math.max(now, platformEnd, 1);
  events.sort((a, b) => a.time - b.time);
  return { group, initial, tracks, steps, events, duration, stateAt,
    stepAt(time) {
      let selected = steps[0];
      for (const step of steps) if (step.start <= time) selected = step;
      return selected;
    }
  };
}
