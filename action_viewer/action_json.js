function number(value, path, low, high, step = 1) {
  if (typeof value !== 'number' || !Number.isFinite(value) || value < low || value > high ||
      Math.abs(value / step - Math.round(value / step)) > 0.000001)
    throw new Error(`${path} 必须为 ${low}～${high} 的数值，步长 ${step}`);
}
function pose(value, path) {
  if (!value || typeof value !== 'object') throw new Error(`${path} 缺少姿态对象`);
  for (const [key, low, high, step] of [
    ['h', -122, 85, 0.1], ['lift', 0, 150, 0.1], ['turret', 0, 360, 0.1],
    ['horizontalRpm', 10, 1200, 1], ['horizontalAccel', 1, 200, 1],
    ['liftRpm', 10, 2000, 1], ['liftAccel', 1, 240, 1], ['turretDps', 1, 220, 0.1],
  ]) number(value[key], `${path}.${key}`, low, high, step);
}

// 与 mechanism_action.h 的单精度计算及 turret_motion.h 的 20ms 向上取整一致。
function motorMs(pulses, rpm, accel) {
  if (!pulses) return 0;
  const f = Math.fround;
  const cruise = f(f(pulses * 60000) / f(3200 * rpm));
  const ramp = f(rpm * (256 - accel) * f(0.05));
  return Math.ceil(cruise >= ramp ? f(cruise + ramp) : f(2 * f(Math.sqrt(f(cruise * ramp)))));
}
function turretMs(from, to, dps) {
  return Math.ceil(Math.abs(Math.round(to * 10) - Math.round(from * 10)) * 100 * 15 / (Math.round(dps * 10) * 16)) * 20;
}

export function parseGroupJson(text, source) {
  let group;
  try { group = JSON.parse(text); }
  catch (error) { throw new Error(`JSON 语法错误：${error.message}`); }
  if (!group || typeof group !== 'object' || Array.isArray(group)) throw new Error('JSON 顶层必须是一个动作组对象');
  // 动作库选择器仍由当前工程提供，避免编辑后身份与左侧选择不一致。
  for (const key of ['id', 'family', 'layer', 'position', 'slot', 'order'])
    if (JSON.stringify(group[key]) !== JSON.stringify(source[key])) throw new Error(`${key} 是动作组标识，请保留原值；可修改 initial、servo 和 actions`);
  pose(group.initial?.pose, 'initial.pose');
  for (const key of ['gripper', 'platform']) number(group.initial[key], `initial.${key}`, 0, 360);
  number(group.initial.slot, 'initial.slot', 1, 3);
  if (!group.servo || typeof group.servo !== 'object') throw new Error('缺少 servo 对象');
  for (const key of ['open', 'close']) number(group.servo[key], `servo.${key}`, 0, 360);
  if (group.servo.open <= group.servo.close) throw new Error('servo.open 必须大于 servo.close');
  for (const key of ['gripperDps', 'platformDps']) number(group.servo[key], `servo.${key}`, 0.1, 1000, 0.1);
  if (!Array.isArray(group.servo.platformAngles) || group.servo.platformAngles.length !== 3) throw new Error('servo.platformAngles 必须包含三个角度');
  group.servo.platformAngles.forEach((angle, i) => number(angle, `servo.platformAngles[${i}]`, 0, 360));
  if (!Array.isArray(group.actions) || group.actions.length < 1 || group.actions.length > 500) throw new Error('actions 必须包含 1～500 个动作');
  let current = { ...group.initial.pose }, holding = group.initial.gripper === group.servo.close;
  group.actions.forEach((action, index) => {
    const path = `actions[${index}]`;
    if (!action || !['pose', 'gripper', 'platform', 'servo', 'wait'].includes(action.type)) throw new Error(`${path}.type 不支持`);
    number(action.waitMs, `${path}.waitMs`, 0, 65535);
    action.sourceStep ??= index;
    action.approach ??= false;
    number(action.sourceStep, `${path}.sourceStep`, 0, 65535);
    if (typeof action.approach !== 'boolean') throw new Error(`${path}.approach 必须为布尔值`);
    action.motionMs = { h: 0, lift: 0, turret: 0 };
    if (action.type === 'pose') {
      pose(action.pose, `${path}.pose`);
      const p = action.pose;
      const dh = Math.round(p.h * 10) - Math.round(current.h * 10);
      const dl = Math.round(p.lift * 10) - Math.round(current.lift * 10);
      action.motionMs = {
        h: motorMs(Math.floor((Math.abs(dh) * 25465 + 5000) / 10000), p.horizontalRpm, p.horizontalAccel) + (dh ? 50 : 0),
        lift: motorMs(Math.abs(dl) * 8, p.liftRpm, p.liftAccel),
        turret: turretMs(current.turret, p.turret, holding ? p.turretDps : 250),
      };
      current = { ...p };
    } else if (action.type === 'gripper') {
      number(action.value, `${path}.value`, 0, 1);
      holding = !action.value;
    } else if (action.type === 'platform') number(action.value, `${path}.value`, 1, 3);
    else if (action.type === 'servo') {
      if (![2, 3, 4].includes(action.channel)) throw new Error(`${path}.channel 只支持 2、3、4`);
      number(action.value, `${path}.value`, 0, 360);
      if (action.channel === 4) current.turret = action.value;
      if (action.channel === 2) holding = action.value === group.servo.close;
    }
  });
  return group;
}
