import test from 'node:test';
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { buildTimeline } from './timeline.js';
import { parseGroupJson } from './action_json.js';
import { toolPoint, armMount, trayCenter, armRotation } from './kinematics.js';

const dataset = JSON.parse(execFileSync('python', [fileURLToPath(new URL('./server.py', import.meta.url)), '--export-only'],
  { encoding: 'utf8', env: { ...process.env, PYTHONIOENCODING: 'utf-8' } }));

test('JSON round trips all firmware groups and recalculates the same production motion durations', () => {
  for (const source of dataset.groups) {
    const group = parseGroupJson(JSON.stringify(source), source);
    assert.deepEqual(group, source, source.id);
  }
});

test('editing JSON recalculates movement after changed poses, speeds and direct turret commands', () => {
  const source = dataset.groups.find(g => g.id === 'coarse-231');
  const draft = structuredClone(source);
  draft.initial.pose.turret = 260; // Fix the test's 40-degree move independently of current calibration.
  draft.actions = [structuredClone(source.actions.find(a => a.type === 'pose'))];
  Object.assign(draft.actions[0].pose, { h: 20, lift: 40, turret: 300 });
  draft.actions[0].motionMs = { h: 99999, lift: 99999, turret: 99999 };
  const first = parseGroupJson(JSON.stringify(draft), source);
  assert.equal(first.actions[0].motionMs.turret, 300);
  assert.notEqual(first.actions[0].motionMs.h, 99999);
  draft.actions[0].pose.horizontalRpm = 10;
  const slow = parseGroupJson(JSON.stringify(draft), source);
  assert.ok(slow.actions[0].motionMs.h > first.actions[0].motionMs.h);
  const final = buildTimeline(slow).stateAt(Infinity);
  assert.equal(final.h, 20);assert.equal(final.lift, 40);assert.equal(final.turret, 300);
  draft.actions.unshift({ type: 'servo', channel: 4, value: 300, waitMs: 0 });
  const direct = parseGroupJson(JSON.stringify(draft), source);
  assert.equal(direct.actions[1].motionMs.turret, 0);
});

test('JSON rejects malformed and unsafe timeline inputs without changing the source', () => {
  const source = dataset.groups.find(g => g.id === 'raw-1-1');
  assert.throws(() => parseGroupJson('{', source), /JSON 语法错误/);
  assert.throws(() => parseGroupJson('null', source), /顶层/);
  const original = JSON.stringify(source);
  for (const modify of [
    g => { g.actions = []; },
    g => { g.initial.pose.lift = 151; },
    g => { g.initial.pose.h = '10'; },
    g => { g.servo.gripperDps = 0; },
    g => { g.actions[0] = { type: 'servo', channel: 9, value: 10, waitMs: 0 }; },
    g => { g.actions[0] = { type: 'platform', value: 4, waitMs: 0 }; },
    g => { g.actions[0] = { type: 'gripper', value: 3, waitMs: 0 }; },
    g => { g.actions[0].waitMs = -1; },
    g => { g.id = 'other'; },
  ]) {
    const draft = structuredClone(source);modify(draft);
    assert.throws(() => parseGroupJson(JSON.stringify(draft), source));
  }
  assert.equal(JSON.stringify(source), original);
});

test('right-side arm, rear tray and observation direction agree with counterclockwise turret rotation', () => {
  assert.ok(armMount.x > 124);assert.ok(trayCenter.z > 168.5);assert.equal(trayCenter.x, 0);
  const observe = toolPoint({ h: 0, lift: 0, turret: 260 });
  assert.equal(observe.x, armMount.x + 220);assert.equal(observe.z, armMount.z);
  const rotated = toolPoint({ h: 0, lift: 0, turret: 350 });
  assert.ok(Math.abs(rotated.x - armMount.x) < 1e-9);
  assert.equal(rotated.z, armMount.z - 220);
  for (const turret of [0, 53, 120, 219.5, 260, 295.8, 360]) {
    const point = toolPoint({ h: 10, lift: 50, turret });
    assert.ok(Math.abs(point.x - (armMount.x - 230 * Math.sin(armRotation(turret)))) < 1e-9);
    assert.ok(Math.abs(point.z - (armMount.z - 230 * Math.cos(armRotation(turret)))) < 1e-9);
    assert.equal(point.y, 150);
  }
  const trayPickup = toolPoint({ h: -50, lift: 0, turret: 120 });
  assert.ok(Math.hypot(trayPickup.x - trayCenter.x, trayPickup.z - (trayCenter.z - 40)) < 1);
});

test('all 39 firmware variants export unique ids, valid timelines and exact final targets', () => {
  assert.equal(dataset.groups.length, 39);
  assert.equal(new Set(dataset.groups.map(g => g.id)).size, 39);
  for (const group of dataset.groups) {
    const timeline = buildTimeline(group);
    assert.ok(timeline.duration > 0 && Number.isFinite(timeline.duration), group.id);
    for (const step of timeline.steps) assert.ok(step.end >= step.start && step.start >= 0, group.id);
    for (let sample = 0; sample <= 50; sample++) {
      const state = timeline.stateAt(timeline.duration * sample / 50);
      for (const value of Object.values(state)) assert.ok(Number.isFinite(value), group.id);
      assert.ok(state.lift >= 0 && state.lift <= 150, group.id);
    }
    const pose = group.actions.filter(a => a.type === 'pose').at(-1).pose;
    const final = timeline.stateAt(timeline.duration);
    for (const key of ['h', 'lift', 'turret']) assert.equal(final[key], pose[key], `${group.id} ${key}`);
    assert.deepEqual(timeline.stateAt(-10), timeline.stateAt(0));
    assert.deepEqual(timeline.stateAt(timeline.duration + 1000), final);
  }
});

test('lower pickup has a horizontal approach before lift descent; upper pickups do not', () => {
  for (const group of dataset.groups.filter(g => g.family === 'raw')) {
    const approaches = group.actions.filter(a => a.approach);
    assert.equal(approaches.length, group.position === 1 ? 1 : 0, group.id);
    if (approaches.length) {
      const index = group.actions.indexOf(approaches[0]);
      assert.equal(approaches[0].pose.lift, 0);
      assert.equal(group.actions[index + 1].pose.h, approaches[0].pose.h);
      assert.ok(group.actions[index + 1].pose.lift > 0);
    }
  }
});

test('raw tray switching is delayed 500ms and overlaps the return motion', () => {
  const group = dataset.groups.find(g => g.id === 'raw-2-1');
  const timeline = buildTimeline(group);
  const release = timeline.events.filter(event => event.type === 'open').at(-1);
  const turn = timeline.tracks.platform.at(-1);
  assert.equal(turn.start, release.time + 500);
  const returnMove = timeline.steps.find(step => step.start >= release.time && step.action.type === 'pose');
  assert.ok(returnMove.start < turn.start);
  assert.ok(timeline.duration >= turn.end);
});

test('gripper closure retains its settling wait and turret motion stays monotonic without angle wrapping', () => {
  for (const group of dataset.groups) {
    const timeline = buildTimeline(group);
    for (const step of timeline.steps.filter(s => s.action.type === 'gripper' && !s.action.value)) {
      const event = timeline.events.find(event => event.type === 'close' && event.time >= step.start);
      assert.equal(step.end, event.time + step.action.waitMs);
    }
    for (const track of timeline.tracks.turret) {
      if (track.end === track.start) continue;
      let previous = track.from;
      for (let i = 0; i <= 20; i++) {
        const value = track.from + (track.to - track.from) * track.curve(i / 20);
        assert.ok(value >= Math.min(track.from, track.to) - 1e-8 && value <= Math.max(track.from, track.to) + 1e-8);
        assert.ok(track.to >= track.from ? value >= previous - 1e-8 : value <= previous + 1e-8);
        previous = value;
      }
      assert.ok(track.curve(0.1) < 0.1 && track.curve(0.9) > 0.9);
    }
  }
});
