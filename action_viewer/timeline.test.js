import test from 'node:test';
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { buildTimeline } from './timeline.js';

const dataset = JSON.parse(execFileSync('python', [fileURLToPath(new URL('./server.py', import.meta.url)), '--export-only'],
  { encoding: 'utf8', env: { ...process.env, PYTHONIOENCODING: 'utf-8' } }));

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
