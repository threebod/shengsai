import { buildTimeline, clamp } from './timeline.js';
import { MechanismScene } from './scene.js';
import { parseGroupJson } from './action_json.js';

const $ = id => document.getElementById(id);
const familyNames = { raw: '原料夹取', coarse: '粗加工放置', return: '粗加工回收', return2: '粗加工回收 · 第二轮', storage1: '暂存 · 第一层', storage2: '暂存 · 第二层' };
const kindNames = { move: '机构运动', grip: '夹爪闭合', release: '夹爪松开', platform: '转盘切换', wait: '稳定等待' };
let data, timeline, view;
let family = 'coarse', time = 0, playing = false, lastFrame = 0, activeStep = -1;
let pausedStep = 0;
let notifyTimer;
const edits = new Map(), drafts = new Map();

function notify(message) {
  $('notification').textContent = message;$('notification').hidden = false;
  clearTimeout(notifyTimer);notifyTimer = setTimeout(() => { $('notification').hidden = true; }, 6500);
}
function formatTime(ms) {
  const tenths = Math.floor(ms / 100);
  return `${String(Math.floor(tenths / 600)).padStart(2, '0')}:${String(Math.floor(tenths / 10) % 60).padStart(2, '0')}.${tenths % 10}`;
}
function setPlaying(value) {
  playing = value;
  if (playing) pausedStep = null;
  if (playing && time >= timeline.duration) time = 0;
  $('play').querySelector('span').textContent = playing ? '暂停播放' : '播放动作';
  $('play-icon').innerHTML = playing ? '<path d="M7 5h3v14H7zM14 5h3v14h-3z"/>' : '<path d="m8 5 11 7-11 7z"/>';
  $('play-state').textContent = playing ? '正在回放' : time >= (timeline?.duration || 1) ? '回放完成' : '已暂停';
  $('play-state').classList.toggle('running', playing);
}
function selectedId() {
  return family === 'raw' ? `raw-${$('position').value}-${$('slot').value}` :
    `${family === 'return' && $('round').value === '2' ? 'return2' : family}-${$('order').value}`;
}
function selectGroup() {
  if (!data) return;
  setPlaying(false);time = 0;activeStep = -1;pausedStep = 0;
  const source = data.groups.find(group => group.id === selectedId());
  const group = edits.get(selectedId()) || source;
  if (!group) { notify('当前源码未导出此动作组');return; }
  timeline = buildTimeline(group);
  $('scene-title').textContent = familyNames[group.family] + (edits.has(group.id) ? ' · 本地编辑' : '');
  $('json-group').textContent = group.id;
  $('json-editor').value = drafts.get(group.id) ?? JSON.stringify(group, null, 2);
  $('json-error').hidden = true;
  for (const id of ['json-apply', 'json-export', 'json-reset']) $(id).disabled = false;
  updateJsonStatus();
  $('step-count').textContent = `${group.actions.length} 步`;
  $('scrubber').max = timeline.duration;
  $('scrubber').value = 0;
  $('total-time').textContent = formatTime(timeline.duration);
  $('raw-fields').hidden = family !== 'raw';
  $('order-fields').hidden = family === 'raw';
  $('return-fields').hidden = family !== 'return';
  $('families').querySelectorAll('button').forEach(button => button.classList.toggle('active', button.dataset.family === family));
  $('step-strip').replaceChildren();
  timeline.steps.forEach((step, index) => {
    const button = document.createElement('button');
    button.className = 'step-chip';button.dataset.kind = step.kind;button.dataset.step = index;
    button.textContent = String(index + 1).padStart(2, '0');
    button.title = `${index + 1}. ${step.title} · ${step.detail}`;
    button.setAttribute('aria-label', `跳转第${index + 1}步：${step.title}`);
    button.onclick = () => seek(step.start, index);
    $('step-strip').append(button);
  });
  view?.setTimeline(timeline);
  $('play').disabled = false;
  $('play-state').textContent = '准备就绪';
  update();
}
function updateJsonStatus() {
  const id = selectedId();
  $('json-state').textContent = drafts.has(id) ? '草稿未应用' : edits.has(id) ? '本地已修改' : '源码动作';
}
function readJson() {
  try {
    const source = data.groups.find(group => group.id === selectedId());
    const group = parseGroupJson($('json-editor').value, source);
    $('json-error').hidden = true;
    return group;
  } catch (error) {
    $('json-error').textContent = error.message;$('json-error').hidden = false;
    return null;
  }
}
function seek(target, stepIndex = null) {
  if (!timeline) return;
  setPlaying(false);pausedStep = stepIndex;time = clamp(target, 0, timeline.duration);update();
}
function jump(direction) {
  if (!timeline) return;
  const current = pausedStep ?? timeline.stepAt(time).index;
  const index = clamp(current + direction, 0, timeline.steps.length - 1);
  seek(timeline.steps[index].start, index);
}
function update() {
  if (!timeline) return;
  const state = timeline.stateAt(time), step = pausedStep === null ? timeline.stepAt(time) : timeline.steps[pausedStep];
  $('current-time').textContent = formatTime(time);
  $('scrubber').value = time;
  for (const key of ['h', 'lift', 'turret', 'gripper', 'platform']) $('value-' + key).textContent = state[key].toFixed(1);
  $('lift-fill').style.width = `${clamp(state.lift / 150 * 100, 0, 100)}%`;
  $('lift-pointer').style.left = `${clamp(state.lift / 150 * 100, 0, 100)}%`;
  $('action-progress').style.width = `${clamp((time - step.start) / Math.max(1, step.end - step.start), 0, 1) * 100}%`;
  if (step.index !== activeStep) {
    activeStep = step.index;
    $('current-step').textContent = String(step.index + 1).padStart(2, '0');
    $('action-kind').textContent = kindNames[step.kind];
    $('action-title').textContent = step.title;$('action-detail').textContent = step.detail;
    $('source-step').textContent = `源码步骤 ${step.action.sourceStep + 1}${step.action.approach ? ' · 水平接近阶段' : ''} / 共 ${timeline.group.actions.at(-1).sourceStep + 1} 步`;
    $('step-strip').querySelectorAll('button').forEach((button, index) => {
      button.classList.toggle('active', index === step.index);
      if (index === step.index) button.setAttribute('aria-current', 'step');else button.removeAttribute('aria-current');
    });
    const chip = $('step-strip').children[step.index];
    if (chip) $('step-strip').scrollLeft = Math.max(0, chip.offsetLeft - $('step-strip').offsetLeft - $('step-strip').clientWidth / 2);
  }
  view?.update(state, time, $('show-path').checked);
}
async function load() {
  $('reload').disabled = true;
  if (timeline) setPlaying(false);
  try {
    const response = await fetch('/api/actions', { cache: 'no-store' });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || '读取动作失败');
    data = result;
    $('source-status').textContent = `当前工程 · ${new Date(data.generatedAt).toLocaleTimeString('zh-CN', { hour: '2-digit', minute: '2-digit', hour12: false })} 更新`;
    $('source-hash').textContent = `SOURCE ${data.sourceHash.toUpperCase()}`;
    $('group-count').textContent = `${data.groups.length} 个变体`;
    $('heights-layers').textContent = `${data.heights[0]} / ${data.heights[1]} mm`;
    $('heights-tray').textContent = `${data.heights[2]} / ${data.heights[3]} mm`;
    view?.setData(data);selectGroup();
    if (edits.size || drafts.size) notify('已重读固件；本页 JSON 修改已保留，可点击“恢复源码”还原当前组');
  } catch (error) {
    $('source-status').textContent = '读取失败 · 请检查本地服务';notify(error.message);
  } finally { $('reload').disabled = false; }
}

try { view = new MechanismScene($('viewport')); }
catch (error) {
  $('scene-error').hidden = false;
  $('scene-error').textContent = '三维视图无法启动。请使用启用了图形加速的 Edge 或 Chrome；仍可查看动作步骤和参数。';
  console.error(error);
}
$('families').querySelectorAll('button').forEach(button => button.onclick = () => { family = button.dataset.family;selectGroup(); });
for (const id of ['order', 'position', 'slot', 'round']) $(id).onchange = selectGroup;
$('reload').onclick = load;
$('json-editor').oninput = () => {
  drafts.set(selectedId(), $('json-editor').value);updateJsonStatus();$('json-error').hidden = true;
};
$('json-apply').onclick = () => {
  const group = readJson();
  if (!group) return;
  edits.set(group.id, group);drafts.delete(group.id);selectGroup();
  notify('已应用到本地回放，运动耗时已重算');
};
$('json-export').onclick = () => {
  const group = readJson();
  if (!group) return;
  const blob = new Blob([JSON.stringify(group, null, 2) + '\n'], { type: 'application/json;charset=utf-8' });
  const url = URL.createObjectURL(blob), link = document.createElement('a');
  link.href = url;link.download = `${group.id}.json`;link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
  notify('已导出编辑框中的动作组 JSON');
};
$('json-reset').onclick = () => {
  edits.delete(selectedId());drafts.delete(selectedId());selectGroup();notify('当前动作组已恢复为源码数据');
};
$('play').onclick = () => { if (timeline) setPlaying(!playing); };
$('restart').onclick = () => seek(0, 0);
$('previous').onclick = () => jump(-1);
$('next').onclick = () => jump(1);
$('scrubber').oninput = event => seek(Number(event.target.value));
$('top-view').onclick = () => { view?.view('top');$('top-view').classList.add('selected');$('angle-view').classList.remove('selected'); };
$('angle-view').onclick = () => { view?.view('angle');$('angle-view').classList.add('selected');$('top-view').classList.remove('selected'); };
document.addEventListener('keydown', event => {
  if (event.target.closest('#json-panel') || ['INPUT', 'SELECT', 'TEXTAREA', 'BUTTON'].includes(event.target.tagName)) return;
  if (event.code === 'Space' && timeline) { event.preventDefault();setPlaying(!playing); }
  if (event.code === 'ArrowLeft') { event.preventDefault();jump(-1); }
  if (event.code === 'ArrowRight') { event.preventDefault();jump(1); }
});
document.addEventListener('visibilitychange', () => { if (document.hidden) setPlaying(false); });
function frame(timestamp) {
  const delta = Math.min(100, timestamp - (lastFrame || timestamp));lastFrame = timestamp;
  if (playing && timeline) {
    time += delta * Number($('speed').value);
    if (time >= timeline.duration) {
      if ($('loop').checked) time %= timeline.duration;
      else { time = timeline.duration;setPlaying(false); }
    }
  }
  update();requestAnimationFrame(frame);
}
await load();requestAnimationFrame(frame);
