(() => {
    'use strict';
    const $ = id => document.getElementById(id);
    const data = window.ROUTE_DATA;
    const controls = ['zone', 'from', 'to', 'obstacle-a', 'obstacle-b', 'play', 'reset', 'progress', 'full', 'speed'];
    if (!data) {
        $('empty').hidden = false;
        $('empty').textContent = '尚未导出路线。请运行 tools/start-route-viewer.ps1，生成当前固件路线后重新打开。';
        $('source-hash').textContent = '未加载';
        controls.forEach(id => $(id).disabled = true);
        return;
    }
    const names = ['启停区 · 出发', '扫码区', '原料区 · 第一轮', '粗加工区 · 第一轮', '暂存区 · 第一轮',
                   '原料区 · 第二轮', '粗加工区 · 第二轮', '暂存区 · 第二轮', '启停区 · 返回'];
    const tasks = [1, 3, 6, 8, 10, 13, 15, 19];
    const headingName = h => ({0: '上', 1: '左', 2: '下', '-1': '右'})[h];
    const change = (a, b) => { let d = b - a; if (d > 2) d -= 4; if (d < -2) d += 4; return d; };
    const rad = h => h * Math.PI / 2;
    const svgNS = 'http://www.w3.org/2000/svg';
    function svg(tag, attributes, parent) {
        const node = document.createElementNS(svgNS, tag);
        Object.entries(attributes).forEach(([key, value]) => node.setAttribute(key, value));
        if (parent) parent.append(node);
        return node;
    }
    function text(x, y, value, className = 'station-label', anchor = 'middle') {
        const node = svg('text', {x, y: 2400 - y, class: className, 'text-anchor': anchor}, $('labels'));
        node.textContent = value;
    }
    let route = null, all = [], selected = [], total = 0, elapsed = 0, playing = false, lastTime = 0;
    let car, basePath, selectedPath, donePath, stepNodes = [], activeIndex = -1;
    const option = (value, name) => new Option(name, value);
    names.slice(0, -1).forEach((name, i) => $('from').add(option(i, name)));
    for (let i = 1; i <= 15; i++) {
        $('obstacle-a').add(option(i, `${i} 号区域`));
        $('obstacle-b').add(option(i, `${i} 号区域`));
    }
    $('from').value = '2';
    $('obstacle-a').value = '7';
    $('obstacle-b').value = '9';
    $('source-hash').textContent = data.sourceHash;
    $('generated').textContent = `路线生成于 ${data.generatedAt.replace('T', ' ').slice(0, 19)}`;

    function destinations() {
        const old = Number($('to').value);
        const start = Number($('from').value);
        $('to').replaceChildren();
        names.forEach((name, i) => { if (i > start) $('to').add(option(i, name)); });
        $('to').value = String(old > start ? old : start + 1);
    }

    function makeTimeline(legs) {
        const result = [];
        let pose = [...legs[0].from];
        function add(kind, points, duration, legIndex, label, detail) {
            const stage = tasks.indexOf(legs[legIndex].task) + 1 || 1;
            result.push({kind, points, duration, legIndex, stage, label, detail, template: legs[legIndex].template});
            pose = [...points[points.length - 1]];
        }
        function turn(target, i) {
            const d = change(pose[2], target);
            if (!d) return;
            add('turn', [[...pose], [pose[0], pose[1], pose[2] + d]], Math.abs(d) * .85, i,
                `原地${d > 0 ? '左' : '右'}转 ${Math.abs(d) * 90}°`, `车头朝${headingName(target)} · 已通过固件转向检查`);
            pose[2] = target;
        }
        legs.forEach((leg, i) => {
            turn(leg.departure, i);
            const dx = leg.to[0] - leg.from[0], dy = leg.to[1] - leg.from[1];
            const length = Math.hypot(dx, dy), ux = dx / length, uy = dy / length;
            const end = [leg.to[0] - (leg.arc ? ux * data.trim : 0),
                         leg.to[1] - (leg.arc ? uy * data.trim : 0), leg.from[2]];
            const movement = dx > 0 ? -1 : dx < 0 ? 1 : dy > 0 ? 0 : 2;
            const difference = Math.abs(change(leg.from[2], movement));
            const kind = difference === 2 ? 'reverse' : difference === 1 ? 'lateral' : 'forward';
            const label = {reverse: '倒车', lateral: '横移', forward: '前进'}[kind];
            const lineLength = Math.hypot(end[0] - pose[0], end[1] - pose[1]);
            add(kind, [[...pose], end], lineLength / 350, i,
                `${label} · 地图向${headingName(movement)}`, `直线指令 ${Math.abs(leg.up || leg.right)} mm · 名义 ${Math.round(lineLength)} mm`);
            if (leg.arc) {
                const d = change(leg.from[2], legs[i + 1].from[2]);
                const angle = rad(movement), s = Math.sin(angle), c = Math.cos(angle);
                const points = data.arc.map(([x, y, degrees]) => {
                    const localX = x * d;
                    return [leg.to[0] + localX * c - y * s, leg.to[1] + localX * s + y * c,
                            leg.from[2] + d * degrees / 90];
                });
                const arcType = leg.experimentalArc ? ' · 7/13实验弯' : leg.fourPointArc ? ' · 四点版' : '';
                const checks = leg.experimentalArc ? ' · 未做车体扫掠检查' :
                    leg.fourPointArc ? ' · 检查边界及所选障碍，未检查固定区' : '';
                add('arc', points, 1.65, i, `${kind === 'reverse' ? '倒车' : '前进'}连续${d > 0 ? '左' : '右'}转 90°${arcType}`,
                    `225 mm 基准半径 · 缓变曲率 · 两端各裁切 ${data.trim} mm${checks}`);
            } else {
                turn(leg.heading, i);
            }
        });
        return result;
    }

    function drawField() {
        const field = $('field');
        field.replaceChildren(); $('labels').replaceChildren();
        svg('rect', {width: 2400, height: 2400, fill: '#fafcf8', stroke: '#b4c5b4', 'stroke-width': 6}, field);
        svg('rect', {width: 2400, height: 2400, fill: 'url(#grid)'}, field);
        data.regions.forEach((r, i) => {
            const selected = [Number($('obstacle-a').value), Number($('obstacle-b').value)].includes(i + 1);
            svg('rect', {x: r[0], y: r[1], width: r[2] - r[0], height: r[3] - r[1], class: `region${selected ? ' selected' : ''}`}, field);
            text((r[0] + r[2]) / 2, (r[1] + r[3]) / 2 - 13, String(i + 1), 'region-label');
        });
        data.fixed.forEach(r => svg('rect', {x: r[0], y: r[1], width: r[2] - r[0], height: r[3] - r[1], rx: 8, class: 'fixed'}, field));
        [[1200, 2080], [1200, 400], [400, 1200], [2100, 1200]].forEach(([x, y]) => {
            svg('circle', {cx: x, cy: y, r: 32, fill: '#fff', stroke: '#719a77', 'stroke-width': 7}, field);
        });
        [[2100, 2100, '1'], [2100, 0, '2']].forEach(([x, y, name]) => {
            svg('rect', {x, y, width: 300, height: 300, fill: '#e9f1df', stroke: '#a7bd94', 'stroke-width': 3}, field);
            text(x + 150, y + 80, `启停 ${name}`, 'station-label');
        });
        text(1200, 2210, '原料区'); text(1200, 225, '粗加工区');
        text(400, 1400, '暂存区'); text(2150, 1410, '扫码区');
        for (let v = 0; v <= 2400; v += 600) {
            text(v, -80, String(v), 'axis-label'); text(-50, v - 10, String(v), 'axis-label', 'end');
        }
        basePath = svg('path', {class: 'path-base'}, field);
        selectedPath = svg('path', {class: 'path-selected'}, field);
        donePath = svg('path', {class: 'path-done'}, field);
        car = svg('g', {filter: 'url(#car-shadow)'}, field);
        svg('rect', {x: -150, y: -150, width: data.body, height: data.body, rx: 30, fill: '#fff', stroke: '#326647', 'stroke-width': 9}, car);
        [[-166, -95], [-166, 65], [132, -95], [132, 65]].forEach(([x, y]) => svg('rect', {x, y, width: 34, height: 60, rx: 12, fill: '#445c4b'}, car));
        svg('rect', {x: -104, y: -100, width: 208, height: 198, rx: 22, fill: '#e2eedf'}, car);
        svg('path', {d: 'M 0 99 L -43 32 L -14 32 L -14 -43 L 14 -43 L 14 32 L 43 32 Z', fill: '#2f764c'}, car);
        svg('circle', {cx: 0, cy: -80, r: 12, fill: '#afc7a9'}, car);
    }

    function path(motions) {
        return motions.filter(m => m.kind !== 'turn').map(m => m.points.map((p, i) => `${i ? 'L' : 'M'}${p[0].toFixed(2)},${p[1].toFixed(2)}`).join(' ')).join(' ');
    }

    function setPlaying(value) {
        playing = value; lastTime = 0;
        $('play').textContent = value ? 'Ⅱ 暂停' : '▶ 播放';
        $('play').setAttribute('aria-label', value ? '暂停路线' : '播放路线');
    }

    function update() {
        setPlaying(false); elapsed = 0; activeIndex = -1;
        const a = Number($('obstacle-a').value), b = Number($('obstacle-b').value);
        route = a !== b ? data.routes[`${$('zone').value}-${Math.min(a, b)}-${Math.max(a, b)}`] : null;
        drawField(); $('steps').replaceChildren(); stepNodes = [];
        const start = Number($('from').value), end = Number($('to').value);
        $('route-title').textContent = `${names[start]} → ${names[end]}`;
        $('route-template').textContent = '';
        const experimental = route && route.some(leg => leg.experimentalArc);
        $('route-status').classList.toggle('error', !route || experimental);
        $('route-status').textContent = route ? `${a} / ${b} 号障碍 · 固件已生成路线${experimental ? ' · 含7/13实验弯，未做该弯车体扫掠检查' : ''}` : a === b ? '请选择两个不同的障碍区域。' : '固件拒绝此组合：完整比赛路线不可行。请更换障碍位置。';
        ['play', 'reset', 'progress', 'speed'].forEach(id => $(id).disabled = !route);
        if (!route) {
            all = selected = []; total = 0;
            car.setAttribute('visibility', 'hidden');
            $('empty').hidden = false;
            $('empty').textContent = a === b ? '障碍 A 与 B 不能位于同一区域' : '当前组合无法生成完整比赛路线';
            $('distance').textContent = $('arc-count').textContent = '—';
            $('progress').value = 0; $('percent').textContent = '0%';
            $('motion').textContent = '等待可行路线';
            return;
        }
        $('empty').hidden = true;
        all = makeTimeline(route);
        selected = all.filter(m => m.stage > start && m.stage <= end);
        $('route-template').textContent = `固定模板：${[...new Set(selected.map(m => m.template))].join(' → ')}`;
        total = selected.reduce((sum, m) => sum + m.duration, 0);
        basePath.setAttribute('d', path(all)); selectedPath.setAttribute('d', path(selected));
        const length = selected.reduce((sum, m) => sum + m.points.slice(1).reduce((s, p, i) => s + Math.hypot(p[0] - m.points[i][0], p[1] - m.points[i][1]), 0), 0);
        $('distance').textContent = (length / 1000).toFixed(2);
        $('arc-count').textContent = selected.filter(m => m.kind === 'arc').length;
        selected.forEach((m, i) => {
            const row = document.createElement('li'); row.className = m.kind; row.dataset.index = String(i + 1).padStart(2, '0');
            row.tabIndex = 0; row.setAttribute('role', 'button');
            row.append(document.createTextNode(m.label));
            const detail = document.createElement('small'); detail.textContent = `${m.template} · ${m.detail}`; row.append(detail);
            row.title = `${names[m.stage - 1]} → ${names[m.stage]}`;
            const seek = () => { setPlaying(false); elapsed = selected.slice(0, i).reduce((sum, item) => sum + item.duration, 0); render(); };
            row.onclick = seek;
            row.onkeydown = event => { if (event.key === 'Enter' || event.key === ' ') { event.preventDefault(); seek(); } };
            $('steps').append(row); stepNodes.push(row);
        });
        render();
    }

    function render() {
        if (!selected.length) return;
        let time = elapsed, index = 0;
        while (index < selected.length - 1 && time >= selected[index].duration) { time -= selected[index].duration; index++; }
        const motion = selected[index], ratio = Math.max(0, Math.min(1, time / motion.duration));
        const position = ratio * (motion.points.length - 1), segment = Math.min(Math.floor(position), motion.points.length - 2);
        const fraction = position - segment, a = motion.points[segment], b = motion.points[segment + 1];
        const pose = a.map((v, i) => v + (b[i] - v) * fraction);
        car.setAttribute('transform', `translate(${pose[0]} ${pose[1]}) rotate(${pose[2] * 90})`);
        const partial = motion.points.slice(0, segment + 1).concat([pose]);
        donePath.setAttribute('d', path(selected.slice(0, index).concat(motion.kind === 'turn' ? [] : [{...motion, points: partial}])));
        $('progress').value = String(Math.round(elapsed / total * 1000));
        $('percent').textContent = `${Math.round(elapsed / total * 100)}%`;
        $('motion').textContent = elapsed >= total ? '已到达所选终点' : `${motion.label} · X ${Math.round(pose[0])} / Y ${Math.round(pose[1])} mm`;
        if (index !== activeIndex) {
            if (stepNodes[activeIndex]) stepNodes[activeIndex].classList.remove('active');
            stepNodes[index].classList.add('active'); activeIndex = index;
            if (playing) {
                const list = $('steps'), row = stepNodes[index];
                if (list.scrollHeight > list.clientHeight) list.scrollTop = row.offsetTop - list.offsetTop - list.clientHeight / 2;
                if (list.scrollWidth > list.clientWidth) list.scrollLeft = row.offsetLeft - list.offsetLeft - 20;
            }
        }
    }

    $('from').onchange = () => { destinations(); update(); };
    ['to', 'zone', 'obstacle-a', 'obstacle-b'].forEach(id => $(id).onchange = update);
    $('full').onclick = () => { $('from').value = '0'; destinations(); $('to').value = '8'; update(); };
    $('play').onclick = () => { if (elapsed >= total) { elapsed = 0; render(); } setPlaying(!playing); };
    $('reset').onclick = () => { setPlaying(false); elapsed = 0; render(); };
    $('progress').oninput = () => { setPlaying(false); elapsed = Number($('progress').value) / 1000 * total; render(); };
    function frame(now) {
        if (playing) {
            if (lastTime) elapsed = Math.min(total, elapsed + Math.min((now - lastTime) / 1000, .1) * Number($('speed').value));
            lastTime = now; render();
            if (elapsed >= total) setPlaying(false);
        }
        requestAnimationFrame(frame);
    }
    destinations(); update(); requestAnimationFrame(frame);
})();
