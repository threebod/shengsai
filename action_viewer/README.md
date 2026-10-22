# 动作实验室

本地 Three.js 机构动作组俯视回放。只读当前固件源码，不打开串口，不连接、控制或写入硬件。

在 `shengsai` 目录运行：

```powershell
./tools/start-action-viewer.ps1
```

然后打开 <http://127.0.0.1:8765>。按 Ctrl+C 关闭服务。端口占用时使用 `-Port 8766`。

首次启动需要已有 Node.js/npm 和 Python；启动器将 Three.js 安装到本项目 `action_viewer/node_modules`，npm 缓存放在 `.deps/npm-cache`，不向 C 盘安装依赖。已有 GCC 会用于导出动作，优先查找 PATH，其次使用本机已有的 `E:/Qt/Tools/mingw1310_64/bin/gcc.exe`。依赖安装成功后可离线运行，无需 CDN。服务器仅监听本机127.0.0.1。

## 使用

- 左侧选择原料夹取、粗加工放置/回收或两层暂存，切换任务顺序。原料动作可选择三个识别位置与三个转盘槽位，回收可选择两轮的不同水平坐标。
- 播放、暂停、倍速、循环、拖动时间轴；点步骤编号或上一步/下一步可检查单步。零耗时命令也能单独选中查看。
- 默认正交俯视；“斜视”查看升降。滚轮缩放，拖动平移，斜视时右键可旋转。右侧同步显示水平位置、升降深度及三路舵机指令角度。
- 改动固件后点击“重读固件”。服务重新编译导出器，直接调用当前的 `rawPickActionsForPosition`、`missionBuildTaskPlace` 和 `missionBuildTaskPickBack`，不另维护动作JSON副本。

## 数据与边界

动作目标、速度、等待、圆环位置和放置深度来自当前工作区源码。C导出器复用固件的步进电机耗时与云台平滑时长计算；网页按指令回放，包含下层物料先水平接近、转盘非阻塞运动、落盘前等待转盘及原料落盘后延迟500ms切槽。原料夹取、三物料放置/回收分别展示，不执行整场比赛路线、视觉采样或实时到位确认。

各动作从其声明的初始姿态开始。底盘固定，模型机械尺寸、云台零角方向、物料与槽位的几何关系是示意，未按实物标定；物料显示仅帮助理解夹取/放置顺序。回放不预测碰撞，不代表反馈到位时间，不能替代实机检查。参数取源码默认值，不读取主控Flash。页面没有修改或固化控制参数的接口。

## 检查

```powershell
cd action_viewer
npm test
```

覆盖39个变体、轨迹终点、插入的水平接近阶段、夹持等待、转盘延后切槽及云台平滑插值。浏览器需支持WebGL2。

Three.js固定版本见 `package-lock.json`，MIT许可证随本地依赖保留。使用[正交相机](https://threejs.org/docs/pages/OrthographicCamera.html)和[WebGLRenderer](https://threejs.org/docs/pages/WebGLRenderer.html)。
