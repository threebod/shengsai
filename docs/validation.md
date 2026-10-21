# v0.1.0-preview.1 验证记录

日期：2026-10-21。以下仅为软件与构建证据，不代表完成实车或安卓实机验收。

| 检查 | 结果 |
| --- | --- |
| Windows Release CMake构建 | 通过，Qt6.11.1 / MinGW13.1 |
| 新工程CTest | 12/12通过 |
| 摄像头主目录Python unittest | 112/112通过 |
| K230 Python unittest | 40/40通过 |
| Keil完整重建 | 0错误、2条已有第三方警告 |
| Android arm64 Release原生库及APK | 构建成功，NDKr27c / JDK21 / API36 / Gradle9.3.1 |
| APK签名 | RSA3072固定预发布签名，apksigner验证通过 |
| APK对齐 | zipalign 16KB校验通过 |
| Windows部署包启动 | 清除开发Qt/QML路径，仅保留部署目录和Windows系统PATH后启动并截图成功 |
| 真实QML布局 | 桌面、窄屏各四页截图成功，无QML启动警告；已检查字段与卡片对齐 |
| 旧上位机完整CTest | 现有build目录43/44通过；旧参考文件缺失导致1项静态检查失败 |

新CTest覆盖：Controller握手/确认、先读取后固化、失败不显示保存成功、比例范围、拖动目标合并、失焦点动停止、不自动重发运动；CRC和拆粘包；CAN FIFO排空、按轴/类型缓存、故障锁存；生产机构运行服务与Flash记录。

机构运行服务测试直接包含固件的`mechanism_runtime.h`，核对超过旧估算截止时间仍不提前停止、上次目标与到位标志不生效、双轴分别到位、到位后实际位置采样、云台结束门槛、超时/堵转/CAN故障使坐标失效。正常完成路径不发送提前停止。

比例Flash测试核对范围、重复值不重复写、提交标记最后写入、中断记录不覆盖旧有效记录、保存回读、空间用尽时保留上一有效比例。

Keil两条已有第三方警告：`vendor/LED/LED.h`末尾缺换行；`vendor/SYSTEM/delay/delay.c`未使用的`wait_ms`缺返回值。未为本任务改动这些原有驱动行为。程序Code81836、RO-data4364、RW-data824、ZI-data17448字节。链接器静态最大调用栈为980字节加未知间接调用项；启动栈已由1KB增至4KB，仍需实机验证中断及峰值余量。

旧CTest唯一失败项`test_task12_host_static`需要工作区原本缺失的`yyb_stm32/host_protocol/host_uart3.h`。旧工程的源码和既有未提交修改保持原样；没有为绕过该项而改变旧工程。

## 界面与预览

- [Imagegen桌面预览](design/desktop-preview-v1.png)、[Imagegen安卓预览](design/android-preview-v1.png)。用户已认可设计方向。
- [Windows真实比赛页](design/desktop-page-1.png)、[圆环页](design/desktop-page-2.png)、[视觉页](design/desktop-page-3.png)、[手动页](design/desktop-page-4.png)。
- [窄屏比赛页](design/mobile-page-1.png)、[圆环页](design/mobile-page-2.png)、[视觉页](design/mobile-page-3.png)、[手动页](design/mobile-page-4.png)。窄屏图是Windows程序布局仿真，非安卓实机截图。
- [独立部署包启动截图](design/windows-portable-runtime.png)。

截图未连接主控，所以运动和保存控件禁用。圆环显示待读取，比例显示固件初值并明确标记尚未读取，不将概念图示例值当成主控读数。

## 尚需硬件验收

没有烧录或发送实机运动指令。以下必须由实车联调补齐：

1. 映射、方向、机械限位和急停，机构实际上电零点及云台/转盘启动净空。
2. 5/6轴低速短行程、长行程、双轴、带负载重复往返；对照日志里的输入目标、编码器位置、到位状态及外部测量。
3. CAN断线、驱动禁用/堵转、急停、蓝牙中断与窗口切换时的真实停止行为。
4. 三组圆环保存后实际掉电恢复；2号视觉比例应用、保存、掉电及离线启动使用。
5. Android9+真实手机权限拒绝、HC-05配对/扫描/重连、后台切换和触控拖动。
6. 两种启停区、障碍路线、两轮任务及全动作组在实车上的完整验证。

只有硬件验证后才能判断剩余偏差是否来自驱动跟随、传动打滑、零点或其他原因。本版消除了可见的按时间提前截断逻辑，不能将所有原有偏差归因于遗漏脉冲。
