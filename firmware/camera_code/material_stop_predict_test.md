# 物料即将停止预测实验

在 MaixCAM2 上单独运行 `material_stop_predict_test.py`，不要运行正式 `main.py`。
脚本只打印信息、显示画面，不打开串口，不发送抓取指令，补光灯关闭。
不检查物料中心是否对准锚点；中心坐标仅用于估算运动速度。

## 运行

1. 在 MaixVision 中打开 `material_stop_predict_test.py`，可直接单文件运行，
   不需要上传 `material_layout.py`、`vision_protocol.py` 或 `vision_settings.py`。
2. 脚本顶部 `COLORS` 已设为当前料盘的黑、绿、黄 `(5, 4, 2)`，换料时修改此处。
   编号：1红、2黄、3蓝、4绿、5黑、6浅蓝。LAB阈值已从当前 `vision_settings.py`
   内置到脚本顶部 `COLOR_THRESHOLDS`；后续重新标定时需手动更新此处。
3. 使用 MaixVision 直接运行测试脚本，预热两秒后观察控制台和屏幕。
   无需修改正式应用入口、版本号或安装包。
4. 让料盘完整经历匀速、减速、停止和再次启动，记录包含 `SAMPLE` 行的完整日志；
   也可遮挡物料观察预测清除。`TRACE_MOTION=True` 默认逐帧记录坐标，采样完成后可关闭。

## 日志

以下仅为字段示例，数值不代表实机结果：

```text
STOP t_ms=5800 color=green region=bottom state=early_hint eta_ms=192.0 speed=12.0 decel=60.0 rmse=0.5 age_ms=8 still_10f=0 reason=consistent window_ms=640
SAMPLE t_ms=5792 age_ms=8 black=NA green=180,190 yellow=NA
STOP t_ms=6800 color=green region=bottom state=moving eta_ms=NA speed=0.0 decel=0.0 rmse=0.0 age_ms=8 still_10f=1 reason=below_speed window_ms=640
```

- `collecting`：运动样本不足；`lost`：该颜色当前没有实际检测结果。
- `moving`：减速模型未通过，不代表测得非零速度；`decelerating`：符合减速模型，但还未满足提前提示条件。
- `early_hint`：连续两次预计停止时刻相差不超过100ms，预测剩余时间在 `LEAD_MS` 内。
  默认350ms是实验提示范围，不是已标定的机械臂提前量，也不保证每次提前350ms。
- `forecast_expired`：采图和检测结束时预测时刻已过去，不作为提前提示。
- `still_10f=1`：该颜色最近十个实际检测帧，两轴波动各不超过8协议像素、前后五帧
  中位数差各不超过2协议像素。与正式布局使用相同的静止数值条件，但这里逐色独立观察，
  不做整组三色布局确认。它也不是真实零速度的测量。此项现在是独立对照字段，
  不再作为 `state` 或提前结束预测；即使为1，仍继续估计速度及减速趋势。
- `CANCEL`：上次提示为 `early_hint`，本次未继续满足预测条件。提前提示不是锁存的抓取许可。

`region` 为**当前画面**的 `bottom`（下半区）、`upper_left`、`upper_right`，
不推断最终停靠区域；上半区正中分界为 `upper_boundary`，不发提前提示。
分区使用现有320×240协议坐标；预览为640×480，绘制时放大两倍。
`speed` 为拟合的当前速度，`decel` 为减速度，单位分别为协议像素/秒、协议像素/秒²；
`rmse` 是速度拟合残差，不是预测正确率。
`eta_ms` 已扣除本轮从读取图像前到打印前的耗时，`age_ms` 记录该耗时，
不包含相机内部未知的缓存/曝光延迟。`t_ms` 是单调时钟毫秒时间，不是日历时间。

`reason` 说明模型状态：`samples` 样本不足、`missing` 目标缺失、`below_speed` 速度过低、
`decel_small` 减速不足、`drop_small` 速度降幅不足、`fit_residual` 拟合残差过大、
`outside_lead` 停止时间超出提示范围、`await_confirmation` 等待下一次预测、
`deadline_changed` 停止时刻不一致、`consistent` 连续预测一致、
`region_boundary` 区域分界、`processing_delay` 预测在处理过程中已过期。
`window_ms` 是实际参与速度拟合的窗口跨度。

`SAMPLE` 每帧记录一次，包含采图前的 `t_ms`、到该行输出前的 `age_ms` 和
三色实际输入坐标，`NA` 表示该帧没有该颜色。坐标仍为320×240单位，
用于离线重放预测，不新增中心对准门槛。记录覆盖预测的降采样间隙和目标丢失，
用于判断减速时长、坐标抖动及模型窗口是否混入匀速阶段。

## 模型与验证边界

速度每隔至少60ms采样，用隔一个样本的位移估速，最多保留最近九个样本，
至少七个样本且跨度达到350ms后做速度对时间的直线拟合。
只有速度明显下降、拟合残差合格、当前速度和减速度超过实验阈值时，
才按短时近似恒定减速度外推停止时间。目标丢失、颜色更换、间隔超过250ms或
单步突跳超过50协议像素会清空历史。十帧静止对照独立使用每个相机帧。

此方法需要减速过程在图像中可见；如果直接急停、减速太短、中心噪声太大、
透视导致图像速度明显变化，可能不给提示或产生错误预测。
当前阈值仅用于相机实验，尚未实机标定；正式视觉、协议、主控和机械动作不变。
先比较 `early_hint`、撤销提示和后续 `still_10f=1` 的时序，
真实停止时刻及误差需结合外部录像/人工标注判断，不能把十帧确认时刻当作真实停止时刻。

电脑离线测试：

```powershell
python -m unittest discover -s shengsai/firmware/camera_code -p test_material_stop_predict.py -v
```
