# 原地转向测试与MATLAB调参

“手动”页的“90°原地转向测试”提供左转、右转和停止按钮。从当前停车朝向转90°，不平移、不自动转回，也不执行机构或舵机动作。它复用正式路线的原地转向PID、加速及完成判定，当前参数保持为轮速指令上限300RPM、指令加速度100RPM/s、20ms控制周期；进入±0.5°后停车，稳定200ms并取得稳定开始200ms之后的新IMU样本才报告完成。

测试需要连接、安全确认、空闲、新鲜IMU及正常CAN，不要求取放机构坐标有效。切页、失焦、主动断连发送停止，会话变化、主控心跳超时、急停、IMU/CAN故障和18秒转向超时中止。开始成功时清空上一趟任务日志，结束或中止保留记录，断电丢失。旧固件不支持新命令时显示未启动，不自动重发。

## 实车采集

1. 保持相同地面、载荷和机构姿态，在转动范围内留足空间。
2. 左、右各测3次，可以左右交替，逐次点击；不自动连转或复位。
3. 每趟完成后立即打开“日志→读取上次任务日志”，保存完整文本，再开始下一趟。建议命名`left1.txt`、`right1.txt`等，注明空载/带料、地面及观察到的滑移或车体中心移动。
4. 先用当前参数取得基准数据，再根据模型和实车复测结果决定提速幅度。轮速指令RPM不等于车身角速度°/s，也不是编码器实测转速。

## 日志格式

记录只写入现有32KiB RAM缓冲，不在运动中周期发送UART5文字日志。原`YAW`日志继续保留。数据量过大时仍会覆盖最旧行；分析器要求完整的起止标记，不能用已覆盖头部的记录拟合。

- `TURN_TEST`：开始时钟ms、方向（+1左/-1右）、IMU偏航极性、起始和目标角度。角度单位0.01°，归一化到±180°。
- `TURN_CFG`：实际使用的Kp、Ki、远/近目标Kd、切换角度、积分上限、轮速上限、加速度、控制周期、稳定等待、完成容差及四轮修调。后缀`_m`除以1000，`_cd`除以100；四轮trim除以1000。
- `TC ms,wheel1,wheel2,wheel3,wheel4`：最终四轮RPM指令改变时记录，包含轮速修调。时钟在整批指令发送后记录；停止命令另补零指令。未变化时驱动保持上一次指令。
- `TY ms,imu_ms,target_cd,actual_cd,error_cd,raw100,ramp100,integral1000,band`：每次控制实际采用一个新IMU样本时记录。`raw100`为限幅后、加速处理前的PID输出×100，`ramp100`为加速处理后的转向分量×100，积分除以1000。`band`为0未入容差、1已入容差、2停车稳定确认阶段。重复采样时间不重复写入。
- `TURN_END result=complete/aborted ...`：结束时间、最新IMU时间、实际角度和误差。停止后若读到了新样本，仅在结束标记中记录，避免把它误认为执行过PID的一次控制采样。

转向PID沿用原算法：Kp=2、Ki=1，误差大于15°时Kd=0.8，否则Kd=0.4；积分仅在15°内累计并限于±7。D项使用按50ms归一化的误差差分，不能把这些系数直接套用到直行PID或连续时间PID。只是将原字面常量命名供日志引用，正式转向行为未调整。

## MATLAB分析与仿真

无需额外工具箱或安装库。在MATLAB中执行：

```matlab
addpath('E:/Download/project/gongchuang/shengsai/tools');
files = ["E:/logs/left1.txt", "E:/logs/right1.txt", ...
         "E:/logs/left2.txt", "E:/logs/right2.txt", ...
         "E:/logs/left3.txt", "E:/logs/right3.txt"];
recording = analyze_turn_pid(files);
model = simulate_turn_pid(files);
```

单趟可用`analyze_turn_pid`检查角度、过冲、完成时间、角速度、反向修正、PID输出复现和完整性；输出`turn-samples.csv`、`turn-commands.csv`、`turn-summary.csv`及`turn-recording.png`。两个时间轴分别保留：角度按`imu_ms`分析，轮速指令按实际发送时钟积分，避免把刚发出的指令当作过去角度变化的输入。32位时钟回绕及角度跨越±180°均作处理。

`simulate_turn_pid`至少需要同一配置、同一载荷下左右各两条完整记录。用首次左、右转拟合，剩余试次独立验证，验证数据不参与模型选择或参数拟合。模型为按实测IMU周期离散化的航向增量ARX，搜索0～4个采样周期的命令延迟；输入按完整`TC`命令历史计算每个IMU区间的平均值。只接受稳定且偏航增益为正的模型。

原参数闭环复现及候选试算按20ms运行控制，按实测IMU周期及中位样本年龄模拟测角和可用时间，保留原PID积分/差分、限幅、加速、整数轮速和四轮修调，以及±0.5°、200ms和新样本完成条件。简化模型未单独描述轮胎滑移和各轮驱动响应；是否能用于调参由独立验证和原参数复现结果决定。

模型门槛为验证角度RMSE≤0.5°、拟合度≥80%、原参数闭环复现RMSE≤0.5°。通过后比较Kp的0.8/1/1.2倍、Ki的0/0.5/1倍、两档Kd共同乘0.8/1/1.2，以及加速度1/1.2/1.4倍。候选需完成、过冲≤1°、最终误差≤0.5°，且命令峰值和角速度不超过本批观测峰值的1.25倍，再按平均完成时间筛选。这些是离线筛选条件，不是实车安全保证或新增固件阈值。

输出`turn-model-delays.csv`、`turn-candidates.csv`、`turn-model-summary.json`及`turn-model-validation.png`。模型门槛失败时`model_accepted=false`、`recommended_pid`为空；没有候选通过时也不推荐。工具不会写固件、Flash或下发运动命令；输出候选需实车复测后再采用。

## 软件自检

Windows构建包含`turn_test`：覆盖正反方向及两种IMU极性、启动拒绝、完整结束/中止、角度跨界、命令变化记录及新鲜样本去重。下列步骤生成合成日志并验证MATLAB工具：

```powershell
New-Item -ItemType Directory -Force shengsai/build/turn-test-fixtures
./shengsai/build/test_turn_test.exe E:/Download/project/gongchuang/shengsai/build/turn-test-fixtures
```

```matlab
test_turn_pid;
```

合成记录由生产C控制器驱动已知ARX模型产生，用于检验PID复现、模型恢复、独立验证、候选筛选、车辆模型变化时拒绝推荐、异常日志拒绝、时钟回绕和图表输出。它不代表实车模型、推荐PID或提速效果。
