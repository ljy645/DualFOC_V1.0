# VECTOR FOC 双轴控制台

双击 `start.bat`。需要 Python 3.10 或更新版本；首次运行缺少依赖时自动安装 pySerial 与 Qt Widgets。
可用 `start.bat --demo` 离线演示，界面明确显示“非实机”，不会打开串口或操作电机。

界面使用 Qt 抗锯齿绘制。参考 [VESC 的圆形仪表](https://github.com/vedderb/vesc_tool/blob/master/mobile/CustomGauge.qml)
及 [ODrive 的分栏控制布局](https://odriverobotics.com/webgui)，没有复制其代码或图像资源。
三个导航按钮固定尺寸；每个电机页只有一个大仪表，切换速度/位置模式时共用，不需要整页滚动。

## 使用

1. 烧录本工程配套固件。关闭 VOFA 对同一 COM 口的占用，选择 USB 串口并连接。
2. 上位机自动发送 `host on` 并读取两轴参数，连接不使能电机。
3. “电机固有参数”单独保存，要求两轴关闭，保存后需重新校准；Ld/Lq 输入 µH、磁链输入 mWb，发送时自动换算为 SI 单位。“调试参数”单独保存速度Kp、Ki及补偿强度，允许运行中写入，不清除校准或补偿表。
4. 两轴输出均关闭后，点一次“完整校准”：先缩短扫描找零/方向，再进行齿槽标定，最后统一保存。界面显示当前阶段；结束后仍关闭输出。
5. 点击使能，单片机确认成功后显示实际使能状态。填写目标并发送，或拖动橙色目标指针，松开后发送。
6. 速度范围为 -3000～3000 rpm。位置模式提供0～360°的单仪表预览和输入接口；固件未实现位置闭环，界面不发送角度运动。

## 齿槽补偿

先保存电机参数，关闭两轴输出，空载点击唯一的“完整校准”。
界面显示“1/2 找零与方向校准”和“2/2 齿槽标定”。找零约5.2秒；第二阶段正反各10 rpm、各3圈，按机械角位移得到的500 ms平均转速连续两个窗口进入目标±2 rpm才采集，扫描约36秒另加稳定等待。每方向等待限10秒，三圈扫描另有独立30秒限时。
窗口不会阻塞，程序内部继续服务控制和反馈，返回后保持输出关闭。
找零失败不执行齿槽标定；找零中间结果只留RAM，齿槽失败或中途掉电时仍为未完成校准，需要重新完整校准。EEPROM存储失败另报错误，须读取参数确认。
成功后显示“补偿表已标定 · 1024 点”，表及CRC保存在原先预留的EEPROM区域，数据格式仍是V1。
有效表默认参与速度控制，无需勾选。首次可把“齿槽补偿强度”填0.5，在调试参数区点“写入并保存”，同速对比；强度0关闭，也允许大于1。
三个调试值共用独立EEPROM页，可运行中保存，保留电机零位、表和速度目标；“保存电机参数”仍会使校准和表失效。
补偿只在速度模式作用，直接id/iq调试不加补偿。不能带外部周期负载学习，也不保证对所有电机都改善。
HOST的完整校准和独立齿槽标定请求等待上限均为120秒；过程中遥测会暂停，但会发布阶段进度，等待最终OK/ERR，不重发标定命令。

```text
calibrate 0
tune 0 0.25 1 0.5
```

蓝色指针为实际反馈，橙色指针为目标。未知/过期反馈显示“—”，不会用零值冒充测量。
电机固有参数保存和校准要求两轴关闭；调试参数可运行中独立保存。运行中的校准按钮不可用，固件同时检查此条件。
断开串口不隐式改变电机输出；先关闭所需轴再断开连接。

USB页有串口、波特率、数据位、校验、停止位、流控及连接状态；下方显示双轴趋势、历史数据和命令应答。
采集数据自动存入 `history/session_*.csv`，演示数据文件有 `DEMO_` 前缀；支持导出和重新打开历史文件。
`settings.json` 保存串口选择及最近连接时间，不保存任何使能或运动指令，不会自动重放。

## 配套协议

USB CDC，下行ASCII文本，空格分隔、实际换行结束。轴号固定0/1，先等应答再发下一条。
应用串口读写在独立线程执行，窗口线程不等待校准；请求超时会断开连接，不自动重发运动指令。
pySerial接口依据[官方API](https://pyserial.readthedocs.io/en/stable/pyserial_api.html)。

```text
host on                 -> OK HOST 1
enable 0 1              -> OK ENABLE axis=0 state=1
enable 0 0              -> OK ENABLE axis=0 state=0
enable 1 1              -> OK ENABLE axis=1 state=1
speed 0 100             -> OK
speed 1 -100            -> OK
param show 0            -> OK axis=0 ... / 多行 key=value / END
param set 0 ld_h 0.00005 -> OK STAGED
param save 0            -> OK
calibrate 0             -> P CALIBRATION ZERO / P CALIBRATION COGGING / OK CALIBRATED_FULL
host off                -> OK WAVEFORM
```

使能失败返回 `ERR ENABLE axis=0 state=0 reason=...`，原因含UNSAVED、UNSAVED_CHANGES、UNCALIBRATED、FAULT、NOT_READY。
参数或校准失败返回ERR，本批后续指令停止；先查看通信记录再决定如何处理。
`param set` 只暂存，`param save` 写EEPROM、读回核对并应用；保存会清零该轴校准及补偿有效状态。
完整校准先保存未校准状态，找零成功只更新RAM，齿槽采集成功后统一保存零位、补偿表和有效位，结束保持输出关闭。
只有 `OK CALIBRATED_FULL` 表示完整成功，旧固件的 `OK CALIBRATED` 会被上位机拒绝为不支持完整校准，避免误报。
高级调试可单独发送 `cogging calibrate <axis>` 重做齿槽阶段。使能/速度命令不写EEPROM。

HOST模式两轴各50 Hz遥测，一行格式为：

```text
T axis enabled calibrated faults rpm target_rpm position_deg bus_v id_a iq_a
```

故障位为十进制无符号整数；测量无效发送nan；position_deg为编码器原始单圈机械角度。
软件不把它当作已经建立机械零点的位置闭环反馈。
握手前可能有在途VOFA二进制帧，软件会等到完整 `OK HOST 1` 后才解析文本。

## VOFA兼容

默认仍上传JustFloat，帧尾 `00 00 80 7F`。前12个通道及顺序保留；新增：

| 通道 | 内容 |
| --- | --- |
| 12 | M0实际使能，0/1 |
| 13 | M1实际使能，0/1 |
| 14 | 最近使能命令轴号，初值-1 |
| 15 | 最近使能结果：0未收到、1成功、-1失败 |

VOFA可直接发 `enable 0 1`、`enable 1 0`、`speed 0 100` 等指令，波形模式的结果通过上述通道反馈，避免混入ASCII破坏波形。
自研上位机使用HOST文本模式获得明确OK/ERR，同时显示两轴。两个程序轮流占用串口，不能同时打开同一COM口。

## 文件

- `main.py`：三个页面、控制操作、参数编辑和CSV历史。
- `widgets.py`：单个双指针仪表、趋势图。
- `link.py`：串口线程、HOST握手、逐条请求/应答、遥测解析。
- `demo_device.py`：显式离线演示设备，不访问硬件。
- `start.bat` / `requirements.txt`：一键启动和依赖。

已完成的软件验证：

- 使用本机Windows字体离屏渲染三个页面和位置模式；1060×690、1280×800下，参数/使能/校准按钮均在窗口内，导航按钮尺寸不随选中变化。
- Qt交互验证两轴参数单位换算、保存/读取、停机校准、校准后保持关闭、使能应答、手动速度、鼠标拖动目标指针、位置模式仅预览及CSV导出/打开。
- 将当前固件command/config_store/usb_hostif源码在临时原生环境中编译，与Python SerialLink联调，验证分包接收、HOST握手、双轴命令、参数状态、错误应答和校准前停机条件。
- 检查VOFA帧为16个float＋4字节帧尾，使能成功/失败通过附加通道返回，不混入ASCII。
- `start.bat --help`、Python编译检查通过，Keil编译0错误/0警告。

上述协议验证模拟了HAL、测量和EEPROM设备；不代表EEPROM物理读写或真实电机已完成实板验证。
