# F405 FOC Controller

基于 STM32F405RGTx 的双轴电机 FOC 控制固件，包含 DRV8303 驱动、MT6826S 编码器、电流采样，以及 USB / CAN 通信相关代码。

## 编译

1. 安装 Keil MDK，并配置 ARM Compiler **5.06 update 6 (build 750)**。
2. 安装工程声明的设备包 **Keil.STM32F4xx_DFP 3.0.0**。工程还声明了 ARM CMSIS 4.5.0 的 CORE 组件。
3. 打开 `MDK-ARM/F405_FOC_Ctrl.uvprojx`。
4. 选择 `F405_FOC_Ctrl` 目标，执行 Rebuild。

输出位于 `MDK-ARM/F405_FOC_Ctrl/`，不纳入版本管理。工程源码及依赖使用相对路径；HAL、CMSIS、CMSIS-DSP 和 USB Device 库已随工程保留。

`F405_FOC_Ctrl.ioc` 为 STM32CubeMX 配置。重新生成代码前请备份并检查差异，避免覆盖工程定制内容。

## 目录

- `App/`：控制算法、板级驱动、参数和通信逻辑。
- `Core/`：STM32 初始化与中断代码。
- `Drivers/`：STM32 HAL 与 CMSIS / CMSIS-DSP。
- `Middlewares/`、`USB_DEVICE/`：USB Device 库与接口。
- `MDK-ARM/`：Keil 工程、启动文件及必要配置。

板级采样与 PWM 接线说明见 `App/board/README.md`。电机与板卡参数请检查 `App/config/` 及相关板级代码。本目录仅包含固件，不包含原理图；现有文档中指向上级 Schematic 目录的链接需要额外硬件资料。

编译验证不能替代实板验证。运行前应核对电源、电流采样、PWM 极性、编码器方向及电机参数。

## 许可证

第三方代码保留各目录原有的许可证及版权声明。项目自有代码的开源许可证尚待作者选定，发布前请补充根目录 LICENSE。